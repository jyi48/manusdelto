#!/usr/bin/env python3
import os

import numpy as np
import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand
from manus_ros2_msgs.msg import ManusGlove
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from manus_tesollo.retargeters import build_retargeters, DEFAULT_JOINT_CALIB

CALIB_PHASE_SEC = 4.0  # seconds per phase — matches scm_gui's CALIB_DURATION
_OPEN_RAMP_HZ = 50.0   # ramp update rate for the gentle "open hand"

try:
    from manus_tesollo.dg5f_kinematics import DG5FKinematics

    _KIN_AVAILABLE = True
except Exception:
    _KIN_AVAILABLE = False

LEFT_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]
RIGHT_JOINT_NAMES = [n.replace("lj_", "rj_") for n in LEFT_JOINT_NAMES]

# DG5F-S (hand_model:=s). Same 5x4 layout as the M, so every retargeter runs
# unchanged -- only the names and topic wiring differ. The S ships no both-hand
# launch: each hand is its own namespace (dg5f_s_left / dg5f_s_right) and BOTH
# sides carry these identical names, which is why joint_states has to be tagged
# per topic instead of matched by name.
S_JOINT_NAMES = [
    f"joint_{finger}_{joint}"
    for finger in range(1, 6)
    for joint in range(1, 5)
]


class ManusTesolloNode(Node):
    def __init__(self):
        super().__init__("manus_tesollo")

        def _p(name, default):
            return (
                self.declare_parameter(name, default).get_parameter_value().string_value
            )

        self._hand_ns = _p("hand_ns", "dg5f_both")          # DG5F-M (both hands)
        self._s_left_ns = _p("s_left_hand_ns", "dg5f_s_left")    # DG5F-S, per hand
        self._s_right_ns = _p("s_right_hand_ns", "dg5f_s_right")
        left_in = _p("left_input_topic", "/manus_glove_0")
        right_in = _p("right_input_topic", "/manus_glove_1")
        # Empty = derive the reference topic from hand_model; a non-empty value
        # pins it and survives a model switch.
        self._out_override = {
            "left": _p("left_output_topic", ""),
            "right": _p("right_output_topic", ""),
        }

        # Joint names, reference publishers and joint_states subscriptions all
        # depend on which hand is attached, so they live in _wire_hand_model()
        # and get rebuilt when the GUI flips the `hand_model` parameter.
        self._pubs = {"left": None, "right": None}
        self._js_subs = []
        self._names = {}
        self._hand_model = None
        self._wire_hand_model(_p("hand_model", "s"))

        self.create_subscription(ManusGlove, left_in, self._cb, 10)
        self.create_subscription(ManusGlove, right_in, self._cb, 10)
        self.create_subscription(
            String, "/teleop/mirror_mode", self._cb_mirror_mode, 10
        )
        self.create_subscription(
            String, "/manus_tesollo/retarget_mode", self._cb_retarget_mode, 10
        )
        self._mirror_mode = False
        self._paused = False
        self._prev_vals = {"left": [0.0] * 20, "right": [0.0] * 20}

        # Gentle "open hand" ramp state. open_ramp_sec = 0 -> instant.
        self._open_ramp_sec = (
            self.declare_parameter("open_ramp_sec", 1.5)
            .get_parameter_value().double_value
        )
        self._open_timer = None
        self._opening = False   # gate the paused-hold publish while ramping

        self.create_service(SetBool, "/manus_tesollo/pause", self._cb_pause)
        self.create_service(SetBool, "/manus_tesollo/set_ik_mode", self._cb_set_ik_mode)
        self.create_service(Trigger, "/manus_tesollo/open_hand", self._srv_open_hand)

        # --- IK / spatial-mapping parameters ---
        pos_w = (
            self.declare_parameter("position_weight", 1.0)
            .get_parameter_value()
            .double_value
        )
        # ori_w must be > 0: each finger has 4 DOF but position constrains only 3,
        # leaving a free null-space DOF that wanders without regularisation (trembling).
        ori_w = (
            self.declare_parameter("orientation_weight", 1.0)
            .get_parameter_value()
            .double_value
        )
        max_itr = (
            self.declare_parameter("max_ik_iterations", 3)
            .get_parameter_value()
            .integer_value
        )
        ik_tol = (
            self.declare_parameter("ik_tolerance", 1e-3)
            .get_parameter_value()
            .double_value
        )
        # Uniform map: Manus glove-local tip position → DG5F base frame.
        # Applied identically to all five fingers (thumb included).
        marker_scale = (
            self.declare_parameter("marker_scale", (2580.0 - 486.0) / (2500.0 - 486.0))
            .get_parameter_value()
            .double_value
        )
        base_offset = np.array(
            [
                self.declare_parameter("base_offset_x", 0.0)
                .get_parameter_value()
                .double_value,
                self.declare_parameter("base_offset_y", 0.0)
                .get_parameter_value()
                .double_value,
                self.declare_parameter("base_offset_z", 0.0486)
                .get_parameter_value()
                .double_value,
            ]
        )
        center_offset = np.array(
            [
                self.declare_parameter("center_offset_x", 0.0)
                .get_parameter_value()
                .double_value,
                self.declare_parameter("center_offset_y", 0.0)
                .get_parameter_value()
                .double_value,
                self.declare_parameter("center_offset_z", 0.03)
                .get_parameter_value()
                .double_value,
            ]
        )
        model_var = (
            self.declare_parameter("dg5f_model", "m").get_parameter_value().string_value
        )

        # DG5FKinematics is owned by the node: it supplies the model (and its
        # internal curl-only joint limits) used by the IK retargeter's CLIK solve.
        self._kin = {}
        if _KIN_AVAILABLE:
            for side in ("left", "right"):
                self._kin[side] = DG5FKinematics(
                    side, model=model_var, logger=self.get_logger()
                )

        ik_params = dict(
            pos_w=pos_w,
            ori_w=ori_w,
            max_itr=max_itr,
            ik_tol=ik_tol,
            marker_scale=marker_scale,
            base_offset=base_offset,
            center_offset=center_offset,
        )
        # dex-retargeting needs the URDF directory (loads dg5f_{left,right}.urdf)
        # and the DexPilot config directory (installed to share/).
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("manus_tesollo")
            urdf_dir = os.path.join(share, "urdf")
            config_dir = os.path.join(share, "retargeters", "configs")
        except Exception:
            urdf_dir = ""
            config_dir = None
        dex_params = dict(urdf_dir=urdf_dir, config_dir=config_dir)

        self._retargeters = build_retargeters(
            kin=self._kin,
            ik_params=ik_params,
            dex_params=dex_params,
            joint_calib=list(DEFAULT_JOINT_CALIB),
            logger=self.get_logger(),
        )
        # _wire_hand_model() ran before the retargeters existed, so hand the
        # model over now; ergo clamps to a different limit table per model.
        self._retargeters["ergo"].set_hand_model(self._hand_model)

        use_ik = (
            self.declare_parameter("use_ik", False).get_parameter_value().bool_value
        )
        self._mode = "ik" if (use_ik and "ik" in self._retargeters) else "ergo"
        if use_ik and "ik" not in self._retargeters:
            self.get_logger().warn(
                "use_ik=True but IK unavailable — falling back to ergo"
            )

        # 2-phase (rest/fist) calibration, driven by a wall timer. Requires
        # mode=='ergo' since that's the retargeter whose compute() is being
        # called to sample ergonomics (ik shares the same instance as its
        # seed, so calibrating while in ik mode also works; dex does not).
        self._calib_step = 0  # 0=idle, 1=rest phase running, 2=fist phase running
        self._calib_timer = None
        self.create_service(Trigger, "/manus_tesollo/calibrate", self._srv_calibrate)

        # Live-tunable knobs, exposed via the node's standard parameter
        # service (rcl_interfaces/SetParameters -- every rclpy node has one
        # at ~/set_parameters, no custom service needed). Applied immediately
        # to the already-built retargeter instances; see set_scaling/
        # set_low_pass_alpha/set_calib -- these mutate plain attributes the
        # retargeters re-read every frame, so no restart is needed.
        self.declare_parameter("dex_scaling_factor", 1.1)
        self.declare_parameter("dex_low_pass_alpha", 0.1)
        self.declare_parameter("ergo_calib", list(DEFAULT_JOINT_CALIB))
        self.declare_parameter("mirror_reflect_axis", "x")
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"left  {left_in} -> {self._pubs['left'].topic_name}")
        self.get_logger().info(
            f"right {right_in} -> {self._pubs['right'].topic_name}")
        self.get_logger().info(f"mode: {self._mode}")

    def _cb_mirror_mode(self, msg: String):
        self._mirror_mode = msg.data == "mirror"
        self.get_logger().info(f"mirror_mode -> {self._mirror_mode}")

    def _cb_set_ik_mode(self, req: SetBool.Request, res: SetBool.Response):
        if req.data and "ik" not in self._retargeters:
            res.success = False
            res.message = "IK unavailable"
            return res
        self._mode = "ik" if req.data else "ergo"
        res.success = True
        res.message = f"mode: {self._mode}"
        self.get_logger().info(f"set_ik_mode: {res.message}")
        return res

    def _srv_calibrate(self, req: Trigger.Request, res: Trigger.Response):
        if self._mode not in ("ergo", "ik"):
            res.success = False
            res.message = "select ergo (or ik) mode first"
            return res
        ergo = self._retargeters["ergo"]
        for side in ("left", "right"):
            ergo.start_rest_capture(side)
        self._calib_step = 1
        if self._calib_timer is not None:
            self._calib_timer.cancel()
        self._calib_timer = self.create_timer(CALIB_PHASE_SEC, self._on_calib_timer)
        res.success = True
        res.message = "calibration started -- open hand fully and hold"
        self.get_logger().info("calibrate: rest phase started")
        return res

    def _on_calib_timer(self):
        ergo = self._retargeters["ergo"]
        if self._calib_step == 1:
            for side in ("left", "right"):
                ergo.finish_capture(side)
                ergo.start_fist_capture(side)
            self._calib_step = 2
            self.get_logger().info("calibrate: fist phase started")
        elif self._calib_step == 2:
            for side in ("left", "right"):
                ergo.finish_capture(side)
            self._calib_step = 0
            self._calib_timer.cancel()
            self._calib_timer = None
            self.get_logger().info("calibrate: complete")

    def _cb_retarget_mode(self, msg: String):
        mode = (msg.data or "").strip().lower()
        if mode not in self._retargeters:
            self.get_logger().warn(
                f"retarget_mode '{mode}' unavailable "
                f"(have: {sorted(self._retargeters)})"
            )
            return
        self._mode = mode
        self.get_logger().info(f"retarget_mode -> {self._mode}")

    def _on_param_change(self, params):
        """Push live-tunable params into the already-built retargeter
        instances. dex_scaling_factor/dex_low_pass_alpha apply to both dex
        variants ('dex' and 'dex_vector') uniformly -- they're independent
        DexRetargeter instances (one per optimizer), not two views of one."""
        for p in params:
            if p.name == "dex_scaling_factor":
                for name in ("dex", "dex_vector"):
                    rt = self._retargeters.get(name)
                    if rt is not None:
                        rt.set_scaling(p.value)
                self.get_logger().info(f"dex_scaling_factor -> {p.value}")
            elif p.name == "dex_low_pass_alpha":
                for name in ("dex", "dex_vector"):
                    rt = self._retargeters.get(name)
                    if rt is not None:
                        rt.set_low_pass_alpha(p.value)
                self.get_logger().info(f"dex_low_pass_alpha -> {p.value}")
            elif p.name == "ergo_calib":
                if len(p.value) != 20:
                    return SetParametersResult(
                        successful=False,
                        reason=f"ergo_calib needs exactly 20 values, got {len(p.value)}",
                    )
                self._retargeters["ergo"].set_calib(p.value)
                self.get_logger().info("ergo_calib updated")
            elif p.name == "mirror_reflect_axis":
                axis = str(p.value).lower()
                if axis not in ("none", "x", "y", "z"):
                    return SetParametersResult(
                        successful=False,
                        reason=f"mirror_reflect_axis must be none/x/y/z, got '{axis}'",
                    )
                for name in ("dex", "dex_vector"):
                    rt = self._retargeters.get(name)
                    if rt is not None:
                        rt.set_mirror_reflect(axis)
                self.get_logger().info(f"mirror_reflect_axis -> {axis}")
            elif p.name == "hand_model":
                try:
                    self._wire_hand_model(p.value)
                except ValueError as ex:
                    return SetParametersResult(successful=False, reason=str(ex))
        return SetParametersResult(successful=True)

    def _cb_pause(self, req: SetBool.Request, res: SetBool.Response):
        self._paused = req.data
        res.success = True
        res.message = "paused" if self._paused else "resumed"
        self.get_logger().info(f"manus_tesollo: {res.message}")
        return res

    def _wire_hand_model(self, model):
        """(Re)wire joint names, reference publishers and joint_states subs for
        the attached hand. Safe to call at runtime: the GUI flips the
        `hand_model` parameter, mirroring how RBY1 switches robot_model."""
        model = (model or "m").strip().lower()
        if model not in ("m", "s"):
            raise ValueError(f"hand_model must be 'm' or 's', got '{model}'")
        if model == self._hand_model:
            return

        if model == "s":
            # Both S hands publish the SAME joint names in different
            # namespaces, so each joint_states topic is tagged with its side.
            # Matching by name here would let the left hand's state also land
            # in _actual["right"] and ramp open-hand from the wrong pose.
            names = list(S_JOINT_NAMES)
            self._names = {"left": names, "right": list(names)}
            out = {
                "left": f"/{self._s_left_ns}/joint_pospid/reference",
                "right": f"/{self._s_right_ns}/joint_pospid/reference",
            }
            js = [
                (("left",), f"/{self._s_left_ns}/joint_states"),
                (("right",), f"/{self._s_right_ns}/joint_states"),
            ]
        else:
            self._names = {
                "left": list(LEFT_JOINT_NAMES),
                "right": list(RIGHT_JOINT_NAMES),
            }
            out = {
                "left": f"/{self._hand_ns}/lj_dg_pospid/reference",
                "right": f"/{self._hand_ns}/rj_dg_pospid/reference",
            }
            # One broadcaster carries both hands; lj_/rj_ prefixes separate them.
            js = [(("left", "right"), f"/{self._hand_ns}/joint_states")]

        for side in ("left", "right"):
            if self._pubs[side] is not None:
                self.destroy_publisher(self._pubs[side])
            self._pubs[side] = self.create_publisher(
                MultiDOFCommand, self._out_override[side] or out[side], 10)

        for sub in self._js_subs:
            self.destroy_subscription(sub)
        self._js_subs = [
            self.create_subscription(
                JointState, topic,
                lambda msg, s=sides: self._cb_joint_states(msg, s), 10)
            for sides, topic in js
        ]

        # Measured poses belong to the previous hand -- drop them so an open-hand
        # ramp falls back to the last command until fresh state arrives.
        self._actual = {"left": None, "right": None}
        self._hand_model = model
        # ergo clamps to per-model joint limits; the S stops short of the M in
        # six places and commanding past a stop is stall current, not a wrong
        # pose. Retargeters may not exist yet on the constructor's first call.
        ergo = getattr(self, "_retargeters", {}).get("ergo")
        if ergo is not None:
            ergo.set_hand_model(model)
        self.get_logger().info(
            f"hand_model -> DG5F-{model.upper()}: "
            f"left {self._pubs['left'].topic_name}, "
            f"right {self._pubs['right'].topic_name}")

    def _cb_joint_states(self, msg: JointState, sides=("left", "right")):
        pos = dict(zip(msg.name, msg.position))
        for side in sides:
            names = self._names[side]
            if all(n in pos for n in names):
                self._actual[side] = [float(pos[n]) for n in names]

    def _publish_vals(self, side, vals):
        names = self._names[side]
        pub = self._pubs[side]
        out = MultiDOFCommand()
        out.dof_names = names
        out.values = list(vals)
        out.values_dot = [0.0] * len(names)
        pub.publish(out)

    def _srv_open_hand(self, req: Trigger.Request, res: Trigger.Response):
        # Ramp both hands from their current pose to all-zeros (open) over
        # open_ramp_sec so the fingers don't snap open. Pause so the glove
        # doesn't fight; the ramp timer owns publishing until it finishes, then
        # the paused-hold path keeps republishing zeros. Resume via
        # /manus_tesollo/pause False to hand control back to the glove.
        self._paused = True
        self._open_start = {
            side: list(self._actual[side]) if self._actual[side] is not None
            else list(self._prev_vals[side])
            for side in ("left", "right")
        }
        if self._open_timer is not None:
            self._open_timer.cancel()
        if self._open_ramp_sec <= 0.0:
            self._finish_open()
            res.message = "hand opened (zeros, instant); stream paused"
        else:
            self._open_t0 = self.get_clock().now()
            self._opening = True
            self._open_timer = self.create_timer(1.0 / _OPEN_RAMP_HZ, self._on_open_tick)
            res.message = (f"opening hand over {self._open_ramp_sec:.1f}s; "
                           "stream paused -- resume to teleop")
        res.success = True
        self.get_logger().info(f"open_hand: {res.message}")
        return res

    def _on_open_tick(self):
        elapsed = (self.get_clock().now() - self._open_t0).nanoseconds * 1e-9
        t = min(1.0, elapsed / self._open_ramp_sec) if self._open_ramp_sec > 0 else 1.0
        for side in ("left", "right"):
            start = self._open_start[side]
            vals = [(1.0 - t) * start[i] for i in range(len(start))]  # target = 0
            self._prev_vals[side] = list(vals)
            self._publish_vals(side, vals)
        if t >= 1.0:
            self._finish_open()

    def _finish_open(self):
        zeros = [0.0] * 20
        self._prev_vals["left"] = list(zeros)
        self._prev_vals["right"] = list(zeros)
        for side in ("left", "right"):
            self._publish_vals(side, zeros)
        self._opening = False
        if self._open_timer is not None:
            self._open_timer.cancel()
            self._open_timer = None

    def _cb(self, msg: ManusGlove):
        side = (msg.side or "").lower()
        if side not in ("left", "right"):
            self.get_logger().warn(f"unknown side: {side}")
            return

        compute_side = (
            "right"
            if (self._mirror_mode and side == "left")
            else "left" if (self._mirror_mode and side == "right") else side
        )

        if self._paused:
            # While an open-hand ramp is running, its timer owns publishing --
            # don't also emit here or the two fight on the reference topic.
            if not self._opening:
                self._publish_vals(compute_side, self._prev_vals[compute_side])
            return

        ergo = {}
        try:
            for e in msg.ergonomics:
                ergo[e.type] = float(e.value)
        except Exception as ex:
            self.get_logger().warn(f"ergonomics parse error: {ex}")
            return

        q_deg = [
            ergo.get("ThumbMCPSpread", 0.0),
            ergo.get("ThumbMCPStretch", 0.0),
            ergo.get("ThumbPIPStretch", 0.0),
            ergo.get("ThumbDIPStretch", 0.0),

            ergo.get("IndexSpread", 0.0),
            ergo.get("IndexMCPStretch", 0.0),
            ergo.get("IndexPIPStretch", 0.0),
            ergo.get("IndexDIPStretch", 0.0),

            ergo.get("MiddleSpread", 0.0),
            ergo.get("MiddleMCPStretch", 0.0),
            ergo.get("MiddlePIPStretch", 0.0),
            ergo.get("MiddleDIPStretch", 0.0),

            ergo.get("RingSpread", 0.0),
            ergo.get("RingMCPStretch", 0.0),
            ergo.get("RingPIPStretch", 0.0),
            ergo.get("RingDIPStretch", 0.0),

            ergo.get("PinkySpread", 0.0),
            ergo.get("PinkyMCPStretch", 0.0),
            ergo.get("PinkyPIPStretch", 0.0),
            ergo.get("PinkyDIPStretch", 0.0),
        ]

        vals = self._retargeters[self._mode].compute(msg, q_deg, compute_side)
        if vals is None:
            return

        # No shared clamp/rate-limiter here: every retargeter is now
        # self-sufficient — ergo and ik clamp to their own joint limits and
        # smooth via EMA; dex's SeqRetargeting clamps to its URDF limits and
        # smooths via its own low_pass_alpha.
        names = self._names[compute_side]
        pub = self._pubs[compute_side]

        self._prev_vals[compute_side] = list(vals)

        out = MultiDOFCommand()
        out.dof_names = names
        out.values = vals
        out.values_dot = [0.0] * len(vals)
        pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ManusTesolloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
