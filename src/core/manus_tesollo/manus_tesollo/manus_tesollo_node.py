#!/usr/bin/env python3
import os

import numpy as np
import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand
from manus_ros2_msgs.msg import ManusGlove
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from manus_tesollo.retargeters import build_retargeters, DEFAULT_JOINT_CALIB

CALIB_PHASE_SEC = 4.0  # seconds per phase — matches scm_gui's CALIB_DURATION

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


class ManusTesolloNode(Node):
    def __init__(self):
        super().__init__("manus_tesollo")

        def _p(name, default):
            return (
                self.declare_parameter(name, default).get_parameter_value().string_value
            )

        ns = _p("hand_ns", "dg5f_both")
        left_in = _p("left_input_topic", "/manus_glove_0")
        right_in = _p("right_input_topic", "/manus_glove_1")
        left_out = _p("left_output_topic", f"/{ns}/lj_dg_pospid/reference")
        right_out = _p("right_output_topic", f"/{ns}/rj_dg_pospid/reference")

        self._left_pub = self.create_publisher(MultiDOFCommand, left_out, 10)
        self._right_pub = self.create_publisher(MultiDOFCommand, right_out, 10)
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

        self.create_service(SetBool, "/manus_tesollo/pause", self._cb_pause)
        self.create_service(SetBool, "/manus_tesollo/set_ik_mode", self._cb_set_ik_mode)

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
        self.declare_parameter("dex_scaling_factor", 1.2)
        self.declare_parameter("dex_low_pass_alpha", 0.2)
        self.declare_parameter("ergo_calib", list(DEFAULT_JOINT_CALIB))
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(f"left  {left_in} -> {left_out}")
        self.get_logger().info(f"right {right_in} -> {right_out}")
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
        return SetParametersResult(successful=True)

    def _cb_pause(self, req: SetBool.Request, res: SetBool.Response):
        self._paused = req.data
        res.success = True
        res.message = "paused" if self._paused else "resumed"
        self.get_logger().info(f"manus_tesollo: {res.message}")
        return res

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
            names = LEFT_JOINT_NAMES if compute_side == "left" else RIGHT_JOINT_NAMES
            pub = self._left_pub if compute_side == "left" else self._right_pub
            out = MultiDOFCommand()
            out.dof_names = names
            out.values = self._prev_vals[compute_side]
            out.values_dot = [0.0] * len(names)
            pub.publish(out)
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
        names = LEFT_JOINT_NAMES if compute_side == "left" else RIGHT_JOINT_NAMES
        pub = self._left_pub if compute_side == "left" else self._right_pub

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
