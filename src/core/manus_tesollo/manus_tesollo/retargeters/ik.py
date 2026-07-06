"""IK retargeting: Manus fingertip poses → DG5F joints via per-finger CLIK.

Imports pinocchio at module load, so importing this module raises
ImportError when pinocchio is unavailable; the factory in __init__.py
catches that and falls back to ergo.
"""
import numpy as np
import pinocchio as pin

from .base import Retargeter
from .smoothing import EMAFilter

_DG5F_FINGER_CHAINS = ["Thumb", "Index", "Middle", "Ring", "Pinky"]


class IKRetargeter(Retargeter):
    name = "ik"

    # Per-finger orientation corrections: aligns Manus tip frame → DG5F tip frame.
    _CORR_THUMB_L = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)
    _CORR_THUMB_R = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]], dtype=float)
    _CORR_FINGER = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    # Manus wrist body frame → DG5F URDF base frame.
    _R_BODY_TO_DG5F = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)

    def __init__(
        self,
        kin,
        seed,
        *,
        logger,
        pos_w=1.0,
        ori_w=1.0,
        max_itr=3,
        ik_tol=1e-3,
        marker_scale=1.0,
        base_offset=None,
        center_offset=None,
        ema_alpha=0.4,
    ):
        self._kin = kin
        self._seed = seed  # ergo retargeter: CLIK initial guess + fallback when IK can't solve
        self._log = logger
        self._pos_w = pos_w
        self._ori_w = ori_w
        self._max_itr = max_itr
        self._ik_tol = ik_tol
        self._marker_scale = marker_scale
        self._base_offset = (
            np.zeros(3) if base_offset is None else np.asarray(base_offset, dtype=float)
        )
        self._center_offset = (
            np.zeros(3) if center_offset is None else np.asarray(center_offset, dtype=float)
        )
        self._ema = {"left": EMAFilter(ema_alpha), "right": EMAFilter(ema_alpha)}

    def compute(self, msg, q_deg, side):
        seed = self._seed.compute(msg, q_deg, side)
        result = seed
        # Mirror mode (glove side != robot side): the extracted tip positions
        # come from the opposite-hand glove, which is the chiral mirror of the
        # target hand. If mirror feels wrong on hardware, reflect the tip
        # positions across one axis in _extract_target_poses before solving
        # (the node would need to pass a `mirrored` flag through compute()).
        if side in self._kin:
            try:
                target_poses = self._extract_target_poses(msg, side)
                if target_poses is None:
                    self._log.warn("IK: no target poses", throttle_duration_sec=5.0)
                else:
                    vals_ik = self._solve_finger_ik(target_poses, seed, side)
                    if vals_ik is not None:
                        result = vals_ik
            except Exception as e:
                self._log.warn(f"IK error: {e}", throttle_duration_sec=1.0)
        return self._ema[side].filter(result)

    def _extract_target_poses(self, msg, robot_side):
        if not msg.raw_nodes:
            return None

        tip_by_chain = {n.chain_type: n for n in msg.raw_nodes if n.joint_type == "TIP"}
        missing = [c for c in _DG5F_FINGER_CHAINS if c not in tip_by_chain]
        if missing:
            self._log.warn(f"Missing TIP nodes: {missing}", throttle_duration_sec=5.0)
            return None

        target_poses = []
        for chain in _DG5F_FINGER_CHAINS:
            tp = tip_by_chain[chain].pose

            raw = np.array([tp.position.x, tp.position.y, tp.position.z])
            rot = self._R_BODY_TO_DG5F @ raw
            rel_pos = (
                (rot - self._center_offset) * self._marker_scale
                + self._base_offset
                + self._center_offset
            )

            tip_rot = pin.Quaternion(
                tp.orientation.w,
                tp.orientation.x,
                tp.orientation.y,
                tp.orientation.z,
            ).toRotationMatrix()
            R_corr = (
                (self._CORR_THUMB_L if robot_side == "left" else self._CORR_THUMB_R)
                if chain == "Thumb"
                else self._CORR_FINGER
            )
            rel_quat = pin.Quaternion(self._R_BODY_TO_DG5F @ tip_rot @ R_corr)

            target_poses.append(
                (
                    float(rel_pos[0]),
                    float(rel_pos[1]),
                    float(rel_pos[2]),
                    float(rel_quat.x),
                    float(rel_quat.y),
                    float(rel_quat.z),
                    float(rel_quat.w),
                )
            )
        return target_poses

    def _solve_finger_ik(self, target_poses, prev_q, side):
        try:
            return self._kin[side].inverse_kinematics(
                target_poses=target_poses,
                initial_guess=prev_q,
                position_weight=self._pos_w,
                orientation_weight=self._ori_w,
                max_iterations=self._max_itr,
                tolerance=self._ik_tol,
            )
        except Exception as e:
            self._log.error(f"IK failed: {e}")
            return None
