#!/usr/bin/env python3
import os
import logging
import numpy as np

try:
    import pinocchio as pin

    _PINOCCHIO_AVAILABLE = True
except Exception:
    _PINOCCHIO_AVAILABLE = False


class DG5FKinematics:
    """
    Pinocchio-based forward/inverse kinematics for the Tesollo DG5F hand.

    Parameters
    ----------
    hand_side : 'left' | 'right'
    model     : 'm' | 's'  (DG5F-M or DG5F-S)
    logger    : ROS/Python logger, optional
    """

    def __init__(self, hand_side: str = "left", model: str = "m", logger=None):
        self.hand_side = hand_side
        self.model_variant = model
        self._log = logger or logging.getLogger(__name__)
        self.model = None
        self.data = None
        self.finger_tip_frames = []
        self.zero_tip_positions = [np.zeros(3)] * 5
        self.joint_limits = {}
        self.joint_limits_by_name = {}

        if not _PINOCCHIO_AVAILABLE:
            self._log.error("pinocchio not found — IK unavailable")
            return

        try:
            from ament_index_python.packages import get_package_share_directory

            # Search order: model-specific → legacy dg5f_description → bundled in manus_tesollo
            candidates = [
                f"dg5f_{model}_description",
                "dg5f_description",
                "manus_tesollo",
            ]
            urdf_path = None
            for pkg in candidates:
                try:
                    pkg_dir = get_package_share_directory(pkg)
                    p = os.path.join(pkg_dir, "urdf", f"dg5f_{hand_side}.urdf")
                    if os.path.exists(p):
                        urdf_path = p
                        break
                except Exception:
                    continue

            if urdf_path is None:
                raise FileNotFoundError(
                    f"No URDF found for DG5F-{model.upper()} {hand_side} "
                    f"(tried: {candidates})"
                )

            # buildModelFromUrdf loads kinematics only — no mesh files required
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.joint_limits = self._extract_joint_limits()

            prefix = "lj_dg_" if hand_side == "left" else "rj_dg_"
            for i in range(1, 6):
                frame_name = f"{prefix}{i}_tip"
                try:
                    fid = self.model.getFrameId(frame_name)
                    if fid < self.model.nframes:
                        self.finger_tip_frames.append(fid)
                    else:
                        self._log.warning(f"Frame not found: {frame_name}")
                        self.finger_tip_frames.append(None)
                except Exception:
                    self._log.warning(f"Frame lookup failed: {frame_name}")
                    self.finger_tip_frames.append(None)

            # FK at zero angles — gives tip positions in URDF base frame for all joints=0.
            # Used as per-finger base offsets in _extract_target_poses IK target computation.
            q_zero = np.zeros(self.model.nq)
            pin.forwardKinematics(self.model, self.data, q_zero)
            pin.updateFramePlacements(self.model, self.data)
            self.zero_tip_positions = []
            for fid in self.finger_tip_frames:
                if fid is not None:
                    self.zero_tip_positions.append(
                        self.data.oMf[fid].translation.copy()
                    )
                else:
                    self.zero_tip_positions.append(np.zeros(3))

            chains = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
            zero_str = "  ".join(
                f"{c}={p.round(4).tolist()}"
                for c, p in zip(chains, self.zero_tip_positions)
            )
            self._log.info(
                f"DG5F-{model.upper()} {hand_side}: {self.model.nframes} frames, "
                f"{sum(f is not None for f in self.finger_tip_frames)}/5 tip frames  "
                f"zero-FK: {zero_str}"
            )

        except Exception as e:
            self._log.error(f"Failed to load DG5F model: {e}")
            self.model = None

    # Joint limits (rad).  Spread (_1) and MCP (_2) use the URDF range from
    # hw-core/tesollo_ros2.  PIP/DIP (_3, _4) are restricted to a single (curl)
    # direction to stop the IK from hyperextending (backdriving) a finger — the
    # direction follows the heuristic _compute sign convention, which is the
    # ground truth for the URDF joint signs:
    #   - fingers (index..pinky) curl POSITIVE on BOTH hands (not a L/R mirror)
    #   - thumb curls NEGATIVE on the left, POSITIVE on the right (the one mirror)
    _PI_2 = 1.57070963267948966
    _HARDCODED_LIMITS = {
        "left": {
            # Finger 1 (thumb)
            "lj_dg_1_1": (-0.8901179185171081, 0.0),  # ref: thumb spread one-sided (was: ..., 0.3839724354387525)
            "lj_dg_1_2": (0.0, 3.14159265358979),
            "lj_dg_1_3": (-_PI_2, 0.0),  # thumb curls negative (left)
            "lj_dg_1_4": (-_PI_2, 0.0),
            # Finger 2 (index)
            "lj_dg_2_1": (-0.6108652381980153, 0.4188790204786391),
            "lj_dg_2_2": (0.0, 2.007128639793479),
            "lj_dg_2_3": (0.0, _PI_2),  # fingers curl positive
            "lj_dg_2_4": (0.0, _PI_2),
            # Finger 3 (middle)
            "lj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
            "lj_dg_3_2": (0.0, 1.9547687622336491),
            "lj_dg_3_3": (0.0, _PI_2),
            "lj_dg_3_4": (0.0, _PI_2),
            # Finger 4 (ring)
            "lj_dg_4_1": (-0.4188790204786391, 0.6108652381980153),
            "lj_dg_4_2": (0.0, 1.9024088846738192),
            "lj_dg_4_3": (0.0, _PI_2),
            "lj_dg_4_4": (0.0, _PI_2),
            # Finger 5 (pinky)
            "lj_dg_5_1": (-1.0471975511965976, 0.017453292519943295),
            "lj_dg_5_2": (-0.6108652381980153, 0.4188790204786391),
            "lj_dg_5_3": (0.0, _PI_2),
            "lj_dg_5_4": (0.0, _PI_2),
        },
        "right": {
            # Finger 1 (thumb)
            "rj_dg_1_1": (0.0, 0.8901179185171081),  # ref: thumb spread one-sided (was: -0.3839724354387525, ...)
            "rj_dg_1_2": (-3.14159265358979, 0.0),
            "rj_dg_1_3": (0.0, _PI_2),  # thumb curls positive (right)
            "rj_dg_1_4": (0.0, _PI_2),
            # Finger 2 (index)
            "rj_dg_2_1": (-0.4188790204786391, 0.6108652381980153),
            "rj_dg_2_2": (0.0, 2.007128639793479),
            "rj_dg_2_3": (0.0, _PI_2),
            "rj_dg_2_4": (0.0, _PI_2),
            # Finger 3 (middle)
            "rj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
            "rj_dg_3_2": (0.0, 1.9547687622336491),
            "rj_dg_3_3": (0.0, _PI_2),
            "rj_dg_3_4": (0.0, _PI_2),
            # Finger 4 (ring)
            "rj_dg_4_1": (-0.6108652381980153, 0.4188790204786391),
            "rj_dg_4_2": (0.0, 1.9024088846738192),
            "rj_dg_4_3": (0.0, _PI_2),
            "rj_dg_4_4": (0.0, _PI_2),
            # Finger 5 (pinky)
            "rj_dg_5_1": (-0.017453292519943295, 1.0471975511965976),
            "rj_dg_5_2": (-0.4188790204786391, 0.6108652381980153),
            "rj_dg_5_3": (0.0, _PI_2),
            "rj_dg_5_4": (0.0, _PI_2),
        },
    }

    def _extract_joint_limits(self):
        limits = {}
        limits_by_name = {}
        hard = self._HARDCODED_LIMITS.get(self.hand_side, {})
        for jid in range(1, self.model.njoints):
            joint = self.model.joints[jid]
            idx_q = joint.idx_q
            if joint.nq != 1:
                continue
            name = self.model.names[jid]
            if name in hard:
                lo, hi = hard[name]
                lo, hi = min(lo, hi), max(lo, hi)
            else:
                # Fall back to URDF for any joint not in the hand-tuned table.
                lo = float(self.model.lowerPositionLimit[idx_q])
                hi = float(self.model.upperPositionLimit[idx_q])
            limits[idx_q] = (lo, hi)
            limits_by_name[name] = (lo, hi)
        self.joint_limits_by_name = limits_by_name
        return limits

    def forward_kinematics(self, joint_angles):
        if self.model is None:
            return [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)] * 5

        try:
            q = np.array(joint_angles, dtype=float)
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            poses = []
            for fid in self.finger_tip_frames:
                if fid is not None:
                    fp = self.data.oMf[fid]
                    pos = fp.translation
                    quat = pin.Quaternion(fp.rotation)
                    poses.append(
                        (
                            float(pos[0]),
                            float(pos[1]),
                            float(pos[2]),
                            float(quat.x),
                            float(quat.y),
                            float(quat.z),
                            float(quat.w),
                        )
                    )
                else:
                    poses.append((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
            return poses

        except Exception as e:
            self._log.error(f"FK error: {e}")
            return [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)] * 5

    def inverse_kinematics(
        self,
        target_poses,
        initial_guess=None,
        position_weight=1.0,
        orientation_weight=0.5,
        max_iterations=10,
        tolerance=1e-6,
    ):
        """CLIK with damped pseudoinverse Jacobian, solved per finger."""
        if self.model is None:
            return list(initial_guess) if initial_guess else [0.0] * 20

        def damped_pinv(J, lam2):
            U, S, Vh = np.linalg.svd(J, full_matrices=False)
            return Vh.T @ np.diag(S / (S**2 + lam2)) @ U.T

        try:
            q = np.array(initial_guess if initial_guess else [0.0] * 20, dtype=float)
            lam2 = 0.01  # DLS damping
            alpha = 0.5  # Position step size
            alpha_ori_scale = 0.5  # Orientation null-space down-weight (=> alpha*0.5)

            for _ in range(max_iterations):
                total_err = 0.0
                dq = np.zeros_like(q)

                pin.forwardKinematics(self.model, self.data, q)
                pin.updateFramePlacements(self.model, self.data)

                for fi in range(5):
                    fid = self.finger_tip_frames[fi]
                    tgt = target_poses[fi]
                    if fid is None or tgt is None:
                        continue

                    cur = self.data.oMf[fid]
                    err_p = position_weight * (np.array(tgt[:3]) - cur.translation)

                    q_tgt = pin.Quaternion(tgt[6], tgt[3], tgt[4], tgt[5])
                    R_err = q_tgt.toRotationMatrix() @ cur.rotation.T
                    err_r_world = pin.log3(R_err)
                    # Per-axis gain in local frame: de-weights orientation near singularity
                    err_r_local = cur.rotation.T @ err_r_world
                    err_r = cur.rotation @ (
                        err_r_local * np.array([0.5, 0.5, 0.5]) * orientation_weight
                    )

                    total_err += np.linalg.norm(np.concatenate([err_p, err_r]))

                    J = pin.computeFrameJacobian(
                        self.model, self.data, q, fid, pin.LOCAL_WORLD_ALIGNED
                    )
                    js, je = fi * 4, fi * 4 + 4
                    Jp = J[:3, js:je]
                    Jr = J[3:, js:je]

                    Jp_inv = damped_pinv(Jp, lam2)
                    dq_p = alpha * (Jp_inv @ err_p)
                    N = np.eye(4) - Jp_inv @ Jp
                    dq_r = alpha * alpha_ori_scale * N @ (damped_pinv(Jr, lam2) @ err_r)
                    dq[js:je] += dq_p + dq_r

                # Joint limit push-back (soft repulsion) — computed from the
                # current q and added to dq, so the solver "sees" the limit
                # instead of having q hard-clipped underneath it.
                margin, k = 0.05, 2.0
                for i, (lo, hi) in self.joint_limits.items():
                    if q[i] < lo + margin:
                        dq[i] += np.clip(k * ((lo + margin) - q[i]), -0.05, 0.05)
                    elif q[i] > hi - margin:
                        dq[i] -= np.clip(k * (q[i] - (hi - margin)), -0.05, 0.05)

                q = pin.normalize(self.model, q + dq)

                # if total_err < tolerance:
                #     break

            return q.tolist()

        except Exception as e:
            self._log.error(f"IK failed: {e}")
            return list(initial_guess) if initial_guess else [0.0] * 20
