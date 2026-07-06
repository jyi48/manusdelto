"""DexPilot retargeting: Manus hand keypoints → DG5F joints.

Wraps the dex-retargeting library (https://github.com/dexsuite/dex-retargeting).
Imports dex_retargeting at module load, so importing this module raises
ImportError when the library is unavailable; the factory in __init__.py
catches that and falls back to ergo.

Pipeline mirrors dex-retargeting's own reference (SingleHandDetector +
show_realtime_retargeting): wrist-shift the keypoints, derive a palm-intrinsic
wrist frame from the hand geometry, rotate into the canonical MANO frame with
a per-hand operator->MANO matrix (see mano_transform.py — the Manus variant,
not the library's MediaPipe one, since Manus VUH space has opposite +y
chirality), then feed keypoint-difference vectors to the optimizer. The
geometry-derived frame makes retargeting invariant to the input coordinate
system and to the global wrist orientation — which is what lets the thumb oppose
"toward the palm" correctly (a fixed rotation could not).

DexPilot's optimizer only consumes the wrist (MANO 0) and the five fingertips
(MANO 4/8/12/16/20); the palm frame additionally needs the index/middle MCP
(MANO 5/9). We fill those from Manus raw_nodes (Hand root + per-finger TIP/PIP).
"""
import os
import time

import numpy as np

from dex_retargeting.retargeting_config import RetargetingConfig

from .base import Retargeter
from .mano_transform import MANUS_OPERATOR2MANO, estimate_wrist_frame

# Manus (chain_type, joint_type) -> MANO 21-keypoint index. Manus publisher
# labels are off-by-one anatomically: "PIP" is the MCP knuckle, "IP" the PIP,
# "DIP" the DIP. Wrist (chain "Hand") is index 0. DexPilot uses only wrist +
# tips (0,4,8,12,16,20); the palm frame adds index/middle MCP (5,9); the vector
# optimizer additionally uses the IP/DIP joints — so we fill the whole skeleton.
_MANO_REMAP = {
    ("Thumb", "MCP"): 1, ("Thumb", "PIP"): 2, ("Thumb", "DIP"): 3, ("Thumb", "TIP"): 4,
    ("Index", "PIP"): 5, ("Index", "IP"): 6, ("Index", "DIP"): 7, ("Index", "TIP"): 8,
    ("Middle", "PIP"): 9, ("Middle", "IP"): 10, ("Middle", "DIP"): 11, ("Middle", "TIP"): 12,
    ("Ring", "PIP"): 13, ("Ring", "IP"): 14, ("Ring", "DIP"): 15, ("Ring", "TIP"): 16,
    ("Pinky", "PIP"): 17, ("Pinky", "IP"): 18, ("Pinky", "DIP"): 19, ("Pinky", "TIP"): 20,
}
_TIP_IDX = (4, 8, 12, 16, 20)

# DG5F command order per side (must match LEFT/RIGHT_JOINT_NAMES in the node).
_CMD_JOINTS = {
    side: [f"{p}j_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]
    for side, p in (("left", "l"), ("right", "r"))
}


class DexRetargeter(Retargeter):
    name = "dex"

    def __init__(self, urdf_dir, logger, config_dir=None, optimizer="dexpilot"):
        self._log = logger
        self._last_solve_ms = 0.0
        if config_dir is None:
            config_dir = os.path.join(os.path.dirname(__file__), "configs")
        RetargetingConfig.set_default_urdf_dir(urdf_dir)

        # 'dexpilot' (tip + pinch prior) or 'vector' (adds MCP->DIP direction
        # vectors so fingers/thumb fold naturally, not just reach the tip).
        if optimizer not in ("dexpilot", "vector"):
            logger.warn(f"dex: unknown optimizer '{optimizer}', using dexpilot")
            optimizer = "dexpilot"
        self._optimizer = optimizer

        self._op2mano = MANUS_OPERATOR2MANO  # per-hand Manus VUH -> MANO
        self._last_frame = {"left": None, "right": None}
        self._retgt = {}
        self._reorder = {}
        for side in ("left", "right"):
            cfg = os.path.join(config_dir, f"dg5f_{side}_{optimizer}.yml")
            rt = RetargetingConfig.load_from_file(cfg).build()
            self._retgt[side] = rt
            # Map dex output order (optimizer.target_joint_names) → DG5F cmd order.
            dex_names = list(rt.optimizer.target_joint_names)
            self._reorder[side] = [dex_names.index(n) for n in _CMD_JOINTS[side]]
            self._log.info(
                f"dex[{optimizer}] {side}: {len(dex_names)} target joints"
            )

    def compute(self, msg, q_deg, side):
        rt = self._retgt.get(side)
        if rt is None:
            return None
        mano = self._build_mano(msg)
        if mano is None:
            return None

        # Palm frame needs the MCP knuckles; check before the wrist shift (a
        # missing node reads as origin). Reuse the last good frame on a brief
        # MCP dropout so the hand doesn't freeze.
        have_mcp = np.any(mano[5]) and np.any(mano[9])
        mano = mano - mano[0]  # wrist origin shift
        if have_mcp:
            wrist_rot = estimate_wrist_frame(mano)
            self._last_frame[side] = wrist_rot
        else:
            wrist_rot = self._last_frame[side]
            if wrist_rot is None:
                self._log.warn(
                    "dex: no MCP nodes yet, can't build palm frame",
                    throttle_duration_sec=5.0,
                )
                return None

        joint_pos = mano @ wrist_rot @ self._op2mano[side]

        # Mirror mode (glove side != robot side): op2mano[side] is keyed on the
        # ROBOT side while the keypoints come from the opposite glove, so the
        # chirality won't match. If mirror feels wrong on hardware, reflect
        # joint_pos across one axis before building ref_value.
        idx = rt.optimizer.target_link_human_indices  # (2, n_vec): [origin; task]
        ref_value = joint_pos[idx[1], :] - joint_pos[idx[0], :]
        t0 = time.perf_counter()
        qpos = rt.retarget(ref_value)  # order: optimizer.target_joint_names
        self._last_solve_ms = (time.perf_counter() - t0) * 1e3
        self._log.info(
            f"dex {side}: solve {self._last_solve_ms:.1f} ms",
            throttle_duration_sec=2.0,
        )
        order = self._reorder[side]
        return [float(qpos[i]) for i in order]

    def _build_mano(self, msg):
        """Fill the MANO 21-keypoint skeleton from Manus raw_nodes (wrist +
        every remappable joint). Returns None if any fingertip is missing."""
        if not msg.raw_nodes:
            return None

        mano = np.zeros((21, 3), dtype=float)
        filled = set()
        for n in msg.raw_nodes:
            if n.chain_type == "Hand":
                mano[0] = self._pos(n)
                filled.add(0)
                continue
            i = _MANO_REMAP.get((n.chain_type, n.joint_type))
            if i is not None:
                mano[i] = self._pos(n)
                filled.add(i)

        missing_tips = [t for t in _TIP_IDX if t not in filled]
        if missing_tips:
            self._log.warn(
                f"dex: missing fingertip nodes (MANO {missing_tips})",
                throttle_duration_sec=5.0,
            )
            return None
        return mano

    @staticmethod
    def _pos(node):
        p = node.pose.position
        return np.array([p.x, p.y, p.z], dtype=float)
