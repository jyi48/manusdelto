"""Heuristic retargeting: direct Manus ergonomics → DG5F joint mapping.

No kinematics — angle-to-angle map with per-joint offsets, calibration
scale, and direction signs, plus a backdrive guard that zeroes commands
pushing a joint past its neutral in the wrong direction.
"""
import math

from .base import Retargeter

N = 20

# Per-joint calibration scale (finger order: thumb, index, middle, ring, pinky)
# Matches tesollo_ros2's manus_retarget.py GRIPPER_CALIB reference.
DEFAULT_JOINT_CALIB = [
    1.0, 1.6, 1.3, 1.3,   # thumb
    1.0, 1.0, 1.3, 1.7,   # index
    1.0, 1.0, 1.3, 1.7,   # middle
    1.0, 1.0, 1.3, 1.7,   # ring
    1.0, 1.0, 1.0, 1.0,   # pinky
]

_DIR_L = [
    -1, 1, -1, -1,      # thumb
    1, 1, 1, 1,         # index
    1, 1, 1, 1,         # middle
    1, 1, 1, 1,         # ring
    -1, 1, 1, 1,        # pinky
]
_DIR_R = [
    1, -1, 1, 1,        # thumb
    -1, 1, 1, 1,        # index
    -1, 1, 1, 1,        # middle
    -1, 1, 1, 1,        # ring
    1, -1, 1, 1,        # pinky
]

# Joint limits (rad) for heuristic-mode output only — matches tesollo_ros2's
# manus_retarget.py reference (LEFT_JOINT_LIMITS/RIGHT_JOINT_LIMITS), taken
# directly from the URDF. Unlike the curl-only limits DG5FKinematics applies
# for ik/dex, these allow the full URDF range on PIP/DIP.
_PI_2 = math.pi / 2
HEURISTIC_JOINT_LIMITS = {
    "left": {
        "lj_dg_1_1": (-0.8901179185171081, 0.3839724354387525),
        "lj_dg_1_2": (0.0, math.pi),
        "lj_dg_1_3": (-_PI_2, _PI_2),
        "lj_dg_1_4": (-_PI_2, _PI_2),
        "lj_dg_2_1": (-0.6108652381980153, 0.4188790204786391),
        "lj_dg_2_2": (0.0, 2.007128639793479),
        "lj_dg_2_3": (-_PI_2, _PI_2),
        "lj_dg_2_4": (-_PI_2, _PI_2),
        "lj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
        "lj_dg_3_2": (0.0, 1.9547687622336491),
        "lj_dg_3_3": (-_PI_2, _PI_2),
        "lj_dg_3_4": (-_PI_2, _PI_2),
        "lj_dg_4_1": (-0.4188790204786391, 0.6108652381980153),
        "lj_dg_4_2": (0.0, 1.9024088846738192),
        "lj_dg_4_3": (-_PI_2, _PI_2),
        "lj_dg_4_4": (-_PI_2, _PI_2),
        "lj_dg_5_1": (-1.0471975511965976, 0.017453292519943295),
        "lj_dg_5_2": (-0.6108652381980153, 0.4188790204786391),
        "lj_dg_5_3": (-_PI_2, _PI_2),
        "lj_dg_5_4": (-_PI_2, _PI_2),
    },
    "right": {
        "rj_dg_1_1": (-0.3839724354387525, 0.8901179185171081),
        "rj_dg_1_2": (-math.pi, 0.0),
        "rj_dg_1_3": (-_PI_2, _PI_2),
        "rj_dg_1_4": (-_PI_2, _PI_2),
        "rj_dg_2_1": (-0.4188790204786391, 0.6108652381980153),
        "rj_dg_2_2": (0.0, 2.007128639793479),
        "rj_dg_2_3": (-_PI_2, _PI_2),
        "rj_dg_2_4": (-_PI_2, _PI_2),
        "rj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
        "rj_dg_3_2": (0.0, 1.9547687622336491),
        "rj_dg_3_3": (-_PI_2, _PI_2),
        "rj_dg_3_4": (-_PI_2, _PI_2),
        "rj_dg_4_1": (-0.6108652381980153, 0.4188790204786391),
        "rj_dg_4_2": (0.0, 1.9024088846738192),
        "rj_dg_4_3": (-_PI_2, _PI_2),
        "rj_dg_4_4": (-_PI_2, _PI_2),
        "rj_dg_5_1": (-0.017453292519943295, 1.0471975511965976),
        "rj_dg_5_2": (-0.4188790204786391, 0.6108652381980153),
        "rj_dg_5_3": (-_PI_2, _PI_2),
        "rj_dg_5_4": (-_PI_2, _PI_2),
    },
}


def compute_heuristic(q_deg, side, calib=None):
    PI = math.pi
    if q_deg is None:
        q_deg = [0.0] * N
    q_deg = (list(q_deg) + [0.0] * N)[:N]
    if calib is None or len(calib) < N:
        calib = [1.0] * N

    for i in range(N):
        if q_deg[i] > 180 or q_deg[i] < -180:
            q_deg[i] = 0.0

    dir_arr = _DIR_L if side == "left" else _DIR_R

    # qd offsets match tesollo_ros2's manus_retarget.py reference exactly.
    qd = [0.0] * N
    qd[0] = (58.5 - q_deg[1]) * (PI / 180)
    qd[1] = (q_deg[0] + 20) * (PI / 180)
    qd[2] = q_deg[2] * (PI / 180)
    qd[3] = 0.5 * (q_deg[2] + q_deg[3]) * (PI / 180)
    qd[4] = q_deg[4] * (PI / 180)
    qd[5] = q_deg[5] * (PI / 180)
    qd[6] = (q_deg[6] - 40.0) * (PI / 180)
    qd[7] = q_deg[7] * (PI / 180)
    qd[8] = q_deg[8] * (PI / 180)
    qd[9] = q_deg[9] * (PI / 180)
    qd[10] = (q_deg[10] - 30.0) * (PI / 180)
    qd[11] = q_deg[11] * (PI / 180)
    qd[12] = q_deg[12] * (PI / 180)
    qd[13] = q_deg[13] * (PI / 180)
    qd[14] = q_deg[14] * (PI / 180)
    qd[15] = q_deg[15] * (PI / 180)
    if q_deg[17] > 55 and q_deg[18] > 25 and q_deg[19] > 20:
        qd[16] = abs(q_deg[16]) * 2 * (PI / 180)
    else:
        qd[16] = abs(q_deg[16]) / 1.5 * (PI / 180)
    qd[17] = q_deg[17] * (PI / 180)
    qd[18] = q_deg[18] * (PI / 180)
    qd[19] = q_deg[19] * (PI / 180)

    if side == "left":
        mQd = [0.0] * N
        for i in range(N):
            mQd[i] = qd[i] * calib[i] * dir_arr[i]
            if i in [4, 8, 12, 16]:
                pass
            elif i in [0, 2, 3]:
                if mQd[i] >= 0:
                    mQd[i] = 0.0
            else:
                if mQd[i] <= 0:
                    mQd[i] = 0.0
    else:
        mQd = [0.0] * N
        for i in range(N):
            mQd[i] = qd[i] * calib[i] * dir_arr[i]
            if i in [4, 8, 12, 16]:
                pass
            elif i == 1:
                if mQd[i] >= 0:
                    mQd[i] = 0.0
            else:
                if mQd[i] <= 0:
                    mQd[i] = 0.0

    return mQd


class HeuristicRetargeter(Retargeter):
    name = "heuristic"

    def __init__(self, joint_calib=None):
        self._calib = list(joint_calib) if joint_calib else list(DEFAULT_JOINT_CALIB)

    def compute(self, msg, q_deg, side):
        return compute_heuristic(q_deg, side, self._calib)
