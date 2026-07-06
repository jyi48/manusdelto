"""Ergo retargeting: direct Manus ergonomics -> DG5F joint-angle mapping, with
optional 2-phase (rest/fist) ROM calibration and EMA smoothing.

Self-contained: owns its own direction signs, calibration factors, and
joint-limit table. The limits are curl-only (flex joints pinned to a 0 lower
bound) — matching dg5f_kinematics._HARDCODED_LIMITS (what ik clamps to), NOT
the raw URDF (which leaves PIP/DIP open to +/-pi/2). heuristic.py is kept in
the repo only as a historical record; nothing imports it.

Backdrive protection has two layers:
- _apply_posture_constraints guards thumb (0,2,3) and the four spread joints
  (4,8,12,16) — axes whose "wrong direction" isn't simply the zero boundary.
- the curl-only JOINT_LIMITS clamp keeps every flex joint (MCP/PIP/DIP) >= 0,
  which is what actually stops fingers bending backward when the qd
  transform's -40deg/-30deg PIP offsets drive an extended finger negative.
"""
import math

from .base import Retargeter
from .smoothing import EMAFilter

N = 20
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
_PI_2 = math.pi / 2
_MIN_ROM_DEG = 5.0  # ignore ROM < 5 deg (sensor noise) during calibration

# Direction signs (Manus -> DG5F axis correction). Hardware-verified.
_RIGHT_DIRECTIONS = [
    1, -1, 1, 1,
    -1, 1, 1, 1,
    -1, 1, 1, 1,
    -1, 1, 1, 1,
    1, -1, 1, 1,
]
_LEFT_DIRECTIONS = [
    -1, 1, -1, -1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    -1, 1, 1, 1,
]

# Per-joint calibration factors. Tesollo reference default.
DEFAULT_JOINT_CALIB = [
    1.0, 1.6, 1.3, 1.3,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.0, 1.0,
]

# Joint limits (rad). Values come from the DG5F URDF, but the flex joints
# (MCP/PIP/DIP) have their lower bound pinned to 0 so fingers can't bend
# backward — the raw URDF opens PIP/DIP to ±pi/2, yet the qd transform's
# -40deg/-30deg PIP offsets drive those joints negative when the hand is
# extended, so an open lower bound shows up as backward-bent fingers. This
# matches dg5f_kinematics._HARDCODED_LIMITS (what ik clamps to). Spread joints
# (_1, except the one-sided thumb) stay two-sided.
JOINT_LIMITS = {
    "left": {
        "lj_dg_1_1": (-0.8901179185171081, 0.0),
        "lj_dg_1_2": (0.0, math.pi),
        "lj_dg_1_3": (-_PI_2, 0.0),  # thumb curls negative (left)
        "lj_dg_1_4": (-_PI_2, 0.0),
        "lj_dg_2_1": (-0.6108652381980153, 0.4188790204786391),
        "lj_dg_2_2": (0.0, 2.007128639793479),
        "lj_dg_2_3": (0.0, _PI_2),  # fingers curl positive
        "lj_dg_2_4": (0.0, _PI_2),
        "lj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
        "lj_dg_3_2": (0.0, 1.9547687622336491),
        "lj_dg_3_3": (0.0, _PI_2),
        "lj_dg_3_4": (0.0, _PI_2),
        "lj_dg_4_1": (-0.4188790204786391, 0.6108652381980153),
        "lj_dg_4_2": (0.0, 1.9024088846738192),
        "lj_dg_4_3": (0.0, _PI_2),
        "lj_dg_4_4": (0.0, _PI_2),
        "lj_dg_5_1": (-1.0471975511965976, 0.017453292519943295),
        "lj_dg_5_2": (-0.6108652381980153, 0.4188790204786391),
        "lj_dg_5_3": (0.0, _PI_2),
        "lj_dg_5_4": (0.0, _PI_2),
    },
    "right": {
        "rj_dg_1_1": (0.0, 0.8901179185171081),
        "rj_dg_1_2": (-math.pi, 0.0),
        "rj_dg_1_3": (0.0, _PI_2),  # thumb curls positive (right)
        "rj_dg_1_4": (0.0, _PI_2),
        "rj_dg_2_1": (-0.4188790204786391, 0.6108652381980153),
        "rj_dg_2_2": (0.0, 2.007128639793479),
        "rj_dg_2_3": (0.0, _PI_2),
        "rj_dg_2_4": (0.0, _PI_2),
        "rj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
        "rj_dg_3_2": (0.0, 1.9547687622336491),
        "rj_dg_3_3": (0.0, _PI_2),
        "rj_dg_3_4": (0.0, _PI_2),
        "rj_dg_4_1": (-0.6108652381980153, 0.4188790204786391),
        "rj_dg_4_2": (0.0, 1.9024088846738192),
        "rj_dg_4_3": (0.0, _PI_2),
        "rj_dg_4_4": (0.0, _PI_2),
        "rj_dg_5_1": (-0.017453292519943295, 1.0471975511965976),
        "rj_dg_5_2": (-0.4188790204786391, 0.6108652381980153),
        "rj_dg_5_3": (0.0, _PI_2),
        "rj_dg_5_4": (0.0, _PI_2),
    },
}
_JOINT_NAMES = {
    "left": [f"lj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)],
    "right": [f"rj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)],
}


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _apply_posture_constraints(qd, side):
    """Zero out anatomically-impossible thumb/spread values (in-place)."""
    if side == "right":
        if qd[0] < 0: qd[0] = 0.0
        if qd[2] < 0: qd[2] = 0.0
        if qd[3] < 0: qd[3] = 0.0
        for i in (4, 8, 12):
            if qd[i] > 0: qd[i] = 0.0
        if qd[16] > 0: qd[16] = 0.0
    else:
        if qd[0] > 0: qd[0] = 0.0
        if qd[2] > 0: qd[2] = 0.0
        if qd[3] > 0: qd[3] = 0.0
        for i in (4, 8, 12):
            if qd[i] < 0: qd[i] = 0.0
        if qd[16] < 0: qd[16] = 0.0


def _compute_qd(q_deg, side, calib):
    """Manus ergonomics (deg) -> DG5F joint angles (rad), before joint-limit
    clamp/EMA. No input guard here so ROM-scaled values (which can legitimately
    exceed 180) pass straight through; the caller sanitises the raw input."""
    if q_deg is None:
        q_deg = [0.0] * N
    q_deg = (list(q_deg) + [0.0] * N)[:N]

    qd = [0.0] * N
    # Thumb: spread/flex swap + offset.
    qd[0] = (58.5 - q_deg[1]) * DEG2RAD
    qd[1] = (q_deg[0] + 20.0) * DEG2RAD
    qd[2] = q_deg[2] * DEG2RAD
    qd[3] = 0.5 * (q_deg[2] + q_deg[3]) * DEG2RAD

    # Index
    qd[4] = q_deg[4] * DEG2RAD
    qd[5] = q_deg[5] * DEG2RAD
    qd[6] = (q_deg[6] - 40.0) * DEG2RAD
    qd[7] = 0.5 * (q_deg[6] + q_deg[7]) * DEG2RAD

    # Middle
    qd[8] = q_deg[8] * DEG2RAD
    qd[9] = q_deg[9] * DEG2RAD
    qd[10] = (q_deg[10] - 30.0) * DEG2RAD
    qd[11] = 0.5 * (q_deg[10] + q_deg[11]) * DEG2RAD

    # Ring
    qd[12] = q_deg[12] * DEG2RAD
    qd[13] = q_deg[13] * DEG2RAD
    qd[14] = q_deg[14] * DEG2RAD
    qd[15] = q_deg[15] * DEG2RAD

    # Pinky (conditional spread boost when the hand is substantially curled)
    if q_deg[17] > 55.0 and q_deg[18] > 25.0 and q_deg[19] > 20.0:
        spread_mult = 2.0
    else:
        spread_mult = 1.0 / 1.5
    qd[16] = q_deg[16] * spread_mult * DEG2RAD
    qd[17] = q_deg[17] * DEG2RAD
    qd[18] = q_deg[18] * DEG2RAD
    qd[19] = q_deg[19] * DEG2RAD

    dirs = _LEFT_DIRECTIONS if side == "left" else _RIGHT_DIRECTIONS
    qd = [qd[i] * calib[i] * dirs[i] for i in range(N)]

    _apply_posture_constraints(qd, side)
    return qd


class _SideCalib:
    """Per-side 2-phase (rest/fist) ROM calibration state, at the ergonomics
    (input, degrees) level.

    Once both poses are captured, each joint is normalised so the rest pose
    reads 0 and the fist pose spans the full DG5F joint range:
        out = q_deg * scale + offset
        scale  = dg5f_range / |fist - rest|      (rest -> 0, fist -> dg5f_range)
        offset = -rest * scale
    Joints whose ROM is below _MIN_ROM_DEG are left untouched (scale 1, no
    offset) so sensor noise on near-static joints isn't amplified. dg5f_range
    is the curl-only JOINT_LIMITS span, in degrees to match the ergonomics
    input; the map pairs ergonomics index i with DG5F joint i (an approximation,
    since the _compute_qd transform mixes some indices, e.g. the thumb)."""

    def __init__(self, dg5f_range_deg):
        self._range = dg5f_range_deg
        self.rest = None
        self.fist = None
        self.scale = [1.0] * N
        self.offset = [0.0] * N
        self.calibrated = False

    def apply(self, q_deg):
        if not self.calibrated:
            return q_deg
        return [q_deg[i] * self.scale[i] + self.offset[i] for i in range(N)]

    def finish(self):
        if self.rest is None or self.fist is None:
            return False
        for i in range(N):
            rom = self.fist[i] - self.rest[i]
            if abs(rom) > _MIN_ROM_DEG:
                self.scale[i] = self._range[i] / abs(rom)
                self.offset[i] = -self.rest[i] * self.scale[i]
            else:
                self.scale[i] = 1.0
                self.offset[i] = 0.0
        self.calibrated = True
        return True


class ErgoRetargeter(Retargeter):
    name = "ergo"

    def __init__(self, joint_calib=None, ema_alpha=0.4):
        self._calib = list(joint_calib) if joint_calib else list(DEFAULT_JOINT_CALIB)
        self._ema = {"left": EMAFilter(ema_alpha), "right": EMAFilter(ema_alpha)}
        self._limits_arr = {
            side: [JOINT_LIMITS[side][n] for n in _JOINT_NAMES[side]]
            for side in ("left", "right")
        }
        # DG5F per-joint range (deg) — the span the fist pose is normalised onto.
        self._range_deg = {
            side: [(hi - lo) * RAD2DEG for (lo, hi) in self._limits_arr[side]]
            for side in ("left", "right")
        }
        self._rom = {
            side: _SideCalib(self._range_deg[side]) for side in ("left", "right")
        }
        self._calib_phase = {"left": 0, "right": 0}  # 0=idle, 1=capturing rest, 2=capturing fist
        self._calib_samples = {"left": [], "right": []}

    def set_calib(self, values):
        """Live-replace the per-joint calibration factors (shared by both
        sides). self._calib is read fresh each compute() call, so this
        applies on the very next frame -- no rebuild needed."""
        values = list(values)
        if len(values) != N:
            raise ValueError(f"expected {N} calib values, got {len(values)}")
        self._calib = values

    def compute(self, msg, q_deg, side):
        qd = self.compute_unfiltered(msg, q_deg, side)
        return self._ema[side].filter(qd)

    def compute_unfiltered(self, msg, q_deg, side):
        """Same as compute(), but without this retargeter's own EMA step.

        ik uses this for its CLIK seed/fallback so the seed isn't smoothed
        twice (once here, once by ik's own EMA on its final output)."""
        if q_deg is None:
            q_deg = [0.0] * N
        q_deg = (list(q_deg) + [0.0] * N)[:N]
        # Sanitise wild sensor readings on the RAW ergonomics (before capture and
        # ROM scale) so a glitch can't poison the calibration average. Not in the
        # reference; kept only on raw input so it never touches ROM-scaled values.
        for i in range(N):
            if q_deg[i] > 180 or q_deg[i] < -180:
                q_deg[i] = 0.0

        if self._calib_phase[side] in (1, 2):
            self._calib_samples[side].append(list(q_deg))

        q_deg = self._rom[side].apply(q_deg)
        qd = _compute_qd(q_deg, side, self._calib)

        lo_hi = self._limits_arr[side]
        return [_clamp(qd[i], lo_hi[i][0], lo_hi[i][1]) for i in range(N)]

    # ── Calibration control (driven by the node's calibrate service) ──────────

    def start_rest_capture(self, side):
        self._calib_phase[side] = 1
        self._calib_samples[side] = []

    def start_fist_capture(self, side):
        self._calib_phase[side] = 2
        self._calib_samples[side] = []

    def finish_capture(self, side):
        """Call at the end of each phase. Averages samples into rest/fist
        ergonomics; on the second call (fist), also computes the rest-shift
        offset. Returns False if no samples were collected."""
        samples = self._calib_samples[side]
        phase = self._calib_phase[side]
        self._calib_phase[side] = 0
        if not samples:
            return False
        avg = [sum(s[i] for s in samples) / len(samples) for i in range(N)]
        if phase == 1:
            self._rom[side].rest = avg
        elif phase == 2:
            self._rom[side].fist = avg
            self._rom[side].finish()
        return True
