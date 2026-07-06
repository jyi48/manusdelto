"""Common interface for Manus → DG5F retargeting strategies.

Each retargeter maps one ManusGlove message to 20 DG5F joint values
(radians, in LEFT/RIGHT_JOINT_NAMES order). The node owns input parsing and
publishing; each retargeter is responsible for its own joint-limit clamping
and smoothing, and produces the raw joint command for the requested side.
"""


class Retargeter:
    name = "base"

    def compute(self, msg, q_deg, side):
        """Return a list of 20 joint values (rad) for ``side``.

        Parameters
        ----------
        msg : ManusGlove
            Raw glove message (used by pose-based strategies such as IK/dex).
        q_deg : list[float]
            Pre-extracted ergonomics angles (deg), used by ergo.
        side : str
            'left' or 'right' (already resolved for mirror mode).
        """
        raise NotImplementedError
