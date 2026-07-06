"""Operator-frame -> canonical MANO-frame transform for hand keypoints.

Ported from dex-retargeting's example ``SingleHandDetector``
(``estimate_frame_from_hand_points``): build a palm-intrinsic wrist frame from
the hand geometry (wrist + index/middle MCP), then rotate into the MANO
convention. Doing this per frame makes downstream retargeting invariant to the
input coordinate system and to the global wrist orientation.

Two operator->MANO matrices, one per input source:

- ``MEDIAPIPE``: camera / MediaPipe world landmarks — dex-retargeting's own
  ``OPERATOR2MANO`` (det +1).
- ``MANUS``: the Manus SDK publishes its raw skeleton in VUH world space, which
  has the OPPOSITE +y chirality, so its matrix is the MediaPipe one with row 1
  (the MANO +y / palm-normal axis) sign-flipped (det -1). Using the MediaPipe
  matrix on Manus input flips the palm-normal — i.e. the thumb-opposition axis —
  so opposition comes out reversed.

The left matrices are derived from the right by bilateral symmetry; the
Manus-left one is not yet hardware-verified on a left DG5F.
"""
import numpy as np

MEDIAPIPE_OPERATOR2MANO = {
    "right": np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float),
    "left": np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=float),
}
MANUS_OPERATOR2MANO = {
    "right": np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]], dtype=float),
    "left": np.array([[0, 0, -1], [-1, 0, 0], [0, -1, 0]], dtype=float),
}
_OP2MANO = {"mediapipe": MEDIAPIPE_OPERATOR2MANO, "manus": MANUS_OPERATOR2MANO}


def estimate_wrist_frame(keypoints):
    """Palm-intrinsic orientation frame from wrist + index/middle MCP.

    ``keypoints`` is a (21, 3) array (wrist-relative) with rows 0/5/9 filled
    (wrist, index MCP, middle MCP). Columns of the result are
    ``[x, palm_normal, z]``; premultiplying keypoints by it (then an
    operator->MANO matrix) normalises out the input frame and the wrist pose.
    """
    points = keypoints[[0, 5, 9], :]  # wrist, index MCP, middle MCP
    x_vector = points[0] - points[2]  # palm -> middle knuckle
    centered = points - np.mean(points, axis=0, keepdims=True)
    _, _, v = np.linalg.svd(centered)
    normal = v[2, :]
    # Gram-Schmidt: make x orthogonal to the palm normal.
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / (np.linalg.norm(x) + 1e-10)
    z = np.cross(x, normal)
    # z should roughly point pinky -> index; flip the frame if it disagrees.
    if np.sum(z * (points[1] - points[2])) < 0:
        normal = -normal
        z = -z
    return np.stack([x, normal, z], axis=1)


def operator2mano(hand_type, convention="manus"):
    """Return the fixed operator->MANO matrix for a side/convention."""
    side = "left" if str(hand_type).lower() == "left" else "right"
    try:
        return _OP2MANO[convention][side]
    except KeyError:
        raise ValueError(
            f"unknown convention {convention!r} (use 'manus' or 'mediapipe')"
        )


def apply_mano_transform(keypoints, hand_type="right", convention="manus"):
    """Rotate wrist-relative (21, 3) keypoints into the canonical MANO frame."""
    return keypoints @ estimate_wrist_frame(keypoints) @ operator2mano(
        hand_type, convention
    )
