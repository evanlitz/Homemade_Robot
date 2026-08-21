"""Geometric self-collision. Pure Python. No simulator, no hardware.

WHY THIS IS NOT DONE IN THE PHYSICS. Collision is disabled on every geom in
both models (contype/conaffinity 0). For the palletizer that is forced: F1 shows
a closed loop puts the cut bodies in permanent coincident contact by
construction, and leaving collision on generates spurious forces of hundreds of
newtons that silently corrupt every internal load. Turning it back on to get
self-collision would reintroduce exactly that. So the check lives outside the
physics, as capsule-vs-capsule distance on the same geometry the MJCF is
generated from.

WHAT IT CHECKS, AND WHY ONLY THAT. Two interferences cannot be designed away by
offsetting parts in the third dimension:

  * anything against the COLUMN -- it sits on the rotation axis, so there is
    nowhere to move it to;
  * the GRIPPER against a link it is not attached to -- the gripper has to
    descend to the board, so it cannot be offset out of the plane either.

Link-against-link interference IS excluded, deliberately. Both models place
their links in a single plane (the palletizer's parallelogram rods at y = 0,
SCARA's two links at one carriage height) where a real build staggers them.
Checking those pairs would report collisions that a competent CAD layout
removes, and it would report them for both candidates unequally depending on how
many rods each has. That would be a modelling artefact scoring a design.

Adjacent pairs are excluded for the usual reason: they touch at their shared
joint by construction.
"""

from __future__ import annotations

import numpy as np


def segment_distance(p0: np.ndarray, p1: np.ndarray,
                     q0: np.ndarray, q1: np.ndarray) -> float:
    """Minimum distance between two 3D line SEGMENTS (not lines).

    Clamped closest-point solve. The degenerate cases -- either segment of zero
    length, or the two parallel -- are handled by falling back on the clamped
    projection rather than dividing by a vanishing determinant.
    """
    u, v, w = p1 - p0, q1 - q0, p0 - q0
    a, b, c = float(u @ u), float(u @ v), float(v @ v)
    d, e = float(u @ w), float(v @ w)
    det = a * c - b * b

    if det < 1e-12:  # parallel or a degenerate segment
        s = 0.0
        t = (e / c) if c > 1e-12 else 0.0
    else:
        s = (b * e - c * d) / det
        t = (a * e - b * d) / det

    s = min(max(s, 0.0), 1.0)
    # Re-solve t for the clamped s, then clamp t and re-solve s. One pass of
    # this is exact for segments; skipping it puts the "closest point" off the
    # end of a segment and understates the distance.
    t = (b * s + e) / c if c > 1e-12 else 0.0
    t = min(max(t, 0.0), 1.0)
    s = (b * t - d) / a if a > 1e-12 else 0.0
    s = min(max(s, 0.0), 1.0)

    return float(np.linalg.norm((p0 + s * u) - (q0 + t * v)))


class Capsule:
    """A named line segment with a radius, in world coordinates."""

    __slots__ = ("name", "p", "q", "r")

    def __init__(self, name: str, p, q, r: float):
        self.name = name
        self.p = np.asarray(p, dtype=float)
        self.q = np.asarray(q, dtype=float)
        self.r = float(r)


def clearance(moving: list[Capsule], static: list[Capsule],
              pairs: list[tuple[str, str]]) -> tuple[float, str]:
    """Least surface-to-surface gap over `pairs`, and which pair produced it.

    Negative means interpenetration. `pairs` is given explicitly rather than
    derived from an all-against-all sweep so that the exclusions above are
    stated in one place per architecture and can be argued with.
    """
    by_name = {c.name: c for c in moving + static}
    worst, culprit = float("inf"), "none"
    for a, b in pairs:
        ca, cb = by_name[a], by_name[b]
        gap = segment_distance(ca.p, ca.q, cb.p, cb.q) - ca.r - cb.r
        if gap < worst:
            worst, culprit = gap, f"{a}/{b}"
    return worst, culprit
