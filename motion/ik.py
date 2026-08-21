"""Damped least-squares inverse kinematics for the AR4.

Iterative, seeded from a starting guess. Converges on poses inside the
workspace; returns None rather than a bad answer when it doesn't.
"""
import numpy as np

from motion.kinematics import fk, jacobian, JOINT_LIMITS, in_limits

def pose_error(current, target):
    """6-vector twist taking `current` to `target`. Position in m, rotation in rad."""
    e = np.zeros(6)
    e[:3] = target[:3, 3] - current[:3, 3]
    r = target[:3, :3] @ current[:3, :3].T
    angle = np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    if angle > 1e-9:
        axis = np.array([r[2, 1] - r[1, 2],
                         r[0, 2] - r[2, 0],
                         r[1, 0] - r[0, 1]]) / (2.0 * np.sin(angle))
        e[3:] = axis * angle
    return e


def ik(target, q0_deg, tol_mm=0.01, tol_deg=0.01,
       max_iters=200, damping=0.02, step_limit_deg=10.0, clamp=True):
    """Solve for joint angles reaching `target` (4x4, metres).

    Returns degrees, or None if it fails to converge or leaves the limits.
    """
    q = np.asarray(q0_deg, dtype=float).copy()
    tol_m = tol_mm / 1000.0
    tol_rad = np.deg2rad(tol_deg)

    for _ in range(max_iters):
        current = fk(q)
        err = pose_error(current, target)

        if np.linalg.norm(err[:3]) < tol_m and np.linalg.norm(err[3:]) < tol_rad:
            return q if (not clamp or in_limits(q)) else None

        j = jacobian(q)
        jt = j.T
        dq = jt @ np.linalg.solve(j @ jt + (damping ** 2) * np.eye(6), err)
        dq = np.rad2deg(dq)

        big = np.max(np.abs(dq))
        if big > step_limit_deg:
            dq *= step_limit_deg / big

        q += dq

        if clamp:
            for i, (lo, hi) in enumerate(JOINT_LIMITS):
                q[i] = min(max(q[i], lo), hi)

    return None


def _default_seeds():
    """Spread over J1 and J4, both elbow branches.

    The previous four seeds all had J1 in {0, 90} and J4 = 0, which left poses
    with large negative J1 or large |J4| unreachable from any of them: they
    missed 2% of regular poses, and some of those misses were at perfectly
    well-conditioned configurations. Cost is ~3.7 solver calls per cold solve
    against 1.2 before, paid only on cold starts -- a warm seed still converges
    in one.
    """
    return [np.array([j1, elbow * 20.0, -elbow * 20.0, j4, elbow * 40.0, 0.0])
            for j1 in (-120.0, -60.0, 0.0, 60.0, 120.0)
            for elbow in (1.0, -1.0)
            for j4 in (0.0, 120.0, -120.0)]


def ik_multistart(target, seeds=None, **kw):
    """Try several seeds. Returns the first solution found."""
    if seeds is None:
        seeds = _default_seeds()
    for s in seeds:
        r = ik(target, s, **kw)
        if r is not None:
            return r
    return None