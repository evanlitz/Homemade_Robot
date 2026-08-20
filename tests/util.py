"""Shared test helpers. Not imported by anything in the package."""
import numpy as np

from motion.kinematics import JOINT_LIMITS


def rotvec(v):
    """Rotation vector -> 3x3. The oracle's forward_kinematics returns its
    orientation as a rotation vector in x, y, z order, despite the field names
    being rz, ry, rx. Confirmed by fitting 84 candidate conventions against
    the analytic FK; this one matched to 0.004 deg and nothing else came
    within 127 deg."""
    theta = np.linalg.norm(v)
    if theta < 1e-12:
        return np.eye(3)
    k = np.asarray(v, dtype=float) / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1.0 - np.cos(theta)) * (kx @ kx)


def oracle_pose(k, q_deg):
    """Oracle FK as a 4x4 in metres."""
    o = np.array(k.forward_kinematics(list(q_deg)))
    t = np.eye(4)
    t[:3, :3] = rotvec(o[3:])
    t[:3, 3] = o[:3] / 1000.0
    return t


def pose_gap(a, b):
    """(mm, deg) between two 4x4 poses."""
    mm = np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000.0
    r = a[:3, :3] @ b[:3, :3].T
    deg = np.rad2deg(np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)))
    return mm, deg


def sample_q(rng, inset=0.0):
    """Random config, optionally inset from the joint limits."""
    q = []
    for lo, hi in JOINT_LIMITS:
        pad = (hi - lo) * inset
        q.append(rng.uniform(lo + pad, hi - pad))
    return np.array(q)


# J5 = 0 puts the J4 and J6 axes collinear -- the spherical wrist singularity.
# The Jacobian's smallest singular value falls off linearly in J5 either side
# of it, so anything inside a few degrees is numerically singular too.
WRIST_SINGULAR_BAND_DEG = 8.0


def sample_q_regular(rng, inset=0.0, band=WRIST_SINGULAR_BAND_DEG):
    """Random config with J5 kept clear of the wrist singularity."""
    while True:
        q = sample_q(rng, inset)
        if abs(q[4]) >= band:
            return q
