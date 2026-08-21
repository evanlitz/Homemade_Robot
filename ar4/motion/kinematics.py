"""Forward kinematics for the AR4, modified (Craig) DH convention.

Verified against Annin's compiled robot_kinematics module; see tests/ for the
pose count and tolerance actually asserted.

Transform per link:  H = Rx(alpha) . Tx(a) . Rz(theta) . Tz(d)

alpha6 is -90, NOT the 180 that robot_set() in the HMI's kinematics.cpp
installs. With 180 the J6 axis comes out exactly antiparallel to J5 at every
configuration, so the Jacobian is rank 5 everywhere and the arm cannot control
orientation. defaults.json, robot_data_reset(), and the URDF joint axes all
agree on -90; robot_set() is a stale fallback that aligns to the 3D model's
link-6 mesh frame. See tests/test_wrist_rank.py.
"""
import numpy as np

# alpha (rad), a (mm), theta_offset (rad), d (mm)
DHM = (
    (0.0,        0.0,   0.0,       169.77),
    (-np.pi / 2, 64.2,  -np.pi / 2,  0.0),
    (0.0,        305.0, 0.0,         0.0),
    (-np.pi / 2, 0.0,   0.0,       222.63),
    (np.pi / 2,  0.0,   0.0,         0.0),
    (-np.pi / 2, 0.0,   np.pi,      41.0),
)

# degrees, (lower, upper)
JOINT_LIMITS = (
    (-170.0, 170.0),
    (-42.0,  90.0),
    (-89.0,  52.0),
    (-180.0, 180.0),
    (-105.0, 105.0),
    (-180.0, 180.0),
)

# per-joint sign; all +1 on a stock AR4
SENSES = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def _tx(d):
    m = np.eye(4)
    m[0, 3] = d
    return m


def _tz(d):
    m = np.eye(4)
    m[2, 3] = d
    return m


def link_transform(i, q_deg):
    """Homogeneous transform for link i (0-indexed) at joint angle q_deg."""
    alpha, a, theta0, d = DHM[i]
    theta = theta0 + np.deg2rad(q_deg * SENSES[i])
    return _rx(alpha) @ _tx(a / 1000.0) @ _rz(theta) @ _tz(d / 1000.0)


def fk(q_deg, upto=6):
    """Forward kinematics. Angles in degrees, returns 4x4 pose in metres."""
    m = np.eye(4)
    for i in range(upto):
        m = m @ link_transform(i, q_deg[i])
    return m


def fk_all(q_deg):
    """Cumulative transform after each link. Useful for Jacobians and drawing."""
    out, m = [], np.eye(4)
    for i in range(6):
        m = m @ link_transform(i, q_deg[i])
        out.append(m.copy())
    return out


def jacobian(q_deg):
    """Geometric Jacobian in the base frame. 6x6: linear rows then angular.

    Under modified DH the joint rotation Rz(theta) is applied *after*
    Rx(alpha).Tx(a), so joint i's axis lives in that intermediate frame --
    not in the previous link frame.
    """
    j = np.zeros((6, 6))
    p_end = fk(q_deg)[:3, 3]
    base = np.eye(4)
    for i in range(6):
        alpha, a, _, _ = DHM[i]
        axis_frame = base @ _rx(alpha) @ _tx(a / 1000.0)
        z = axis_frame[:3, :3] @ np.array([0.0, 0.0, 1.0])
        origin = axis_frame[:3, 3]
        j[:3, i] = np.cross(z, p_end - origin)
        j[3:, i] = z
        base = base @ link_transform(i, q_deg[i])
    return j


def in_limits(q_deg):
    return all(lo <= q <= hi for q, (lo, hi) in zip(q_deg, JOINT_LIMITS))