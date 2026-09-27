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

# degrees, (lower, upper). The MK5's: identical to the MK3 except J1, which
# the MK5's hall-effect homing narrows to +/-160 (Annin-Robotics/ar4_ros_driver,
# annin_ar4_description/config/mk5.yaml and the MK5 limits in AR4_teensy.ino).
# The link geometry is the same on both, so nothing else here changes.
JOINT_LIMITS = (
    (-160.0, 160.0),
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


# Fingertip offset of the SG1 gripper along the flange approach axis (+z6).
# MEASURED off models/ar4.xml: every vertex of both jaw meshes projected onto
# the ee site's z axis with the jaws closed; the furthest lies at 64.18 mm and
# the two jaws agree to 0.01 mm. The gripper mounts with a pure rotation about
# x, so the offset is purely axial and the tool frame keeps the flange's
# orientation -- z6 already IS the approach axis.
TOOL_MM = 64.18

TOOL = np.eye(4)
TOOL[2, 3] = TOOL_MM / 1000.0

TOOL_INV = np.eye(4)
TOOL_INV[2, 3] = -TOOL_MM / 1000.0


def axial_tool(length_mm):
    """Tool frame `length_mm` out along the flange's approach axis (+z6).

    A pen held in the SG1's jaws is this, with length = TOOL_MM plus however
    far the tip sticks out past the fingertips. The length does not need to be
    exact for drawing: touching the page off with the same tool puts any
    axial error into the page height, where it cancels (see motion.draw.Page).
    """
    t = np.eye(4)
    t[2, 3] = length_mm / 1000.0
    return t


def tool_matrix(tool):
    """None for the flange, else the 4x4 flange->tool transform.

    `tool` is False/None (flange), True (the SG1 fingertip, TOOL), or any 4x4
    such as axial_tool(...). Accepting a matrix lets every caller that already
    threads `tool=` through -- ik, plan_line -- aim a pen without changes.
    """
    if tool is None or tool is False:
        return None
    if tool is True:
        return TOOL
    t = np.asarray(tool, dtype=float)
    if t.shape != (4, 4):
        raise ValueError(f"tool must be a bool or a 4x4 transform, got shape {t.shape}")
    return t


def fk(q_deg, upto=6, tool=False):
    """Forward kinematics. Angles in degrees, returns 4x4 pose in metres.

    tool=False gives the flange, which is what the Annin oracle computes and
    what every existing test compares against -- do not change that default.
    tool=True gives the fingertip frame, which is what a grasp actually aims at
    and what app code should use. A 4x4 gives that tool frame instead.
    """
    m = np.eye(4)
    for i in range(upto):
        m = m @ link_transform(i, q_deg[i])
    t = tool_matrix(tool)
    if t is not None and upto == 6:
        m = m @ t
    return m


def grasp_pose(x_mm, y_mm, z_mm, yaw_deg=0.0, tilt_deg=0.0):
    """Fingertip pose for grasping at (x, y, z), tool pointing down.

    tilt_deg leans the approach away from vertical toward +x; 0 is straight
    down, which is what a chess board wants. yaw_deg spins the jaw line about
    the vertical, so a piece can be approached with the jaws across whichever
    axis has clearance.

    Returns a 4x4 in metres in the TOOL frame -- pass it to ik or plan_line
    with tool=True.
    """
    t = np.deg2rad(tilt_deg)
    z_axis = np.array([np.sin(t), 0.0, -np.cos(t)])
    x_axis = np.array([np.cos(t), 0.0, np.sin(t)])
    y_axis = np.cross(z_axis, x_axis)

    r = np.column_stack((x_axis, y_axis, z_axis))
    yaw = np.deg2rad(yaw_deg)
    c, s = np.cos(yaw), np.sin(yaw)
    r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ r

    pose = np.eye(4)
    pose[:3, :3] = r
    pose[:3, 3] = np.array([x_mm, y_mm, z_mm]) / 1000.0
    return pose


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