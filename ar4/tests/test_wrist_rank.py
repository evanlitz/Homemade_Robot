"""Structural invariants on the arm geometry.

A 6R arm whose Jacobian is never full rank is not a 6-DOF arm. That is
checkable without any ground truth at all, and it is the check that caught
alpha6 = 180 after a position-only FK comparison passed against the oracle.
"""
import numpy as np

from motion.kinematics import DHM, jacobian, link_transform, _rx, _tx
from util import sample_q, sample_q_regular

# Joint origins and axes read from
# ../ar4_ros_driver/annin_ar4_description/urdf/ar_macro.xacro.
# (xyz, rpy extrinsic-XYZ, axis). Independent of the DH table and of the
# oracle: this comes from the CAD.
URDF_CHAIN = [
    ((0, 0, 0.092),           (np.pi, 0, 0),          (0, 0, 1)),
    ((0, 0.06415, -0.07778),  (1.5708, 0, -1.5708),   (0, 0, -1)),
    ((0, -0.305, 0),          (0, 0, 3.1416),         (0, 0, -1)),
    ((0, 0, 0),               (1.5708, 0, -1.5708),   (0, 0, -1)),
    ((0, 0, -0.22294),        (np.pi, 0, -1.5708),    (1, 0, 0)),
    ((0, 0, 0.041),           (0, 0, 0),              (0, 0, 1)),
]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _axis_rot(axis, theta):
    a = np.asarray(axis, dtype=float)
    a /= np.linalg.norm(a)
    kx = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1.0 - np.cos(theta)) * (kx @ kx)


def urdf_joint_axes(q_deg):
    """World-frame direction of each joint axis, straight from the URDF."""
    m, axes = np.eye(4), []
    for (xyz, rpy_, axis), qd in zip(URDF_CHAIN, q_deg):
        step = np.eye(4)
        step[:3, :3] = _rpy(*rpy_)
        step[:3, 3] = xyz
        m = m @ step
        a = np.asarray(axis, dtype=float)
        axes.append(m[:3, :3] @ (a / np.linalg.norm(a)))
        rot = np.eye(4)
        rot[:3, :3] = _axis_rot(axis, np.deg2rad(qd))
        m = m @ rot
    return axes


def dh_joint_axes(q_deg):
    """Same quantity from the DH table. Under modified DH joint i's axis is the
    z of the frame after Rx(alpha).Tx(a), not of the previous link frame."""
    base, axes = np.eye(4), []
    for i in range(6):
        alpha, a, _, _ = DHM[i]
        frame = base @ _rx(alpha) @ _tx(a / 1000.0)
        axes.append(frame[:3, :3] @ np.array([0.0, 0.0, 1.0]))
        base = base @ link_transform(i, q_deg[i])
    return axes


def test_jacobian_full_rank_away_from_singularities():
    """alpha6 = 180 fails this at 100% of configurations. The correct arm is
    full rank generically and drops rank only on the singular set, which the
    next test pins down separately."""
    rng = np.random.default_rng(7)
    worst = np.inf
    for _ in range(500):
        s = np.linalg.svd(jacobian(sample_q_regular(rng)), compute_uv=False)
        worst = min(worst, s[-1])
    print(f"\nsmallest min-singular-value over 500 regular configs: {worst:.3e}")
    assert worst > 1e-4, "wrist is rank deficient; check alpha6"


def test_wrist_singularity_is_isolated_at_j5_zero():
    """Documents the singularity rather than being surprised by it later.

    J5 = 0 makes the J4 and J6 axes collinear. This is real and expected on any
    spherical wrist -- trajectory planning has to route around it. Before the
    alpha6 fix the arm was rank-5 at *every* pose, so this structure did not
    exist to be found.
    """
    q = np.array([-58.5, 46.1, -37.3, 89.1, 0.0, -42.4])

    at_zero = np.linalg.svd(jacobian(q), compute_uv=False)[-1]
    assert at_zero < 1e-12, f"expected rank drop at J5=0, got {at_zero:.3e}"

    # Grows with |J5|, and near-symmetrically about zero. Only near-, because
    # the rest of the arm is not symmetric -- the two sides agree to ~1e-5
    # relative, not to machine precision.
    prev = 0.0
    for j5 in (1.0, 2.0, 5.0, 10.0):
        s = [np.linalg.svd(jacobian(np.r_[q[:4], sgn * j5, q[5]]),
                           compute_uv=False)[-1] for sgn in (-1.0, 1.0)]
        assert np.isclose(s[0], s[1], rtol=1e-3), "asymmetric about J5=0"
        assert s[0] > prev, "min singular value should grow with |J5|"
        prev = s[0]


def test_axis_geometry_matches_urdf():
    """Angles between consecutive joint axes are invariant to the base frame,
    so this compares DH against the CAD without reconciling frames -- which is
    just as well, since the two differ by ~300 mm of base offset."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = sample_q(rng)
        d, u = dh_joint_axes(q), urdf_joint_axes(q)
        for i in range(5):
            got = np.rad2deg(np.arccos(np.clip(abs(d[i] @ d[i + 1]), -1, 1)))
            want = np.rad2deg(np.arccos(np.clip(abs(u[i] @ u[i + 1]), -1, 1)))
            assert abs(got - want) < 1e-3, (
                f"J{i + 1}-J{i + 2} angle {got:.2f} deg, URDF says {want:.2f} deg"
            )


def test_wrist_axes_are_perpendicular():
    """The specific regression. Named so a failure says what it means."""
    rng = np.random.default_rng(1)
    for _ in range(100):
        axes = dh_joint_axes(sample_q(rng))
        assert abs(axes[4] @ axes[5]) < 1e-9, "J5 and J6 are not perpendicular"
