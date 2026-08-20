"""Numerical check on the analytical Jacobian.

Central differences of fk() reconstruct each column by a route that shares no
code with the analytical derivation, so a wrong axis frame or a wrong rotation
origin shows up as one bad column rather than a plausible-looking matrix.
"""
import numpy as np

from motion.kinematics import fk, jacobian, JOINT_LIMITS


def numerical_jacobian(q_deg, h_rad=1e-6):
    """Central-difference Jacobian, per radian to match jacobian()."""
    h_deg = np.rad2deg(h_rad)
    j = np.zeros((6, 6))
    r0 = fk(q_deg)[:3, :3]

    for i in range(6):
        qp = np.asarray(q_deg, dtype=float).copy()
        qm = qp.copy()
        qp[i] += h_deg
        qm[i] -= h_deg
        tp, tm = fk(qp), fk(qm)

        j[:3, i] = (tp[:3, 3] - tm[:3, 3]) / (2.0 * h_rad)

        # Angular velocity from the skew part of Rdot . R^T. Deliberately not
        # ik.pose_error's axis-angle extraction -- reusing that would let a
        # shared convention error cancel out and pass.
        w = ((tp[:3, :3] - tm[:3, :3]) / (2.0 * h_rad)) @ r0.T
        j[3:, i] = [w[2, 1], w[0, 2], w[1, 0]]

    return j


def test_jacobian_matches_numerical():
    lo = [b[0] for b in JOINT_LIMITS]
    hi = [b[1] for b in JOINT_LIMITS]
    rng = np.random.default_rng(0)

    worst = 0.0
    for _ in range(100):
        q = rng.uniform(lo, hi)
        a, n = jacobian(q), numerical_jacobian(q)
        err = np.abs(a - n)
        worst = max(worst, err.max())
        bad = np.argmax(err.max(axis=0))
        assert np.allclose(a, n, atol=1e-6), (
            f"column {bad} (J{bad + 1}) off by {err.max():.3e} at q={q}"
        )

    print(f"\nworst entry error across 100 poses: {worst:.3e}")


def test_jacobian_zero_pose():
    """Regression anchor: the home pose, so a refactor has something exact."""
    q = np.zeros(6)
    assert np.allclose(jacobian(q), numerical_jacobian(q), atol=1e-6)
