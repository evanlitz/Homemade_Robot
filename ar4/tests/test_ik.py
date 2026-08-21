"""Round-trip checks on the damped least-squares solver.

The test is on the *pose*, never on the joint angles: the AR4 has multiple
solutions for most poses, so ik(fk(q)) is under no obligation to return q.
"""
import numpy as np

from motion.kinematics import fk
from motion.ik import ik, ik_multistart
from util import WRIST_SINGULAR_BAND_DEG, pose_gap, sample_q, sample_q_regular


def test_ik_round_trip():
    """Poses clear of the wrist singularity must all solve. Near J5 = 0 the
    damped solver deliberately trades accuracy for stability and stalls a few
    microns short of tolerance -- that is DLS working as designed, not a bug,
    and test_ik_degrades_gracefully_at_singularity covers it."""
    rng = np.random.default_rng(0)
    misses, worst_mm, worst_deg = 0, 0.0, 0.0

    for _ in range(200):
        q = sample_q_regular(rng, inset=0.1)
        target = fk(q)
        sol = ik_multistart(target)

        if sol is None:
            misses += 1
            continue

        mm, deg = pose_gap(fk(sol), target)
        worst_mm, worst_deg = max(worst_mm, mm), max(worst_deg, deg)
        assert mm < 0.05, f"converged but {mm:.3f} mm off at q={q}"
        assert deg < 0.05, f"converged but {deg:.3f} deg off at q={q}"

    rate = 1.0 - misses / 200
    print(f"\nconvergence {rate:.1%}, worst {worst_mm:.2e} mm / {worst_deg:.2e} deg")
    assert rate == 1.0, f"{misses} regular poses did not converge"


def test_ik_from_nearby_seed():
    """The case trajectory.py will actually hit: small step, warm seed."""
    rng = np.random.default_rng(1)
    for _ in range(100):
        q = sample_q_regular(rng, inset=0.2)
        seed = q + rng.uniform(-3.0, 3.0, 6)
        sol = ik(fk(q), seed)
        assert sol is not None, f"failed from a 3-degree seed at q={q}"
        mm, deg = pose_gap(fk(sol), fk(q))
        assert mm < 0.05 and deg < 0.05


def test_ik_degrades_gracefully_at_singularity():
    """At the wrist singularity the solver may return None, but when it does
    return something it must not be silently wrong. This is the property
    trajectory.py depends on to decide whether a waypoint is usable.

    Warm-seeded rather than multistart: it is the case trajectory.py actually
    hits, and it keeps the test off the 30-seed cold path, which spends most of
    its time proving that unreachable-to-tolerance poses are unreachable.
    """
    rng = np.random.default_rng(4)
    returned = 0
    for _ in range(60):
        q = sample_q(rng, inset=0.15)
        q[4] = rng.uniform(-1.0, 1.0)     # inside the singular band
        sol = ik(fk(q), q + rng.uniform(-2.0, 2.0, 6))
        if sol is None:
            continue
        returned += 1
        mm, deg = pose_gap(fk(sol), fk(q))
        assert mm < 0.05, f"returned a {mm:.3f} mm solution near J5=0"
        assert deg < 0.05, f"returned a {deg:.3f} deg solution near J5=0"
    print(f"\nnear-singular: {returned}/60 returned a solution, all accurate")


def test_singular_band_constant_is_honest():
    """The band used to filter the other tests should actually bracket the
    trouble, so it cannot be quietly widened to hide a regression."""
    assert WRIST_SINGULAR_BAND_DEG <= 10.0


def test_ik_rejects_unreachable():
    """Two metres out along x is outside a 600 mm arm."""
    target = fk(np.zeros(6)).copy()
    target[0, 3] = 2.0
    assert ik_multistart(target) is None
