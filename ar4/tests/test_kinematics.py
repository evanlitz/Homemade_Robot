import numpy as np

from motion.kinematics import fk
from util import oracle_pose, pose_gap, sample_q


def test_fk_matches_annin(oracle):
    """Position *and* orientation. The position-only version of this test is
    what let the alpha6 = 180 bug through: with a6 = 0 the tool offset lands in
    the same place under both alpha6 values, so position cannot see it.

    Poses whose rotation vector is near pi are excluded. That is a defect in
    the oracle's *output representation*, not in either FK: at theta = pi the
    axis sign is ambiguous, and the oracle stores DH as float32, so the
    returned vector can land on the wrong branch. Those cases show ~1 deg of
    apparent error while position still agrees to 2e-4 mm.
    """
    rng = np.random.default_rng(0)
    worst_mm = worst_deg = 0.0
    skipped = 0

    for _ in range(500):
        q = sample_q(rng)
        o = np.array(oracle.forward_kinematics(list(q)))
        if abs(np.linalg.norm(o[3:]) - np.pi) < 0.05:
            skipped += 1
            continue
        mm, deg = pose_gap(fk(q), oracle_pose(oracle, q))
        worst_mm, worst_deg = max(worst_mm, mm), max(worst_deg, deg)

    print(f"\nworst {worst_mm:.2e} mm / {worst_deg:.2e} deg "
          f"over {500 - skipped} poses ({skipped} near-pi skipped)")
    assert skipped < 25, f"{skipped} poses near pi; the guard is too wide"
    assert worst_mm < 1e-3
    assert worst_deg < 1e-2
