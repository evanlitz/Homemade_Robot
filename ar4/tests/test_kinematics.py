import numpy as np

from motion.kinematics import fk
from util import oracle_pose, pose_gap, sample_q


def test_fk_matches_annin(oracle):
    """Position *and* orientation. The position-only version of this test is
    what let the alpha6 = 180 bug through: with a6 = 0 the tool offset lands in
    the same place under both alpha6 values, so position cannot see it.

    Poses whose rotation is near pi are excluded from the ORIENTATION check. That is a defect in
    the oracle's *output representation*, not in either FK: at theta = pi the
    axis sign is ambiguous, and the oracle stores DH as float32, so the
    returned vector can land on the wrong branch. Those cases show ~1 deg of
    apparent error while position still agrees to 2e-4 mm.

    The guard measures closeness to pi on OUR rotation, not on the oracle's
    returned vector. Near pi the oracle's vector is not just on the wrong
    branch but short: at q = [23.1, 3.4, 18.0, -2.3, 68.0, -53.3] the true
    angle is 0.0008 rad from pi and the oracle reports 0.14 rad from it, 8 deg
    off, with position agreeing to 5e-5 mm. Asking the oracle whether its own
    output is in the region where its output is wrong let that pose through.
    """
    rng = np.random.default_rng(0)
    worst_mm = worst_deg = 0.0
    skipped = 0

    for _ in range(500):
        q = sample_q(rng)
        mm, deg = pose_gap(fk(q), oracle_pose(oracle, q))
        # position is unaffected by the defect, so it is checked everywhere
        worst_mm = max(worst_mm, mm)
        r = fk(q)[:3, :3]
        angle = np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
        if np.pi - angle < 0.15:
            skipped += 1
            continue
        worst_deg = max(worst_deg, deg)

    print(f"\nworst {worst_mm:.2e} mm over 500 poses, {worst_deg:.2e} deg over "
          f"{500 - skipped} ({skipped} near-pi excluded from orientation)")
    assert skipped < 100, f"{skipped} poses near pi; the guard is too wide"
    assert worst_mm < 1e-3
    assert worst_deg < 1e-2
