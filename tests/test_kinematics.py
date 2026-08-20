import numpy as np
import pytest

# The oracle is a pybind11 .pyd linked against python312.dll. Skip rather than
# error if it will not import, so a broken checkout does not block the suite.
K = pytest.importorskip("ARrobots.robot_kinematics")

from motion.kinematics import fk, JOINT_LIMITS


def test_fk_matches_annin():
    K.robot_data_reset()
    K.robot_set()
    lo = [b[0] for b in JOINT_LIMITS]
    hi = [b[1] for b in JOINT_LIMITS]
    rng = np.random.default_rng(0)
    for _ in range(500):
        q = rng.uniform(lo, hi)
        ref = np.array(K.forward_kinematics(list(q)))[:3]
        assert np.allclose(ref, fk(q)[:3, 3] * 1000, atol=1e-3)
