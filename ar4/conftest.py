"""Resolve imports relative to this file, not the shell's working directory.

Puts the package root on sys.path so `motion` / `backends` import, and the
sibling HMI checkout on sys.path so the `ARrobots` oracle imports. pytest loads
this before collection, so both work regardless of where pytest is invoked from.
"""
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).parent
# ar4-hmi is a checkout alongside the repo, not inside it, so this climbs out
# of ar4/ and then out of the repo root.
sys.path[:0] = [str(_ROOT), str(_ROOT.parent.parent / "ar4-hmi")]


@pytest.fixture(scope="session")
def oracle():
    """Annin's compiled kinematics, with alpha6 corrected to -90.

    robot_set() installs alpha6 = 180, which puts the J6 axis exactly
    antiparallel to J5 and makes the arm rank-5. Any test using the oracle as
    ground truth has to override it first, so do it in one place.

    Note the two calls disagree on layout: get_dh_parameters() returns one row
    per link as [theta, alpha, a, d]; set_dh_parameters_explicit() takes 24
    scalars grouped by parameter (6 thetas, then 6 alphas, ...). Hence the .T.
    """
    import numpy as np

    k = pytest.importorskip("ARrobots.robot_kinematics")
    k.robot_data_reset()
    k.robot_set()
    dh = np.array(k.get_dh_parameters())
    dh[5, 1] = -np.pi / 2
    k.set_dh_parameters_explicit(*dh.T.flatten())
    return k
