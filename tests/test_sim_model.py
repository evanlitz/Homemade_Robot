"""The MJCF loads, and its joint convention is the same one motion/ speaks.

Both halves matter. A model that loads but whose joint_1 turns the other way
would look plausible on screen and send the arm to the mirror image of every
target the solver produces.
"""
import numpy as np
import pytest

from motion.kinematics import JOINT_LIMITS, fk
from util import sample_q

mujoco = pytest.importorskip("mujoco")

MODEL = "models/ar4.xml"

# MuJoCo world is the DH base frame turned -90 degrees about z. Established by
# fitting the wrist centre over 300 random configurations; the fit is constant
# to 0.9 deg, and the residual is the tool-point difference between the two
# frame conventions, not slop in the yaw.
BASE_YAW_DEG = -90.0

# The URDF and the DH table disagree slightly on link lengths -- d4 is 222.63 mm
# in the DH table and 0.22294 m in the URDF, a 0.31 mm difference that accounts
# for essentially all of the residual below. Not worth reconciling; it is well
# under the +/-5 mm tool accuracy the chess task needs.
DIMENSIONAL_SLOP_M = 5e-4


@pytest.fixture(scope="module")
def model():
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / MODEL
    if not (path.parent / "meshes" / "Link_1_Aluminum.STL").exists():
        pytest.skip("models/meshes/ not populated -- run tools/populate_meshes.py")
    return mujoco.MjModel.from_xml_path(str(path))


def _rz(deg):
    r = np.deg2rad(deg)
    return np.array([[np.cos(r), -np.sin(r), 0],
                     [np.sin(r), np.cos(r), 0],
                     [0, 0, 1]])


def test_model_loads(model):
    assert model.nq == 6
    assert model.nu == 6
    for i in range(6):
        assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) == f"joint_{i + 1}"


def test_joint_ranges_match_kinematics(model):
    for i, (lo, hi) in enumerate(JOINT_LIMITS):
        got = np.rad2deg(model.jnt_range[i])
        assert np.allclose(got, [lo, hi], atol=0.01), (
            f"joint_{i + 1} range {got} != {(lo, hi)} from motion.kinematics"
        )


def test_joint_convention_matches_dh(model):
    """qpos[i] must be the same J1..J6 the solver produces, including sign.

    Compared at the wrist centre (link_5 origin) rather than the tool: the two
    chains put the flange in different places, but the wrist centre is the
    intersection of the J4/J5/J6 axes in both, so it is a shared landmark.
    """
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_5")
    rot = _rz(BASE_YAW_DEG)

    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        q = sample_q(rng)
        data.qpos[:6] = np.deg2rad(q)
        mujoco.mj_forward(model, data)
        worst = max(worst, np.abs(data.xpos[body] - rot @ fk(q, upto=5)[:3, 3]).max())

    print(f"\nwrist centre MuJoCo vs Rz({BASE_YAW_DEG:.0f}).fk: worst {worst:.3e} m")
    assert worst < DIMENSIONAL_SLOP_M


def test_j1_sign_is_not_the_urdf_one(model):
    """Guards the one deliberate divergence from the URDF. If someone 'fixes'
    joint_1's axis back to the URDF's "0 0 1", this fails loudly instead of the
    arm quietly mirroring about the base."""
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_5")
    rot = _rz(BASE_YAW_DEG)

    q = np.array([40.0, 20.0, -20.0, 0.0, 30.0, 0.0])
    data.qpos[:6] = np.deg2rad(q)
    mujoco.mj_forward(model, data)
    right = np.abs(data.xpos[body] - rot @ fk(q, upto=5)[:3, 3]).max()

    mirrored = q.copy()
    mirrored[0] = -mirrored[0]
    wrong = np.abs(data.xpos[body] - rot @ fk(mirrored, upto=5)[:3, 3]).max()

    assert right < DIMENSIONAL_SLOP_M
    assert wrong > 0.05, "J1 sign convention is not being tested by this pose"
