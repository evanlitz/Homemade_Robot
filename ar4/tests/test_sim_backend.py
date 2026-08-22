"""SimBackend's half of the Backend contract, especially follow()."""
from pathlib import Path

import numpy as np
import pytest

from motion.kinematics import fk
from motion.trajectory import Trajectory, plan_path

pytest.importorskip("mujoco")
from backends.sim import JAW_TRAVEL_M, SETPOINT_LEAD_S, SimBackend  # noqa: E402

Q_HOME = np.array([0.0, 20.0, -20.0, 0.0, 45.0, 0.0])
ROOT = Path(__file__).resolve().parents[1]

# The chess move the lead compensation was tuned against: lift 40 mm, traverse
# two 57 mm squares, descend.
def chess_path():
    start = fk(Q_HOME)
    lift = start.copy()
    lift[2, 3] += 0.040
    over = lift.copy()
    over[1, 3] += 0.114
    down = over.copy()
    down[2, 3] -= 0.040
    return plan_path(Q_HOME, [lift, over, down])


@pytest.fixture
def backend():
    if not (ROOT / "models" / "meshes" / "Link_1_Aluminum.STL").exists():
        pytest.skip("models/meshes/ not populated -- run tools/populate_meshes.py")
    b = SimBackend(model_path=str(ROOT / "models" / "ar4.xml"),
                   render=False, realtime=False)
    b.connect()
    yield b
    b.disconnect()


def test_follow_ends_at_rest_on_the_last_waypoint(backend):
    path = chess_path()
    backend.move_joints(path.q[0], speed=100)
    got = np.array(backend.follow(path))

    assert np.max(np.abs(got - path.q[-1])) < 0.1
    assert np.abs(backend.d.qvel[:6]).max() < 1e-6, "follow must block until at rest"


def test_follow_arrives_before_the_settle_loop(backend, monkeypatch):
    """Streaming alone should land close, with settling only mopping up.

    Measured at the end of streaming, with _settle disabled, because settling
    hides tracking error entirely.

    Note this understates the worst case: a trapezoidal profile decelerates to
    rest at the last waypoint, so lag there is small by construction. The
    error that actually matters peaks mid-path at full speed -- 9.2 mm of wrist
    error uncompensated against 0.9 mm with the lead. Asserting *that* needs a
    way to observe during the move, which follow() deliberately does not expose;
    if it ever grows a progress callback, tighten this test to use it.
    """
    monkeypatch.setattr(backend, "_settle", lambda *a, **k: None)
    path = chess_path()
    backend.move_joints(path.q[0], speed=100)
    backend.follow(path)

    err = np.max(np.abs(np.rad2deg(backend.d.qpos[:6]) - path.q[-1]))
    assert err < 0.35, f"tracking error {err:.3f} deg -- check SETPOINT_LEAD_S"


def test_uncompensated_streaming_is_worse(backend, monkeypatch):
    """Guards against someone deleting the lead as needless complication.

    2x rather than the 10x seen mid-path, for the reason in the test above --
    this is the tail of the move, where the gap is smallest. A weak signal in
    the right direction beats asserting a number this seam cannot see.
    """
    monkeypatch.setattr(backend, "_settle", lambda *a, **k: None)
    path = chess_path()

    backend.move_joints(path.q[0], speed=100)
    backend.follow(path)
    led = np.max(np.abs(np.rad2deg(backend.d.qpos[:6]) - path.q[-1]))

    backend.setpoint_lead_s = 0.0
    backend.move_joints(path.q[0], speed=100)
    backend.follow(path)
    raw = np.max(np.abs(np.rad2deg(backend.d.qpos[:6]) - path.q[-1]))

    assert raw > 2 * led, f"lead {led:.3f} deg vs none {raw:.3f} deg"


def test_single_waypoint_trajectory(backend):
    backend.move_joints(Q_HOME, speed=100)
    target = Q_HOME + np.array([5.0, 0, 0, 0, 0, 0])
    got = np.array(backend.follow(Trajectory(target.reshape(1, 6), 0.02)))
    assert np.max(np.abs(got - target)) < 0.1


def test_empty_trajectory_is_a_no_op(backend):
    backend.move_joints(Q_HOME, speed=100)
    got = np.array(backend.follow(Trajectory(np.zeros((0, 6)), 0.02)))
    assert np.max(np.abs(got - Q_HOME)) < 0.1


def test_follow_only_needs_q_and_dt(backend):
    """The ABC promises backends do not depend on motion.trajectory."""
    class Duck:
        q = np.vstack([Q_HOME, Q_HOME + np.array([3.0, 0, 0, 0, 0, 0])])
        dt = 0.02

    backend.move_joints(Q_HOME, speed=100)
    got = np.array(backend.follow(Duck()))
    assert np.max(np.abs(got - Duck.q[-1])) < 0.1


def test_setpoint_lead_is_in_the_measured_range():
    """40 ms was the sweep optimum; kv/kp predicts 50 ms and measures 2.7x
    worse. If this drifts far from the measurement, re-run the sweep."""
    assert 0.02 <= SETPOINT_LEAD_S <= 0.06


def test_set_gripper_spans_the_documented_travel(backend):
    """0 -> 1 must map onto the full 28 mm the URDF gives the SG1.

    Chess needs this number to be right, not approximately right: at 28 mm
    the jaws cannot close on a tournament piece's base, only its stem, and
    that is what sets the grasp height.
    """
    backend.set_gripper(0.0)
    assert backend.d.qpos[6:8].sum() == pytest.approx(0.0, abs=1e-4)

    backend.set_gripper(1.0)
    opening = backend.d.qpos[6:8].sum()
    assert opening == pytest.approx(2 * JAW_TRAVEL_M, abs=1e-4)
    assert opening == pytest.approx(0.028, abs=1e-4)

    backend.set_gripper(0.5)
    assert backend.d.qpos[6:8].sum() == pytest.approx(JAW_TRAVEL_M, abs=1e-4)


def test_set_gripper_leaves_the_arm_where_it_was(backend):
    """Actuating the gripper steps the sim, so the arm keeps settling under
    gravity while it runs. That drift has to stay far below the 5 mm task
    budget or a grasp would nudge the tool off the square."""
    backend.move_joints(Q_HOME, speed=100)
    before = np.rad2deg(backend.d.qpos[:6].copy())
    backend.set_gripper(1.0)
    backend.set_gripper(0.0)
    after = np.rad2deg(backend.d.qpos[:6])

    assert np.max(np.abs(after - before)) < 0.05
