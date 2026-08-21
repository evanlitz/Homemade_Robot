"""Trajectory generation: the path is straight, the profile respects its
limits, and bad segments are refused rather than silently mangled."""
import numpy as np
import pytest

from motion.kinematics import fk
from motion.trajectory import (
    MAX_JOINT_STEP_DEG, PlanningError, Trajectory, interpolate_pose,
    matrix_from_rotvec, plan_line, plan_path, rotvec_from_matrix,
    trapezoid_duration, trapezoid_profile,
)

Q_HOME = np.array([0.0, 20.0, -20.0, 0.0, 45.0, 0.0])


def line_deviation_mm(traj, start, end):
    """Worst perpendicular distance from the straight line, in mm."""
    d = end[:3, 3] - start[:3, 3]
    n = np.linalg.norm(d)
    if n < 1e-12:
        return 0.0
    worst = 0.0
    for q in traj.q:
        v = fk(q)[:3, 3] - start[:3, 3]
        worst = max(worst, np.linalg.norm(np.cross(v, d)) / n)
    return worst * 1000.0


# --- profile --------------------------------------------------------------

def test_trapezoid_reaches_cruise_when_long_enough():
    # 400 mm at 250 mm/s, 500 mm/s^2: 0.5 s ramp each end, 1.1 s cruise
    t, v = trapezoid_duration(400.0, 250.0, 500.0)
    assert v == pytest.approx(250.0)
    assert t == pytest.approx(2.1)


def test_trapezoid_degenerates_to_triangle_when_short():
    """A 20 mm chess lift never reaches cruise speed."""
    t, v = trapezoid_duration(20.0, 250.0, 500.0)
    assert v == pytest.approx(100.0)          # sqrt(L * a)
    assert t == pytest.approx(0.4)
    assert v < 250.0


def test_profile_respects_velocity_and_acceleration():
    dt = 0.005
    t, s = trapezoid_profile(400.0, 250.0, 500.0, dt=dt)
    assert s[0] == 0.0 and s[-1] == pytest.approx(400.0)
    assert np.all(np.diff(s) >= -1e-9), "profile must be monotonic"

    v = np.diff(s) / dt
    a = np.diff(v) / dt
    assert v.max() <= 250.0 * 1.02
    assert np.abs(a).max() <= 500.0 * 1.05


def test_profile_stretches_to_a_longer_duration():
    """Used when rotation binds: the path is unchanged, just slower."""
    dt = 0.005
    _, fast = trapezoid_profile(100.0, 250.0, 500.0, dt=dt)
    _, slow = trapezoid_profile(100.0, 250.0, 500.0, duration=3.0, dt=dt)
    assert len(slow) > len(fast)
    assert slow[-1] == pytest.approx(100.0)
    assert (np.diff(slow) / dt).max() < (np.diff(fast) / dt).max()


# --- rotation helpers -----------------------------------------------------

def test_rotvec_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(200):
        v = rng.normal(size=3)
        v = v / np.linalg.norm(v) * rng.uniform(0, np.pi - 1e-3)
        assert np.allclose(rotvec_from_matrix(matrix_from_rotvec(v)), v, atol=1e-9)


def test_rotvec_survives_theta_near_pi():
    """The off-diagonal formula divides by sin(theta) and blows up here; the
    symmetric-part branch is what keeps slerp usable through a half turn."""
    for axis in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]),
                 np.array([1.0, 1.0, 0]) / np.sqrt(2)):
        v = axis * (np.pi - 1e-9)
        got = rotvec_from_matrix(matrix_from_rotvec(v))
        assert np.linalg.norm(got) == pytest.approx(np.pi, abs=1e-6)
        assert np.allclose(np.abs(got / np.linalg.norm(got)), np.abs(axis), atol=1e-4)


def test_interpolate_pose_hits_both_ends():
    a = fk(Q_HOME)
    b = fk(Q_HOME + np.array([10.0, 5.0, -5.0, 0.0, 5.0, 20.0]))
    assert np.allclose(interpolate_pose(a, b, 0.0), a, atol=1e-12)
    assert np.allclose(interpolate_pose(a, b, 1.0), b, atol=1e-9)


# --- planning -------------------------------------------------------------

def test_line_is_actually_straight():
    """The whole reason this module exists. Joint-space interpolation between
    the same endpoints bows out by centimetres."""
    start = fk(Q_HOME)
    end = start.copy()
    end[:3, 3] = start[:3, 3] + np.array([0.10, 0.05, -0.03])

    traj = plan_line(Q_HOME, end)
    assert line_deviation_mm(traj, start, end) < 0.05

    # the joint-space alternative, for contrast
    naive = np.linspace(traj.q[0], traj.q[-1], len(traj))
    bow = max(
        np.linalg.norm(np.cross(fk(q)[:3, 3] - start[:3, 3],
                                end[:3, 3] - start[:3, 3]))
        / np.linalg.norm(end[:3, 3] - start[:3, 3])
        for q in naive) * 1000.0
    assert bow > 1.0, "joint interpolation should visibly bow; test is not proving anything"


def test_endpoints_are_hit():
    start = fk(Q_HOME)
    end = start.copy()
    end[:3, 3] = start[:3, 3] + np.array([0.05, -0.04, 0.02])
    traj = plan_line(Q_HOME, end)

    assert np.allclose(traj.q[0], Q_HOME)
    assert np.linalg.norm(fk(traj.q[-1])[:3, 3] - end[:3, 3]) * 1000 < 0.05


def test_joint_path_is_continuous():
    start = fk(Q_HOME)
    end = start.copy()
    end[:3, 3] = start[:3, 3] + np.array([0.08, 0.06, 0.02])
    traj = plan_line(Q_HOME, end)
    assert np.abs(np.diff(traj.q, axis=0)).max() < MAX_JOINT_STEP_DEG


def test_pure_rotation_is_paced_by_the_wrist():
    """Zero translation must not produce an instantaneous move."""
    start = fk(Q_HOME)
    end = start.copy()
    end[:3, :3] = matrix_from_rotvec([0, 0, np.deg2rad(30)]) @ start[:3, :3]
    traj = plan_line(Q_HOME, end)
    assert traj.duration > 0.3
    assert len(traj) > 10


def test_zero_motion_is_a_single_waypoint():
    traj = plan_line(Q_HOME, fk(Q_HOME))
    assert len(traj) == 1
    assert traj.duration == 0.0


def test_unreachable_target_reports_where():
    start = fk(Q_HOME)
    end = start.copy()
    end[0, 3] = 2.0
    with pytest.raises(PlanningError) as e:
        plan_line(Q_HOME, end)
    assert e.value.index > 0
    assert 0.0 < e.value.u <= 1.0


def test_path_through_wrist_singularity_is_refused():
    """J5 -> 0 is reachable but must not be planned through: the arm loses
    orientation control there."""
    target = fk(np.array([0.0, 20.0, -20.0, 0.0, 2.0, 0.0]))
    with pytest.raises(PlanningError, match="wrist singularity"):
        plan_line(Q_HOME, target)


def test_validation_can_be_switched_off():
    """So a caller can inspect a rejected path instead of only being told no."""
    target = fk(np.array([0.0, 20.0, -20.0, 0.0, 2.0, 0.0]))
    traj = plan_line(Q_HOME, target, validate=False)
    assert abs(traj.q[-1][4]) < 4.0


def test_plan_path_chains_segments():
    """The chess shape: lift, traverse two squares, descend."""
    start = fk(Q_HOME)
    lift = start.copy()
    lift[2, 3] += 0.040
    over = lift.copy()
    over[1, 3] += 0.114
    down = over.copy()
    down[2, 3] -= 0.040

    path = plan_path(Q_HOME, [lift, over, down])

    assert np.allclose(path.q[0], Q_HOME)
    assert np.linalg.norm(fk(path.q[-1])[:3, 3] - down[:3, 3]) * 1000 < 0.05
    assert np.abs(np.diff(path.q, axis=0)).max() < MAX_JOINT_STEP_DEG
    # no duplicated waypoint at the joins
    assert np.abs(np.diff(path.q, axis=0)).max(axis=1).min() > 0.0


def test_trajectory_container():
    q = np.zeros((5, 6))
    t = Trajectory(q, 0.1)
    assert len(t) == 5
    assert t.duration == pytest.approx(0.4)
    assert np.allclose(t.times, [0, 0.1, 0.2, 0.3, 0.4])
    assert t.joint_speeds().shape == (4, 6)
