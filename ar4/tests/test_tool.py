"""The tool frame: aiming at the fingertips instead of the flange.

The gripper hangs 64 mm past the flange, so every pose the chess code cares
about is a fingertip pose. These tests pin the offset, prove the default is
still the flange (which is what the oracle comparison depends on), and check
that a line planned in tool space is straight for the FINGERTIP -- which is a
different curve from the flange's whenever the wrist rotates.
"""
import numpy as np
import pytest

from motion.kinematics import TOOL_MM, fk, grasp_pose
from motion.ik import ik, ik_multistart
from motion.trajectory import plan_line

from util import sample_q_regular


def _rot_gap_deg(a, b):
    c = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1.0) / 2.0
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def test_default_is_still_the_flange():
    """The oracle computes the flange. If fk's default ever moves to the tool,
    test_kinematics starts comparing two different frames and the 2e-4 mm
    agreement quietly becomes a 64 mm disagreement."""
    rng = np.random.default_rng(3)
    for _ in range(40):
        q = sample_q_regular(rng)
        assert np.array_equal(fk(q), fk(q, tool=False))


def test_tool_offset_is_purely_axial():
    rng = np.random.default_rng(4)
    for _ in range(40):
        q = sample_q_regular(rng)
        flange = fk(q)
        tip = fk(q, tool=True)

        delta = tip[:3, 3] - flange[:3, 3]
        axial = delta @ flange[:3, 2]
        lateral = np.linalg.norm(delta - flange[:3, 2] * axial)

        assert axial * 1000 == pytest.approx(TOOL_MM, abs=1e-6)
        assert lateral * 1000 < 1e-9, "offset must lie along z6, nowhere else"
        # a pure translation cannot change the orientation
        assert np.allclose(tip[:3, :3], flange[:3, :3])


def test_grasp_pose_points_down():
    straight_down = np.array([0.0, 0.0, -1.0])

    for yaw in (0.0, 45.0, -90.0, 180.0):
        pose = grasp_pose(350.0, 0.0, 30.0, yaw_deg=yaw)
        assert np.allclose(pose[:3, 2], straight_down, atol=1e-12), (
            "yaw spins the jaws about the vertical; it must not tip the approach"
        )

    # the fingertip lands exactly where it was asked to
    pose = grasp_pose(312.0, -84.0, 26.0, yaw_deg=17.0)
    assert np.allclose(pose[:3, 3] * 1000, [312.0, -84.0, 26.0])

    # tilt leans away from the base, toward +x
    tilted = grasp_pose(350.0, 0.0, 30.0, tilt_deg=30.0)
    assert tilted[0, 2] == pytest.approx(0.5, abs=1e-9)
    assert tilted[2, 2] == pytest.approx(-np.cos(np.deg2rad(30.0)), abs=1e-9)


def test_ik_round_trip_in_tool_frame():
    """Ask for a fingertip pose, and check the fingertip actually gets there."""
    targets = [(350, 0, 30, 0), (300, 150, 45, 30),
               (450, -120, 25, -45), (280, 200, 60, 90)]

    for x, y, z, yaw in targets:
        want = grasp_pose(x, y, z, yaw_deg=yaw)
        q = ik_multistart(want, tool=True)
        assert q is not None, f"({x},{y},{z}) should be reachable"

        got = fk(q, tool=True)
        assert np.linalg.norm(got[:3, 3] - want[:3, 3]) * 1000 < 0.05
        assert _rot_gap_deg(got, want) < 0.05


def test_tool_flag_moves_the_target_by_exactly_the_offset():
    """ik(tool=True) must be the same problem as ik on a pulled-back flange
    target -- not an approximation of it."""
    want = grasp_pose(340.0, 60.0, 40.0, yaw_deg=25.0)
    seed = ik_multistart(want, tool=True)
    assert seed is not None

    flange_target = want.copy()
    flange_target[:3, 3] -= want[:3, 2] * (TOOL_MM / 1000.0)

    a = ik(want, seed, tool=True)
    b = ik(flange_target, seed, tool=False)
    assert np.allclose(a, b, atol=1e-9)


def test_planned_line_is_straight_for_the_fingertip():
    """The payoff. With the approach axis rotating along the path, the flange
    and the fingertip sweep different curves; tool=True must straighten the one
    that matters.
    """
    # Held off the y=0 plane on purpose: the same move down the centreline
    # routes through the J5 singularity and the planner rightly refuses it.
    start_pose = grasp_pose(350.0, 120.0, 90.0, tilt_deg=0.0)
    end_pose = grasp_pose(430.0, 120.0, 90.0, tilt_deg=40.0)

    q0 = ik_multistart(start_pose, tool=True)
    assert q0 is not None
    traj = plan_line(q0, end_pose, tool=True)

    tips = np.array([fk(q, tool=True)[:3, 3] for q in traj.q])
    flanges = np.array([fk(q)[:3, 3] for q in traj.q])

    def max_deviation_mm(points):
        a, b = points[0], points[-1]
        d = b - a
        d = d / np.linalg.norm(d)
        rel = points - a
        perp = rel - np.outer(rel @ d, d)
        return np.max(np.linalg.norm(perp, axis=1)) * 1000

    tip_dev = max_deviation_mm(tips)
    flange_dev = max_deviation_mm(flanges)

    assert tip_dev < 0.05, f"fingertip should track a straight line, got {tip_dev:.3f} mm"
    # If these were equal the test would prove nothing -- the whole point is
    # that the two frames disagree once the wrist turns. Measured ~500x apart:
    # 0.006 mm at the fingertip against 2.9 mm at the flange.
    assert flange_dev > 1.5, (
        f"flange should bow away from its own chord ({flange_dev:.2f} mm); if it "
        "does not, this path no longer exercises a rotating approach axis"
    )


def test_vertical_grasp_workspace_bounds():
    """Regression on the measured envelope. A tournament board is 457 mm and
    only ~300 mm of depth admits a straight-down grasp -- if that ever changes,
    the board sizing decision changes with it.
    """
    z = 30.0
    for x in (250.0, 350.0, 450.0, 500.0):
        for y in (-250.0, 0.0, 250.0):
            assert ik_multistart(grasp_pose(x, y, z), tool=True) is not None, (
                f"({x},{y}) is inside the mapped envelope and must solve"
            )

    for x, y in ((650.0, 0.0), (700.0, 0.0), (200.0, 0.0)):
        assert ik_multistart(grasp_pose(x, y, z), tool=True) is None, (
            f"({x},{y}) is outside the envelope; a solution here means the "
            "reachability map in the manual is wrong"
        )
