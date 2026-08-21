"""FK/IK property tests for candidate #8. Same discipline as the palletizer's.

CLAUDE.md: sample JOINT space, not Cartesian space.

The two architecture-specific tests at the bottom are the ones worth having.
Everything above them is the same shape as `test_kinematics.py` and would have
caught the same class of bug.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from config import load_architecture
from motion.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    jacobian,
    tool_m_per_full_step,
)
from motion.limits import within_limits

CFG = load_architecture("scara")


def joint_grid(per_joint: int = 7):
    axes = [np.linspace(j.min_rad, j.max_rad, per_joint) for j in CFG.joints]
    return [np.array(t) for t in np.array(np.meshgrid(*axes)).T.reshape(-1, len(axes))]


def same_state(a, b, atol=1e-9) -> bool:
    """Angles compared modulo 2*pi, the lift compared directly.

    A single modulo comparison across all three would wrap the LIFT, silently
    treating a carriage at 0 m and at 2*pi m as equal.
    """
    a, b = np.asarray(a), np.asarray(b)
    ang = (a[:2] - b[:2] + np.pi) % (2 * np.pi) - np.pi
    return bool(np.all(np.abs(ang) < atol) and abs(a[2] - b[2]) < atol)


def test_ik_recovers_fk_over_joint_space():
    misses = []
    for theta in joint_grid():
        pose = forward_kinematics(CFG, theta)
        sols = inverse_kinematics(CFG, pose)
        if not any(same_state(s, theta) for s in sols):
            misses.append((theta, pose, sols))
    assert not misses, f"{len(misses)} joint vectors not recovered, e.g. {misses[0]}"


def test_every_ik_solution_lands_on_the_requested_pose():
    for theta in joint_grid(5):
        pose = forward_kinematics(CFG, theta)
        for s in inverse_kinematics(CFG, pose):
            assert np.allclose(forward_kinematics(CFG, s), pose, atol=1e-9)


def test_unreachable_pose_returns_no_solutions():
    far = CFG.g("upper_arm_m") + CFG.g("forearm_m") + 0.5
    assert inverse_kinematics(CFG, np.array([far, 0.0, 0.02])) == []


def test_jacobian_matches_finite_difference():
    for theta in joint_grid(4):
        analytic = jacobian(CFG, theta)
        numeric = np.zeros((3, 3))
        for i in range(3):
            step = np.zeros(3)
            step[i] = 1e-7
            numeric[:, i] = (forward_kinematics(CFG, theta + step)
                             - forward_kinematics(CFG, theta - step)) / 2e-7
        assert np.allclose(analytic, numeric, atol=1e-6)


def test_absolute_convention_columns_have_constant_norm():
    """D2 property, same as the palletizer's: in absolute angles the two links
    are independent, so column i has norm exactly L_i at every pose."""
    for theta in joint_grid(5):
        cols = np.linalg.norm(jacobian(CFG, theta), axis=0)
        assert cols[0] == pytest.approx(CFG.g("upper_arm_m"), abs=1e-12)
        assert cols[1] == pytest.approx(CFG.g("forearm_m"), abs=1e-12)


def test_no_jacobian_column_scales_with_reach():
    """THE structural claim this candidate is being modelled to test.

    The palletizer's yaw column has norm r, the target's horizontal radius,
    which exceeds both link lengths at every surviving placement -- so its
    resolution is set by reach. SCARA's columns are bounded by link length and
    by 1 (the prismatic unit vector). Resolution therefore cannot be degraded
    by pushing the board outward, which is the difference the comparison is
    for. Asserted, not assumed.
    """
    bound = max(CFG.g("upper_arm_m"), CFG.g("forearm_m"), 1.0)
    for theta in joint_grid(5):
        assert np.linalg.norm(jacobian(CFG, theta), axis=0).max() <= bound + 1e-12


def test_prismatic_step_is_metres_not_radians():
    """`output_per_full_step` returns mixed units by design. A lift drive with
    lead L at reduction N moves L/(steps_per_rev * N) metres per full step, and
    nothing in the resolution path may reinterpret that as an angle."""
    lift = CFG.joints[2].drive
    assert lift.is_linear
    expected = lift.lead_m_per_rev / (CFG.motor.full_steps_per_rev * lift.reduction)
    assert lift.output_per_full_step(CFG.motor) == pytest.approx(expected, rel=1e-12)
    with pytest.raises(ValueError):
        lift.joint_rad_per_full_step(CFG.motor)
    # The prismatic column is a unit vector, so tool travel == joint travel.
    res = tool_m_per_full_step(CFG, np.array([0.3, -0.2, 0.1]))
    assert res[2] == pytest.approx(expected, rel=1e-12)


def test_limits_reject_lift_outside_travel():
    ok = np.array([0.0, 0.0, CFG.joints[2].max_rad])
    assert within_limits(CFG, ok)
    assert not within_limits(CFG, ok + np.array([0.0, 0.0, 0.01]))


def test_self_collision_detects_a_known_interference():
    """A pose that folds link 2 back through the column must be caught.

    The check is geometric and lives outside the physics because collision is
    disabled in both models (F1). If it cannot catch a hand-constructed
    collision it is not testing anything.
    """
    from motion.architectures import ScaraKinematics
    from motion.collision import clearance

    folded = np.array([0.0, math.pi, 0.10])  # link 2 doubled straight back
    moving, static = ScaraKinematics.capsules(CFG, folded)
    gap, who = clearance(moving, static, ScaraKinematics.COLLISION_PAIRS)
    assert gap < 0.0, f"folded pose reported clear: {gap:.4f} m at {who}"

    extended = np.array([0.0, 0.0, 0.10])  # straight out, obviously clear
    moving, static = ScaraKinematics.capsules(CFG, extended)
    gap, _ = clearance(moving, static, ScaraKinematics.COLLISION_PAIRS)
    assert gap > 0.0


def test_segment_distance_matches_hand_computed_cases():
    from motion.collision import segment_distance

    z = np.zeros(3)
    # Parallel segments, 1 m apart.
    assert segment_distance(z, np.array([1.0, 0, 0]),
                            np.array([0, 1.0, 0]),
                            np.array([1.0, 1.0, 0])) == pytest.approx(1.0)
    # Crossing segments share a point.
    assert segment_distance(np.array([-1.0, 0, 0]), np.array([1.0, 0, 0]),
                            np.array([0, -1.0, 0]),
                            np.array([0, 1.0, 0])) == pytest.approx(0.0)
    # Endpoint-to-endpoint: the clamped solve must not slide off the segment.
    assert segment_distance(z, np.array([1.0, 0, 0]),
                            np.array([3.0, 0, 0]),
                            np.array([4.0, 0, 0])) == pytest.approx(2.0)
