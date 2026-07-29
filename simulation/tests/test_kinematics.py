"""FK/IK property tests.

CLAUDE.md: sample JOINT space, not Cartesian space. For a grid of joint angles
within limits, compute p = FK(theta), then assert IK(p) recovers theta.
Sampling (x, y) instead produces false failures on unreachable points where IK
correctly has no solution.
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
from motion.limits import clamp, violations, within_limits

CFG = load_architecture("palletizer")


def joint_grid(per_joint: int = 7):
    axes = [np.linspace(j.min_rad, j.max_rad, per_joint) for j in CFG.joints]
    return [np.array(t) for t in np.array(np.meshgrid(*axes)).T.reshape(-1, len(axes))]


def same_angles(a, b, atol=1e-9) -> bool:
    """Compare joint vectors MODULO 2*pi.

    Raw comparison fails spuriously at the yaw limit: the placeholder range is
    +/-3.1416, which is 4e-6 rad wider than pi, so a commanded -3.1416 comes
    back as +3.14158 -- the same physical angle, wrapped. That is a property of
    the comparison, not of the IK.
    """
    d = (np.asarray(a) - np.asarray(b) + np.pi) % (2 * np.pi) - np.pi
    return bool(np.all(np.abs(d) < atol))


def test_ik_recovers_fk_over_joint_space():
    misses = []
    for theta in joint_grid():
        pose = forward_kinematics(CFG, theta)
        sols = inverse_kinematics(CFG, pose)
        if not any(same_angles(s, theta) for s in sols):
            misses.append((theta, pose, sols))
    assert not misses, f"{len(misses)} joint vectors not recovered, e.g. {misses[0]}"


def test_every_ik_solution_lands_on_the_requested_pose():
    for theta in joint_grid(5):
        pose = forward_kinematics(CFG, theta)
        for s in inverse_kinematics(CFG, pose):
            assert np.allclose(forward_kinematics(CFG, s), pose, atol=1e-9)


def test_unreachable_pose_returns_no_solutions():
    far = CFG.g("upper_arm_m") + CFG.g("forearm_m") + 0.5
    assert inverse_kinematics(CFG, np.array([far, 0.0, 0.0])) == []


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


def test_absolute_convention_planar_columns_have_constant_norm():
    """D2 property: in absolute angles the planar links are independent, so
    column i of the Jacobian has norm exactly L_i regardless of pose. This
    fails under the parent-relative convention, so it is a live check that
    D2 is actually in force."""
    for theta in joint_grid(5):
        cols = np.linalg.norm(jacobian(CFG, theta), axis=0)
        assert cols[1] == pytest.approx(CFG.g("upper_arm_m"), abs=1e-12)
        assert cols[2] == pytest.approx(CFG.g("forearm_m"), abs=1e-12)


def test_resolution_uses_full_steps_not_microsteps():
    """CLAUDE.md constraint 4: microstepping is smoothness, not resolution."""
    theta = np.array([0.0, 0.0, 0.0])
    res = tool_m_per_full_step(CFG, theta)
    expected_yaw = abs(CFG.g("upper_arm_m") + CFG.g("forearm_m")) * (
        math.radians(CFG.motor.full_step_deg) / CFG.joints[0].drive.reduction)
    assert res[0] == pytest.approx(expected_yaw, rel=1e-12)
    assert CFG.joints[0].drive.microsteps not in res


def test_limits():
    lo = np.array([j.min_rad for j in CFG.joints])
    # lo is inside every ABSOLUTE range by construction, and its relative elbow
    # is lo[2] - lo[1] = -90 deg, inside the +/-150 deg mechanism limit.
    assert within_limits(CFG, lo)
    assert not within_limits(CFG, lo - 0.01)
    # Three absolute breaches; the relative elbow is unchanged by shifting all
    # three joints equally, so it does not add a fourth.
    assert len(violations(CFG, lo - 0.01)) == 3


def test_relative_elbow_limit_is_enforced_independently_of_absolute():
    """D19: a pose can satisfy every absolute range and still be a fold the
    mechanism cannot make.

    The excluded region is the 60 deg band around full doubling-back: relative
    angles between +150 and +210 deg, which wrap onto each other and are the
    same physical configuration. The palletizer's absolute ranges alone admit
    that band, because q2 - q1 reaches -210 deg and -210 wraps to +150.

    Substituting the relative limit FOR the absolute one -- which is what the
    first D19 attempt did -- raised the survivor count from 7 to 16 by silently
    discarding the table-clearance limit. Both must apply.
    """
    q1, q2 = CFG.joints[1], CFG.joints[2]
    theta = np.array([0.0, q1.max_rad, -1.9])  # relative -199 deg == +161 deg
    assert q1.min_rad <= theta[1] <= q1.max_rad, "absolute q1 must be legal"
    assert q2.min_rad <= theta[2] <= q2.max_rad, "absolute q2 must be legal"
    assert not within_limits(CFG, theta)
    assert any("relative to" in v for v in violations(CFG, theta))

    # And a normal working fold is accepted.
    assert within_limits(CFG, np.array([0.0, 1.0, -0.6]))


def test_clamp_refuses_relative_limits_rather_than_guessing():
    with pytest.raises(NotImplementedError):
        clamp(CFG, np.array([0.0, 0.0, 0.0]))
