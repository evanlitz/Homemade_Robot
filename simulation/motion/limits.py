"""Joint limits, speed caps and workspace clamps.

CLAUDE.md constraint 5: safety modelling belongs in the motion layer from the
start. This module is a hard gate that every commanded motion passes through,
in sim and on hardware alike -- not an advisory check.
"""

from __future__ import annotations

import math

import numpy as np

from config import ArchitectureConfig

from .conventions import JointVector, Pose


def within_limits(cfg: ArchitectureConfig, theta: JointVector) -> bool:
    """True if every joint angle is inside its configured range, inclusive.

    ASSUMES: `theta` is ABSOLUTE radians (D2) in `cfg.joints` order, the same
    convention `forward_kinematics` takes. A limit expressed in the sim's frame
    and applied to a vector in another frame fails silently and plausibly,
    which is why the convention is stated in one place and not per-function.
    """
    return not violations(cfg, theta)


def violations(cfg: ArchitectureConfig, theta: JointVector) -> list[str]:
    """Human-readable description of each limit breach. Empty list if clean."""
    out = []
    for joint, value in zip(cfg.joints, np.asarray(theta, dtype=float)):
        if not joint.min_rad <= value <= joint.max_rad:
            out.append(f"{joint.name}={math.degrees(value):.2f} deg outside "
                       f"[{math.degrees(joint.min_rad):.2f}, {math.degrees(joint.max_rad):.2f}]")
    return out


def clamp(cfg: ArchitectureConfig, theta: JointVector) -> JointVector:
    """Clamp each joint into range.

    NOT a safety mechanism. Clamping a trajectory point silently moves the tool
    somewhere other than commanded. Use for visualisation and for seeding
    solvers; use `within_limits` to decide whether a motion may execute.
    """
    lo = np.array([j.min_rad for j in cfg.joints])
    hi = np.array([j.max_rad for j in cfg.joints])
    return np.clip(np.asarray(theta, dtype=float), lo, hi)


def check_velocity(cfg: ArchitectureConfig, theta_dot: JointVector) -> bool:
    """True if every joint rate is within `max_vel_rad_s`.

    ASSUMES: `theta_dot` is in radians/second at the JOINT, after reduction --
    not motor shaft rate. Motor-side limits are a `backends/hw_backend`
    concern; this layer never sees a motor.
    """
    cap = np.array([j.max_vel_rad_s for j in cfg.joints])
    return bool(np.all(np.abs(np.asarray(theta_dot, dtype=float)) <= cap))


def check_acceleration(cfg: ArchitectureConfig, theta_ddot: JointVector) -> bool:
    """True if every joint acceleration is within `max_accel_rad_s2`.

    ASSUMES: joint-side rad/s^2, same as `check_velocity`. This is the limit
    that keeps a stepper from losing steps, so it is a real constraint and not
    a comfort setting.
    """
    cap = np.array([j.max_accel_rad_s2 for j in cfg.joints])
    return bool(np.all(np.abs(np.asarray(theta_ddot, dtype=float)) <= cap))


def workspace_clamp(cfg: ArchitectureConfig, pose: Pose) -> Pose:
    """Clamp a Cartesian pose into the allowed operating volume.

    ASSUMES: world frame, +Z up, origin at the base mounting face.

    The allowed volume is a configured box, deliberately smaller than the
    reachable workspace: it excludes the near-singular fully-extended shell
    where a small joint error becomes a large tool error.
    """
    raise NotImplementedError


def max_joint_torque_nm(cfg: ArchitectureConfig) -> np.ndarray:
    """Per-joint torque ceiling from motor holding torque, reduction, efficiency.

    Holding torque is a static rating. Torque at speed is lower and depends on
    drive voltage and inductance, neither of which is modelled or purchased.
    Treat the result as an upper bound that hardware will not reach.

    NOTE: this is the MOTOR-side ceiling. D10 caps torque far below it by
    driver current, because the belt cannot carry motor capability -- see
    `ValiditySpec.joint_torque_cap_nm`, which is the limit that actually binds.
    """
    return np.array([j.drive.joint_torque_limit_nm(cfg.motor) for j in cfg.joints])

