"""Forward and inverse kinematics. Pure Python. No simulator, no hardware.

All angle and frame conventions are defined in `motion.conventions` and are
assumed, unmodified, by every function below.
"""

from __future__ import annotations

import numpy as np

from config import ArchitectureConfig

from .architectures import kinematics_for
from .conventions import JointVector, Pose


def forward_kinematics(cfg: ArchitectureConfig, theta: JointVector) -> Pose:
    """Tool pose from joint angles.

    ASSUMES: `theta` is ABSOLUTE (from horizontal), radians, base-outward
    ordering, in the order of `cfg.joints`. D2. See `motion.conventions`.

    Does NOT check joint limits. FK of an out-of-limit pose is a well-defined
    geometric question and callers sometimes want the answer; use
    `motion.limits.within_limits` if you care.

    For closed-loop architectures `theta` is the ACTUATED joint vector only.
    The passive joints are solved for internally from the loop constraint and
    are not part of the signature.
    """
    return kinematics_for(cfg).fk(cfg, np.asarray(theta, dtype=float))


def inverse_kinematics(cfg: ArchitectureConfig, pose: Pose) -> list[JointVector]:
    """All joint vectors that place the tool at `pose`.

    ASSUMES: `pose` is in the world frame, metres and radians, in the layout
    fixed by `motion.conventions`.

    Returns every geometric branch, unfiltered and unordered -- elbow-up and
    elbow-down, both delta assembly modes, both five-bar working modes.
    Returns [] when the pose is unreachable. Does NOT apply joint limits;
    filter with `motion.limits.within_limits`, then choose with
    `select_branch`.

    Closed-form per architecture. No numerical IK: a solver that converges to
    "a" solution hides exactly the branch structure this project is trying to
    measure.
    """
    return kinematics_for(cfg).ik(cfg, np.asarray(pose, dtype=float))


def select_branch(
    cfg: ArchitectureConfig,
    solutions: list[JointVector],
    previous: JointVector | None = None,
) -> JointVector | None:
    """Pick one IK branch.

    ASSUMES: every element of `solutions` already satisfies joint limits.

    Policy, not geometry: prefers the solution nearest `previous` in joint
    space so a trajectory does not flip branch mid-move. With `previous=None`
    the tiebreak is architecture-specific and must be stated in the model file
    header. Returns None for an empty list.
    """
    if not solutions:
        return None
    if previous is None:
        # Elbow-down: the lowest forearm angle. Stated here because the model
        # file header has to name a tiebreak and this is it.
        return min(solutions, key=lambda s: s[-1])
    prev = np.asarray(previous, dtype=float)
    return min(solutions, key=lambda s: float(np.linalg.norm(s - prev)))


def jacobian(cfg: ArchitectureConfig, theta: JointVector) -> np.ndarray:
    """Tool-velocity Jacobian, shape (len(pose), n_dof).

    ASSUMES: same angle convention as `forward_kinematics`.

    Maps joint rates to tool velocity in the world frame. Two consumers:
    static torque (tau = J^T @ wrench) and singularity detection (smallest
    singular value -> 0). Both are phase-1 deliverables.
    """
    return kinematics_for(cfg).jacobian(cfg, np.asarray(theta, dtype=float))


def tool_m_per_full_step(cfg: ArchitectureConfig, theta: JointVector) -> np.ndarray:
    """Tool displacement in metres for one FULL step of each motor, at `theta`.

    ASSUMES: full steps, NOT microsteps. CLAUDE.md constraint 4 -- incremental
    torque per microstep falls off as the sine of the microstep angle, so under
    load the rotor settles where the load balances. Microstepping is smoothness,
    not resolution, and must not appear in this calculation.

    Configuration-dependent: resolution is worst where the arm is most extended,
    so the phase-1 comparison number is the worst value over the board, not the
    value at home.
    """
    jac = jacobian(cfg, theta)
    step = np.array([j.drive.output_per_full_step(cfg.motor) for j in cfg.joints])
    return np.linalg.norm(jac, axis=0) * step
