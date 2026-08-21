"""Reachable workspace computation and board coverage.

CLAUDE.md: every architecture needs an explicitly computed reachable workspace.
Never assume a clean annulus -- joint limits and branch selection carve pieces
out of it.
"""

from __future__ import annotations

import numpy as np

from config import ArchitectureConfig

from .conventions import JointVector, Pose


def sample_joint_space(cfg: ArchitectureConfig, per_joint: int) -> np.ndarray:
    """Grid of joint vectors spanning the configured limits, shape (N, n_dof).

    ASSUMES: ABSOLUTE radians (D2), `cfg.joints` order.

    Joint-space sampling, not Cartesian sampling. Sampling (x, y, z) and asking
    whether IK succeeds conflates "unreachable" with "IK is broken"; sampling
    joint space and pushing through FK cannot produce an unreachable point.
    """
    raise NotImplementedError


def reachable_points(cfg: ArchitectureConfig, per_joint: int) -> np.ndarray:
    """Tool positions reachable within joint limits, shape (N, 3), metres.

    ASSUMES: world frame. FK applied to `sample_joint_space`, then filtered by
    `motion.limits.within_limits` and by self-collision if a collision model is
    available. Points near a singularity are retained here and flagged by
    `singularity_margin`, because "reachable" and "usable" are different
    questions and the phase-1 report wants both.
    """
    raise NotImplementedError


def is_reachable(cfg: ArchitectureConfig, pose: Pose) -> bool:
    """True if at least one IK branch for `pose` satisfies all joint limits.

    ASSUMES: world frame, `motion.conventions` pose layout.
    """
    raise NotImplementedError


def board_coverage(cfg: ArchitectureConfig, board_pose: Pose) -> float:
    """Fraction of the 64 board squares whose centre is reachable, 0.0 to 1.0.

    ASSUMES: `board_pose` places the board's centre in the world frame with the
    board plane horizontal. Square centres come from `cfg.task.board_square_m`.

    Centres only. A reachable centre does not imply a reachable approach for a
    piece of finite height, and this number will read optimistically until a
    tool offset and an approach vector exist.
    """
    raise NotImplementedError


def singularity_margin(cfg: ArchitectureConfig, theta: JointVector) -> float:
    """Smallest singular value of the Jacobian at `theta`.

    ASSUMES: same convention as `motion.kinematics.jacobian`.

    Units are mixed when the pose vector mixes metres and radians, so the
    absolute value means nothing across architectures -- only its approach to
    zero within one architecture does. Any cross-architecture comparison needs
    a scaled Jacobian, and that scaling has to be stated before the number is
    reported.
    """
    raise NotImplementedError

