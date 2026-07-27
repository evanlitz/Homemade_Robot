"""Trajectory generation. Joint-space and Cartesian-space paths.

Output of this module is what both backends consume: `sim_backend` tracks it,
`hw_backend` turns it into step/dir pulses. Same trajectory, both times.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from config import ArchitectureConfig

from .conventions import JointVector, Pose


@dataclass(frozen=True)
class Trajectory:
    """Time-sampled joint-space path.

    ASSUMES: `theta` is (N, n_dof) ABSOLUTE radians (D2) in `cfg.joints`
    order; `t` is (N,) seconds, monotonic, starting at 0.0. `theta_dot` and
    `theta_ddot` are the analytic derivatives of the generating spline, not
    finite differences of `theta` -- finite differences of a resampled path
    understate peak acceleration, which is the value the stepper cares about.
    """

    t: np.ndarray
    theta: np.ndarray
    theta_dot: np.ndarray
    theta_ddot: np.ndarray


def joint_move(
    cfg: ArchitectureConfig,
    start: JointVector,
    goal: JointVector,
    dt: float,
) -> Trajectory:
    """Time-optimal joint-space move honouring per-joint vel/accel limits.

    ASSUMES: `start` and `goal` are ABSOLUTE radians (D2) and both already
    satisfy `motion.limits.within_limits`. This function does not check them.

    The tool path is NOT a straight line in Cartesian space. Fine for a
    move-above-the-board transit, wrong for a descent onto a piece.
    """
    raise NotImplementedError


def cartesian_move(
    cfg: ArchitectureConfig,
    start: Pose,
    goal: Pose,
    dt: float,
) -> Trajectory:
    """Straight-line tool path, resolved to joints by IK at each sample.

    ASSUMES: world frame poses, `motion.conventions` layout.

    Every sample runs IK and `select_branch` seeded with the previous sample,
    so the path cannot flip branch mid-move. Raises if any intermediate sample
    is unreachable -- endpoints being reachable does not make the segment
    between them reachable, and a straight line can leave the workspace and
    re-enter it.
    """
    raise NotImplementedError


def pick_and_place(
    cfg: ArchitectureConfig,
    from_square: str,
    to_square: str,
    board_pose: Pose,
    dt: float,
) -> Trajectory:
    """Full chess move: approach, descend, grasp, lift, transit, descend, release.

    ASSUMES: algebraic square names ("e2", "e4"); `board_pose` as in
    `motion.workspace.board_coverage`. Vertical segments are Cartesian,
    the transit between squares is joint-space.

    This is the phase-1 benchmark task -- every candidate runs this same
    trajectory so peak torque and cycle time are measured on identical terms.
    """
    raise NotImplementedError


def validate(cfg: ArchitectureConfig, traj: Trajectory) -> list[str]:
    """Every limit breach in a trajectory. Empty list means safe to execute.

    Checks position, velocity and acceleration limits at every sample. This is
    the gate `backends/hw_backend` must call before emitting a single pulse.
    """
    raise NotImplementedError

