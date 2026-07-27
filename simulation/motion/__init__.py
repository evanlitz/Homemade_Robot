"""Backend-free motion layer: FK, IK, limits, workspace, trajectories.

No simulator imports. No hardware imports. This package must stay importable
with nothing installed but numpy and pyyaml (the latter only via `config`).
"""

from .conventions import JointVector, Pose

__all__ = ["JointVector", "Pose"]
