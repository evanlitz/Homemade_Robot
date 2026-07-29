"""MuJoCo backend. Owns the mapping between `motion`'s actuated joint vector
and MuJoCo's `qpos`, which differs for every candidate.

One generator module per architecture, dispatched by `cfg.name`. Deliberately
dumb, for the same reason `motion.architectures.REGISTRY` is: with two
candidates a dict beats any plugin mechanism, and an unknown name should be a
KeyError rather than a silent fallback to whatever imported.
"""

from __future__ import annotations

from types import ModuleType

from config import ArchitectureConfig

from . import palletizer_mjcf, scara_mjcf

REGISTRY: dict[str, ModuleType] = {
    "palletizer": palletizer_mjcf,
    "scara": scara_mjcf,
}


def backend_for(cfg: ArchitectureConfig) -> ModuleType:
    """The MJCF module for this architecture.

    Every module in REGISTRY exposes the same three names: `build_mjcf(cfg)`,
    `qpos_from_absolute(model, *theta)` and `absolute_from_qpos(model, qpos)`.
    That contract is what lets `app.sweep` run either candidate unchanged.
    """
    return REGISTRY[cfg.name]
