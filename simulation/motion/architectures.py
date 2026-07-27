"""Per-architecture closed-form kinematics. Pure Python, absolute angles (D2).

One class per candidate, registered by `ArchitectureConfig.name`. The registry
is deliberately dumb: with five candidates a dict beats any plugin mechanism,
and a wrong architecture name should be a KeyError rather than a silent
fallback to something that happens to import.
"""

from __future__ import annotations

import math

import numpy as np

from config import ArchitectureConfig

from .conventions import JointVector, Pose


class PalletizerKinematics:
    """Candidate #6. theta = (q0_yaw, q1_upper_arm, q2_forearm), ABSOLUTE.

    In absolute coordinates the two planar links are INDEPENDENT: changing q1
    does not rotate link 2. FK is therefore a plain sum of link vectors, and
    the Jacobian columns for the planar joints have constant norm L1 and L2.
    Both facts are properties of D2, not of the mechanism.

    The tool drop is always vertical because the plate is held level by the two
    levelling parallelograms, so it subtracts from z and never from r.
    """

    n_dof = 3

    @staticmethod
    def fk(cfg: ArchitectureConfig, theta: JointVector) -> Pose:
        q0, q1, q2 = theta
        l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
        r = l1 * math.cos(q1) + l2 * math.cos(q2)
        z = cfg.g("shoulder_height_m") + l1 * math.sin(q1) + l2 * math.sin(q2) - cfg.g("tool_drop_m")
        return np.array([r * math.cos(q0), r * math.sin(q0), z])

    @staticmethod
    def ik(cfg: ArchitectureConfig, pose: Pose) -> list[JointVector]:
        x, y, z = pose
        l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
        zp = z - cfg.g("shoulder_height_m") + cfg.g("tool_drop_m")
        rho = math.hypot(x, y)
        yaw = math.atan2(y, x)

        out: list[JointVector] = []
        # Two yaw branches: face the target, or face away and reach backwards.
        for q0, r in ((yaw, rho), (_wrap(yaw + math.pi), -rho)):
            reach_sq = r * r + zp * zp
            denom = 2.0 * l1 * l2
            cos_phi = (reach_sq - l1 * l1 - l2 * l2) / denom
            # The fully-extended and fully-folded poses ARE reachable -- they are
            # the workspace boundary -- and they are exactly where |cos_phi|
            # rounds to just past 1. Rejecting on the raw comparison discards the
            # entire boundary shell, which is also where resolution is worst and
            # therefore the part of the workspace the gates care about most.
            if abs(cos_phi) > 1.0:
                if abs(cos_phi) - 1.0 > 1e-9:
                    continue
                cos_phi = math.copysign(1.0, cos_phi)
            base = math.atan2(zp, r)
            # Two elbow branches. phi is the interior bend, q2 - q1.
            for phi in ({math.acos(cos_phi), -math.acos(cos_phi)}):
                q1 = base - math.atan2(l2 * math.sin(phi), l1 + l2 * math.cos(phi))
                out.append(np.array([q0, _wrap(q1), _wrap(q1 + phi)]))
        return out

    @staticmethod
    def jacobian(cfg: ArchitectureConfig, theta: JointVector) -> np.ndarray:
        q0, q1, q2 = theta
        l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
        r = l1 * math.cos(q1) + l2 * math.cos(q2)
        c0, s0 = math.cos(q0), math.sin(q0)
        return np.array([
            [-r * s0, -l1 * math.sin(q1) * c0, -l2 * math.sin(q2) * c0],
            [r * c0, -l1 * math.sin(q1) * s0, -l2 * math.sin(q2) * s0],
            [0.0, l1 * math.cos(q1), l2 * math.cos(q2)],
        ])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


REGISTRY = {"palletizer": PalletizerKinematics}


def kinematics_for(cfg: ArchitectureConfig):
    return REGISTRY[cfg.name]
