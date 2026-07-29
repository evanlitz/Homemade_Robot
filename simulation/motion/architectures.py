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

from .collision import Capsule
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
    def tool_yaw(cfg: ArchitectureConfig, theta: JointVector) -> float:
        """Gripper heading. Always RADIAL -- it is the yaw joint itself, which
        D3 fixes to atan2(y, x). See ScaraKinematics.tool_yaw for the contrast."""
        return float(theta[0])

    @staticmethod
    def branch_key(theta: JointVector) -> float:
        """Elbow-DOWN: lowest forearm angle. Unchanged from the original
        `select_branch` tiebreak, restated here so each architecture owns its
        own rule instead of sharing one that happens to suit this one."""
        return float(theta[2])

    # Pairs whose interference no CAD layout can remove. See motion.collision.
    COLLISION_PAIRS = [("forearm", "column"), ("gripper", "column"),
                       ("gripper", "upper_arm")]

    @staticmethod
    def capsules(cfg: ArchitectureConfig, theta: JointVector):
        q0, q1, q2 = theta
        l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
        h, drop = cfg.g("shoulder_height_m"), cfg.g("tool_drop_m")
        rl, rc = cfg.g("link_radius_m"), cfg.g("column_radius_m")
        c0, s0 = math.cos(q0), math.sin(q0)

        def world(r: float, z: float) -> np.ndarray:
            """(radius, height) in the arm's vertical plane -> world xyz."""
            return np.array([r * c0, r * s0, z])

        shoulder = world(0.0, h)
        elbow = world(l1 * math.cos(q1), h + l1 * math.sin(q1))
        wrist = world(l1 * math.cos(q1) + l2 * math.cos(q2),
                      h + l1 * math.sin(q1) + l2 * math.sin(q2))
        tcp = wrist - np.array([0.0, 0.0, drop])
        moving = [Capsule("upper_arm", shoulder, elbow, rl),
                  Capsule("forearm", elbow, wrist, rl),
                  Capsule("gripper", wrist, tcp, rl * 0.7)]
        static = [Capsule("column", [0.0, 0.0, 0.0], [0.0, 0.0, h], rc)]
        return moving, static

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


class ScaraKinematics:
    """Candidate #8. theta = (q0_shoulder, q1_forearm, q2_lift), D2 ABSOLUTE.

    Two revolute joints about VERTICAL axes plus a prismatic lift. The planar
    pair is the same 2R chain as the palletizer's, rotated into the xy plane,
    so the same absolute-angle argument applies unchanged and FK is again a
    plain sum of link vectors.

    Two structural differences from the palletizer, and they are the reason
    this candidate is worth modelling:

    1. NO JACOBIAN COLUMN HAS NORM r. The palletizer's yaw column norm is the
       target's horizontal radius, which exceeds both link lengths at every
       surviving placement, so yaw sets its resolution. Here the two revolute
       columns have norm L1 and L2 and the prismatic column is a unit vector.
       Resolution is bounded by the LONGER LINK, not by the reach.
    2. BOTH REVOLUTE JOINTS ARE GRAVITY-FREE. Vertical axes carry no gravity
       torque at all. The whole weight goes to the lift.

    q2_lift is the carriage position in METRES above `lift_home_m`, not an
    angle. Every consumer that mixes units branches on `drive.is_linear`.
    """

    n_dof = 3

    @staticmethod
    def fk(cfg: ArchitectureConfig, theta: JointVector) -> Pose:
        q0, q1, d = theta
        l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
        return np.array([
            l1 * math.cos(q0) + l2 * math.cos(q1),
            l1 * math.sin(q0) + l2 * math.sin(q1),
            cfg.g("lift_home_m") + d - cfg.g("tool_drop_m"),
        ])

    @staticmethod
    def ik(cfg: ArchitectureConfig, pose: Pose) -> list[JointVector]:
        x, y, z = pose
        l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
        d = z - cfg.g("lift_home_m") + cfg.g("tool_drop_m")
        rho = math.hypot(x, y)

        cos_phi = (rho * rho - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        # Same boundary tolerance as the palletizer: the fully-extended pose IS
        # reachable and is exactly where |cos_phi| rounds past 1.
        if abs(cos_phi) > 1.0:
            if abs(cos_phi) - 1.0 > 1e-9:
                return []
            cos_phi = math.copysign(1.0, cos_phi)

        base = math.atan2(y, x)
        out: list[JointVector] = []
        for phi in {math.acos(cos_phi), -math.acos(cos_phi)}:
            q0 = base - math.atan2(l2 * math.sin(phi), l1 + l2 * math.cos(phi))
            out.append(np.array([_wrap(q0), _wrap(q0 + phi), d]))
        return out

    @staticmethod
    def jacobian(cfg: ArchitectureConfig, theta: JointVector) -> np.ndarray:
        q0, q1, _ = theta
        l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
        return np.array([
            [-l1 * math.sin(q0), -l2 * math.sin(q1), 0.0],
            [l1 * math.cos(q0), l2 * math.cos(q1), 0.0],
            [0.0, 0.0, 1.0],
        ])

    @staticmethod
    def tool_yaw(cfg: ArchitectureConfig, theta: JointVector) -> float:
        """Gripper heading. Fixed to link 2, so it is the forearm's own angle.

        NOT radial. The palletizer's gripper is always radial because its tool
        yaw is `atan2(y, x)` by construction; a SCARA gripper points wherever
        the forearm points, which for the same target is a different heading.
        D3-REQ's clearance check has to use this or it is testing the wrong
        footprint orientation.
        """
        return float(theta[1])

    @staticmethod
    def branch_key(theta: JointVector) -> float:
        """Consistently one elbow sense ("righty"), by the sign of the interior
        bend phi = q1 - q0.

        The palletizer's elbow-down rule would be degenerate here: its key is
        theta[2], which for SCARA is the LIFT, identical across both branches.
        A tiebreak that does not distinguish the branches would pick whichever
        order the solver happened to emit, silently making the torque figure
        depend on set iteration order.
        """
        return -float(_wrap(theta[1] - theta[0]))

    # The SAME three pairs as the palletizer, by the same rule. Symmetry here is
    # the whole point -- a collision check applied unequally scores the check,
    # not the architecture.
    COLLISION_PAIRS = [("link2", "column"), ("gripper", "column"),
                       ("gripper", "link1")]

    @staticmethod
    def capsules(cfg: ArchitectureConfig, theta: JointVector):
        q0, q1, d = theta
        l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
        drop = cfg.g("tool_drop_m")
        rl, rc = cfg.g("link_radius_m"), cfg.g("column_radius_m")
        z = cfg.g("lift_home_m") + d

        shoulder = np.array([0.0, 0.0, z])
        elbow = np.array([l1 * math.cos(q0), l1 * math.sin(q0), z])
        wrist = elbow + np.array([l2 * math.cos(q1), l2 * math.sin(q1), 0.0])
        tcp = wrist - np.array([0.0, 0.0, drop])
        moving = [Capsule("link1", shoulder, elbow, rl),
                  Capsule("link2", elbow, wrist, rl),
                  Capsule("gripper", wrist, tcp, rl * 0.7)]
        # The column runs the FULL height here, not just to the shoulder: the
        # carriage rides it, so it extends above the arm as well as below.
        static = [Capsule("column", [0.0, 0.0, 0.0],
                          [0.0, 0.0, cfg.g("column_height_m")], rc)]
        return moving, static


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


REGISTRY = {"palletizer": PalletizerKinematics, "scara": ScaraKinematics}


def kinematics_for(cfg: ArchitectureConfig):
    return REGISTRY[cfg.name]
