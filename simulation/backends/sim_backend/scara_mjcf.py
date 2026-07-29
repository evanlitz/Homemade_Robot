"""Generate the SCARA+Z MJCF from config, and map absolute angles to qpos.

Candidate #8. D18 scope: coverage, peak torque, resolution. Nothing else.

Same generation approach as the palletizer for the same reason (D9 sweeps link
lengths), and the same `qpos_from_absolute` contract so `app.sweep` can drive
either architecture without knowing which it has.

WHAT IS DELIBERATELY IDENTICAL to the palletizer, per D18: link cross-section
and density, motor mass and placement style, collision disabled, timestep,
solver settings, and the 1:25 two-stage drive on every joint. The comparison is
only meaningful if the assumptions are shared, including the wrong ones.
"""

from __future__ import annotations

import numpy as np

from config import ArchitectureConfig

ACTUATED = ("q0_shoulder", "q1_forearm_rel", "q2_lift")

HEADER = """<!--
  GENERATED FILE -- do not edit. Source: backends/sim_backend/scara_mjcf.py
  Candidate #8, SCARA + Z. DECISIONS.md D18.

  ============================ ANGLE CONVENTIONS ============================
  DECISIONS.md D2: angles are ABSOLUTE, measured from the +x axis, in the
  horizontal plane. Same convention as the palletizer, different plane.

  Plane   : the two revolute joints work in the world xy plane.
  Axis    : both revolute hinges are axis="0 0 1". Positive rotation carries
            +x toward +y, so an absolute angle read as atan2(y, x) INCREASES
            and the cumulative-sum property of D2 holds with the natural sign.
            (The palletizer needs axis="0 -1 0" for the same property; that is
            a consequence of working in a vertical plane, not a convention
            difference.)
  Zero    : both hinges at 0 is the fully-extended pose along +x. SINGULAR for
            the planar 2R chain -- reference pose only.
  Lift    : q2_lift is a SLIDE along +z, in METRES, measured from lift_home_m.
            It is not an angle. Anything that converts steps to output must
            branch on DriveSpec.is_linear.

  ================================ TOPOLOGY =================================
  3 DOF, open chain. No loops, no equality constraints, no cut joints.

    world
     `- lift (q2_lift, slide z)         carriage: BOTH revolute motors ride here
         `- link1 (q0_shoulder)         S->E, length L1
             `- link2 (q1_forearm_rel)  E->T, length L2
                 `- tool                gripper, drop d

  BOTH REVOLUTE MOTORS ARE ON THE CARRIAGE, not at the joints they drive. The
  elbow is belt-driven from a shoulder-coaxial pulley along link 1. This is what
  satisfies CLAUDE.md hard constraint 1 -- a motor at the elbow would ride link 1
  and spend its own torque budget being carried. It is also what makes the
  forearm angle naturally ABSOLUTE: holding the elbow pulley while link 1 turns
  leaves the forearm pointing the same way in world space.

  The belt itself is not modelled, exactly as for the palletizer (F5). Its mass
  lands on the carriage, so it loads the lift and not the revolute joints.

  ================================= NOTES ===================================
  Collision is OFF (contype/conaffinity 0), matching the palletizer. There the
  reason was F1 -- coincident bodies at a loop cut. Here there is no loop and no
  such need, but SELF-COLLISION IS THEREFORE NOT CHECKED FOR EITHER CANDIDATE.
  A folded SCARA pose that puts link 2 through the column is not rejected. Same
  omission both sides, so the comparison is fair; it is still an omission.

  ARMATURE on all three joints is reflected rotor inertia. On the slide joint
  MuJoCo's armature is a MASS, and the reflected mass of a rotary drive through
  a lead is J_rotor * (2*pi*N/lead)^2 -- large for any fine lead. DECISIONS.md
  F6 is the same physics.

  NO EQUALITY CONSTRAINTS, so there is no loop residual to gate. The residual
  gate simply does not fire for this candidate. That is a real difference in
  what the model can get wrong, not an oversight.
-->
"""


def build_mjcf(cfg: ArchitectureConfig) -> str:
    l1 = cfg.g("upper_arm_m")
    l2 = cfg.g("forearm_m")
    d = cfg.g("tool_drop_m")
    home = cfg.g("lift_home_m")
    col = cfg.g("column_height_m")
    mm = cfg.motor.mass_kg
    # From config, identical to palletizer.yaml. See the note there.
    rl = cfg.g("link_radius_m")
    rc = cfg.g("column_radius_m")

    by_name = {j.name: j for j in cfg.joints}
    arm_q0 = by_name["q0_shoulder"].drive.reflected_inertia_kgm2(cfg.motor)
    arm_q1 = by_name["q1_forearm"].drive.reflected_inertia_kgm2(cfg.motor)
    arm_q2 = by_name["q2_lift"].drive.reflected_inertia_kgm2(cfg.motor)

    return f"""{HEADER}<mujoco model="scara">
  <compiler angle="radian"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"
          solver="Newton" iterations="200" tolerance="1e-14"/>

  <default>
    <geom type="capsule" density="2700" contype="0" conaffinity="0"
          size="{rl:.6f}" rgba="0.62 0.64 0.68 1"/>
    <joint type="hinge" axis="0 0 1" damping="0.05"/>
    <site type="sphere" size="0.010" rgba="1 0.35 0.1 1"/>
  </default>

  <worldbody>
    <geom name="column" type="capsule" fromto="0 0 0  0 0 {col:.6f}" size="{rc:.6f}"
          rgba="0.30 0.30 0.32 1"/>
    <!-- lift motor: grounded at the column base, drives the carriage -->
    <geom name="motor_lift" type="box" size="0.043 0.043 0.058"
          pos="0 0 -0.058" mass="{mm}" rgba="0.20 0.20 0.22 1"/>

    <body name="carriage" pos="0 0 {home:.6f}">
      <joint name="q2_lift" type="slide" axis="0 0 1" damping="1.0"
             armature="{arm_q2:.6f}"/>
      <geom name="g_carriage" type="box" size="0.060 0.050 0.030"
            rgba="0.40 0.42 0.46 1"/>
      <!-- both revolute motors ride the carriage, coaxial at the shoulder -->
      <geom name="motor_q0" type="box" size="0.043 0.043 0.058"
            pos="0 0.050 0.088" mass="{mm}" rgba="0.20 0.20 0.22 1"/>
      <geom name="motor_q1" type="box" size="0.043 0.043 0.058"
            pos="0 -0.050 0.088" mass="{mm}" rgba="0.20 0.20 0.22 1"/>

      <body name="link1" pos="0 0 0">
        <joint name="q0_shoulder" armature="{arm_q0:.6f}"/>
        <geom name="g_link1" fromto="0 0 0  {l1:.6f} 0 0"/>
        <site name="shoulder" pos="0 0 0"/>

        <body name="link2" pos="{l1:.6f} 0 0">
          <joint name="q1_forearm_rel" armature="{arm_q1:.6f}"/>
          <geom name="g_link2" fromto="0 0 0  {l2:.6f} 0 0"/>
          <site name="elbow" pos="0 0 0"/>

          <body name="tool" pos="{l2:.6f} 0 0">
            <geom name="g_gripper" size="0.014" fromto="0 0 0  0 0 {-d:.6f}"
                  rgba="0.85 0.55 0.15 1"/>
            <site name="tcp" pos="0 0 {-d:.6f}" size="0.012"
                  rgba="0.15 0.85 0.35 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="a_q0"   joint="q0_shoulder"    kp="4000" kv="200"/>
    <position name="a_q1"   joint="q1_forearm_rel" kp="4000" kv="200"/>
    <position name="a_lift" joint="q2_lift"        kp="40000" kv="2000"/>
  </actuator>
</mujoco>
"""


def actuated_forces(model, data) -> np.ndarray:
    """Generalised forces conjugate to the ABSOLUTE actuated coordinates.

    Order matches `cfg.joints`: (q0_shoulder, q1_forearm, q2_lift). Units are
    N.m, N.m, N -- the last one is a FORCE, and callers must not take a max
    across the three.

    THE TRANSFORM MATTERS HERE AND IT DID NOT FOR THE PALLETIZER. MuJoCo returns
    forces conjugate to the tree coordinates, and the elbow's tree coordinate is
    RELATIVE (q1_forearm_rel). Both motors sit on the carriage and drive
    ABSOLUTE angles -- that is what the elbow belt along link 1 does. Equating
    work in the two coordinate sets, with theta_rel = A theta_abs and
    A = [[1, 0], [-1, 1]]:

        tau_abs = A^T tau_rel   ->   tau_abs0 = tau_rel0 - tau_rel1
                                     tau_abs1 = tau_rel1

    Skipping this reports the shoulder torque of a machine whose elbow motor is
    bolted to the elbow, which is the arrangement CLAUDE.md hard constraint 1
    forbids -- so it would be the wrong machine, not a small error.

    The palletizer needs no equivalent because its three actuated bodies are all
    children of the rotating base and their coordinates are already absolute.
    """
    import mujoco

    def force(name: str) -> float:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(data.qfrc_inverse[model.jnt_dofadr[jid]])

    tau_rel0, tau_rel1 = force("q0_shoulder"), force("q1_forearm_rel")
    return np.array([tau_rel0 - tau_rel1, tau_rel1, force("q2_lift")])


def hinge_values(q0: float, q1a: float, d: float) -> dict[str, float]:
    """D2 translation layer: absolute angles -> MJCF joint values.

    Only the forearm needs translating; link 1's parent is the carriage, which
    does not rotate. The lift is already in its own units and passes through.
    """
    return {"q0_shoulder": q0, "q1_forearm_rel": q1a - q0, "q2_lift": d}


def qpos_from_absolute(model, q0: float, q1a: float, d: float) -> np.ndarray:
    import mujoco

    qpos = np.zeros(model.nq)
    for name, value in hinge_values(q0, q1a, d).items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos[model.jnt_qposadr[jid]] = value
    return qpos


def absolute_from_qpos(model, qpos: np.ndarray) -> tuple[float, float, float]:
    import mujoco

    def get(name: str) -> float:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(qpos[model.jnt_qposadr[jid]])

    q0 = get("q0_shoulder")
    return q0, q0 + get("q1_forearm_rel"), get("q2_lift")
