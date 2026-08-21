"""Generate the palletizer MJCF from config, and map absolute angles to qpos.

The model is GENERATED, not hand-written, because D9 sweeps link lengths --
a static XML with baked numbers would go stale the moment the sweep runs.
`models/palletizer.xml` is a regenerated snapshot for reading, not the source.

This module owns D2's translation layer: `motion/` speaks absolute angles and
never sees qpos. See `qpos_from_absolute`.
"""

from __future__ import annotations

import numpy as np

from config import ArchitectureConfig

# Body order is not relied on -- joints are always addressed by name.
PLANAR_JOINTS = (
    "q1_upper_arm",
    "q2_forearm_rel",
    "tool_plate",
    "elbow_yoke",
    "rod_b2",
    "drive_lever",
    "push_rod",
    "rod_b1",
)

ACTUATED = ("q0_yaw", "q1_upper_arm", "drive_lever")

# (rod-side site, mechanism-side site) for each closed loop.
CUT_PAIRS = (
    ("cut_drive_rod", "cut_drive"),
    ("cut_level1_rod", "cut_level1"),
    ("cut_level2_rod", "cut_level2"),
)

HEADER = """<!--
  GENERATED FILE -- do not edit. Source: backends/sim_backend/palletizer_mjcf.py
  Candidate #6, parallelogram palletizer. DECISIONS.md D1.

  ============================ ANGLE CONVENTIONS ============================
  DECISIONS.md D2: angles are ABSOLUTE, measured from horizontal.

  Plane   : the arm works in the xz plane of the rotating base frame.
  Axis    : every planar hinge is axis="0 -1 0".  NOT "0 1 0".
            With +y, a positive hinge rotation carries +z toward +x, so an
            absolute angle read as atan2(z, x) DECREASES and the cumulative-sum
            property of D2 fails. With -y, absolute angle is the plain running
            sum of hinge values from the base. This is a correctness
            requirement of D2, not a style choice.
  Zero    : all hinges at 0 is the fully-extended horizontal pose. That pose is
            SINGULAR for the planar 2-link chain and is a reference/assembly
            pose only -- never an operating pose.
  Yaw     : q0_yaw about +z, standard right-handed.

  Absolute angles of interest, given actuated (q1a, q2a):
      upper_arm  q1a        forearm    q2a
      tool_plate 0 (level)  elbow_yoke 0 (level)
  D2 payoff visible here: BOTH actuated planar hinges are absolute angles
  directly (upper_arm and drive_lever are children of the base, which does not
  tilt), so the actuated set needs no translation at all. Only the passive
  joints do. See qpos_from_absolute.

  ================================ TOPOLOGY =================================
  3 DOF: base yaw + two grounded planar drives. No elbow motor. All motors are
  grounded or on the rotating base (CLAUDE.md hard constraint 1).

    base (q0_yaw)
     |- upper_arm (q1_upper_arm) ---- S->E, length L1
     |   |- forearm (q2_forearm_rel)  E->T, length L2, plus a backward stub a
     |   |   `- tool_plate            level; carries the TCP, drop d
     |   `- elbow_yoke                level; arm length b
     |       `- rod_b2                parallel to forearm, length L2
     |- drive_lever (actuated)        S->P, length a, points opposite forearm
     |   `- push_rod                  parallel to upper arm, length L1
     `- rod_b1                        parallel to upper arm, length L1

  Three closed loops, three cuts, three <connect> constraints:
    L-drive   push_rod end  <-> forearm stub tip   sets forearm abs = lever abs
    L-level1  rod_b1 end    <-> elbow_yoke tip     holds yoke level
    L-level2  rod_b2 end    <-> tool_plate tip     holds tool plate level
  Levelling is two CHAINED parallelograms, not one: the only constant-angle
  reference available is the rotating base, so the yoke must be levelled off the
  base before the tool plate can be levelled off the yoke.

  DOF: 9 tree joints - 3 loops x 2 independent rows = 3.
  (<connect> writes 3 rows each, 9 nominal; the out-of-plane row of each is
  redundant on a planar loop -- DECISIONS.md F4.)

  ================================= NOTES ===================================
  Collision is OFF on every geom (contype/conaffinity 0). A closed loop puts
  the cut bodies in permanent coincident contact by construction; leaving
  collision on generates spurious contacts of hundreds of newtons that silently
  corrupt every internal load. DECISIONS.md F1.

  solref/solimp are tightened past MuJoCo defaults. At defaults this class of
  loop sits at ~0.39 mm residual, 7.8% of the +/-5 mm accuracy budget, consumed
  as pure modelling artefact. DECISIONS.md F2, OPEN-A.

  JOINT LIMITS ARE NOT IN THIS FILE. They live in config and are enforced by
  motion/limits.py (CLAUDE.md: safety belongs in the motion layer). Duplicating
  a placeholder limit here would let the two drift apart silently.

  ARMATURE on the three actuated joints is reflected rotor inertia, J_rotor*N^2.
  At 1:25 that is 625x the rotor figure and comparable to the whole arm's
  inertia. It is physics, not a solver tweak: without it this model diverges on
  EVERY integrator within 0.02-0.18 s. DECISIONS.md F6. Passive loop joints
  carry no motor and correctly get no armature.

  TIMESTEP must stay at or below 0.002. At 0.004 the model diverges even with
  armature present. DECISIONS.md F6.

  KNOWN MODELLING GAP: the 1:25 belt reduction stack is not modelled. Two 159 mm
  aluminium pulleys per joint is roughly 1 kg each, mounted on the rotating base.
  That mass affects yaw inertia and is currently missing. DECISIONS.md F5.
-->
"""


def build_mjcf(cfg: ArchitectureConfig) -> str:
    h = cfg.g("shoulder_height_m")
    l1 = cfg.g("upper_arm_m")
    l2 = cfg.g("forearm_m")
    a = cfg.g("drive_lever_m")
    b = cfg.g("level_offset_m")
    d = cfg.g("tool_drop_m")
    mo = cfg.g("motor_mount_offset_m")
    mm = cfg.motor.mass_kg
    # From config, not hardcoded: motion.collision needs the same radii to size
    # clearance, and the link radius also sizes link MASS through the capsule
    # density. Two copies would drift.
    rl = cfg.g("link_radius_m")
    rc = cfg.g("column_radius_m")

    eq = 'solref="0.002 1" solimp="0.999 0.9999 0.0001 0.5 2"'

    # Reflected rotor inertia, J_rotor * N^2, on the ACTUATED joints only.
    # Physics, not a solver tweak -- without it the model diverges on every
    # integrator. Passive loop joints carry no motor and get no armature.
    # DECISIONS.md F6.
    by_name = {j.name: j for j in cfg.joints}
    arm_yaw = by_name["q0_yaw"].drive.reflected_inertia_kgm2(cfg.motor)
    arm_q1 = by_name["q1_upper_arm"].drive.reflected_inertia_kgm2(cfg.motor)
    arm_q2 = by_name["q2_forearm"].drive.reflected_inertia_kgm2(cfg.motor)

    return f"""{HEADER}<mujoco model="palletizer">
  <compiler angle="radian"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"
          solver="Newton" iterations="200" tolerance="1e-14"/>

  <default>
    <geom type="capsule" density="2700" contype="0" conaffinity="0"
          size="{rl:.6f}" rgba="0.62 0.64 0.68 1"/>
    <joint type="hinge" axis="0 -1 0" damping="0.05"/>
    <site type="sphere" size="0.010" rgba="1 0.35 0.1 1"/>
    <default class="rod">
      <geom size="0.008" rgba="0.45 0.55 0.75 1"/>
    </default>
    <default class="level">
      <geom size="0.010" rgba="0.35 0.70 0.45 1"/>
    </default>
  </default>

  <worldbody>
    <geom name="column" type="capsule" fromto="0 0 0  0 0 {h:.6f}" size="{rc:.6f}"
          rgba="0.30 0.30 0.32 1"/>
    <!-- yaw motor: grounded, does not rotate, contributes to no DOF -->
    <geom name="motor_yaw" type="box" size="0.043 0.043 0.058"
          pos="0 0 -0.058" mass="{mm}" rgba="0.20 0.20 0.22 1"/>

    <body name="base" pos="0 0 0">
      <joint name="q0_yaw" type="hinge" axis="0 0 1" damping="0.10"
             armature="{arm_yaw:.6f}"/>
      <geom name="turret" type="capsule" fromto="0 0 0  0 0 {h:.6f}" size="{rc:.6f}"
            rgba="0.40 0.42 0.46 1"/>
      <!-- both planar drive motors ride the rotating base (hard constraint 1) -->
      <geom name="motor_q1" type="box" size="0.043 0.043 0.058"
            pos="{-mo:.6f} 0.050 {h:.6f}" mass="{mm}" rgba="0.20 0.20 0.22 1"/>
      <geom name="motor_q2" type="box" size="0.043 0.043 0.058"
            pos="{-mo:.6f} -0.050 {h:.6f}" mass="{mm}" rgba="0.20 0.20 0.22 1"/>

      <body name="upper_arm" pos="0 0 {h:.6f}">
        <joint name="q1_upper_arm" armature="{arm_q1:.6f}"/>
        <geom name="g_upper_arm" fromto="0 0 0  {l1:.6f} 0 0"/>
        <site name="shoulder" pos="0 0 0"/>

        <body name="forearm" pos="{l1:.6f} 0 0">
          <joint name="q2_forearm_rel"/>
          <geom name="g_forearm" fromto="0 0 0  {l2:.6f} 0 0"/>
          <geom name="g_drive_stub" class="level" fromto="0 0 0  {-a:.6f} 0 0"/>
          <site name="cut_drive" pos="{-a:.6f} 0 0"/>
          <site name="elbow" pos="0 0 0"/>

          <body name="tool_plate" pos="{l2:.6f} 0 0">
            <joint name="tool_plate"/>
            <geom name="g_plate_arm" class="level" fromto="0 0 0  0 0 {b:.6f}"/>
            <geom name="g_gripper" size="0.014" fromto="0 0 0  0 0 {-d:.6f}"
                  rgba="0.85 0.55 0.15 1"/>
            <site name="cut_level2" pos="0 0 {b:.6f}"/>
            <site name="tcp" pos="0 0 {-d:.6f}" size="0.012" rgba="0.15 0.85 0.35 1"/>
          </body>
        </body>

        <body name="elbow_yoke" pos="{l1:.6f} 0 0">
          <joint name="elbow_yoke"/>
          <geom name="g_yoke" class="level" fromto="0 0 0  0 0 {b:.6f}"/>
          <site name="cut_level1" pos="0 0 {b:.6f}"/>

          <body name="rod_b2" pos="0 0 {b:.6f}">
            <joint name="rod_b2"/>
            <geom name="g_rod_b2" class="rod" fromto="0 0 0  {l2:.6f} 0 0"/>
            <site name="cut_level2_rod" pos="{l2:.6f} 0 0"/>
          </body>
        </body>
      </body>

      <body name="drive_lever" pos="0 0 {h:.6f}">
        <joint name="drive_lever" armature="{arm_q2:.6f}"/>
        <geom name="g_drive_lever" class="level" fromto="0 0 0  {-a:.6f} 0 0"/>

        <body name="push_rod" pos="{-a:.6f} 0 0">
          <joint name="push_rod"/>
          <geom name="g_push_rod" class="rod" fromto="0 0 0  {l1:.6f} 0 0"/>
          <site name="cut_drive_rod" pos="{l1:.6f} 0 0"/>
        </body>
      </body>

      <body name="rod_b1" pos="0 0 {h + b:.6f}">
        <joint name="rod_b1"/>
        <geom name="g_rod_b1" class="rod" fromto="0 0 0  {l1:.6f} 0 0"/>
        <site name="cut_level1_rod" pos="{l1:.6f} 0 0"/>
      </body>
    </body>
  </worldbody>

  <equality>
    <connect name="L_drive"  site1="cut_drive_rod"  site2="cut_drive"  {eq}/>
    <connect name="L_level1" site1="cut_level1_rod" site2="cut_level1" {eq}/>
    <connect name="L_level2" site1="cut_level2_rod" site2="cut_level2" {eq}/>
  </equality>

  <actuator>
    <position name="a_yaw"   joint="q0_yaw"       kp="2000" kv="120"/>
    <position name="a_q1"    joint="q1_upper_arm" kp="4000" kv="200"/>
    <position name="a_q2"    joint="drive_lever"  kp="4000" kv="200"/>
  </actuator>
</mujoco>
"""


def actuated_forces(model, data) -> np.ndarray:
    """Generalised forces conjugate to the ABSOLUTE actuated coordinates.

    Order matches `cfg.joints`: (q0_yaw, q1_upper_arm, q2_forearm). Units are
    N.m for all three -- this architecture has no prismatic joint.

    Addressed BY NAME. Indexing `qfrc_inverse[:3]` instead picks up
    q2_forearm_rel, which is the PASSIVE relative elbow hinge, not the actuated
    `drive_lever` that the motor turns. All three bodies named here are children
    of the rotating base, so their joint coordinates are already absolute and no
    transform is needed -- that is the D2 payoff the model header describes.
    """
    import mujoco

    out = []
    for name in ACTUATED:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        out.append(float(data.qfrc_inverse[model.jnt_dofadr[jid]]))
    return np.array(out)


def hinge_values(q0: float, q1a: float, q2a: float) -> dict[str, float]:
    """D2 translation layer: absolute angles -> MJCF hinge values.

    Each body's absolute angle is the running sum of hinge values from the base.
    Inverting that for the required absolute angles of every body:

        upper_arm   q1a         -> q1a - 0
        forearm     q2a         -> q2a - q1a
        tool_plate  0  (level)  -> 0   - q2a
        elbow_yoke  0  (level)  -> 0   - q1a
        rod_b2      q2a         -> q2a - 0        (parent is the level yoke)
        drive_lever q2a         -> q2a - 0
        push_rod    q1a         -> q1a - q2a
        rod_b1      q1a         -> q1a - 0

    Satisfies all three loops exactly, so qpos set this way closes the loops to
    machine precision without invoking the constraint solver at all.
    """
    return {
        "q0_yaw": q0,
        "q1_upper_arm": q1a,
        "q2_forearm_rel": q2a - q1a,
        "tool_plate": -q2a,
        "elbow_yoke": -q1a,
        "rod_b2": q2a,
        "drive_lever": q2a,
        "push_rod": q1a - q2a,
        "rod_b1": q1a,
    }


def qpos_from_absolute(model, q0: float, q1a: float, q2a: float) -> np.ndarray:
    import mujoco

    qpos = np.zeros(model.nq)
    for name, value in hinge_values(q0, q1a, q2a).items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos[model.jnt_qposadr[jid]] = value
    return qpos


def absolute_from_qpos(model, qpos: np.ndarray) -> tuple[float, float, float]:
    """Inverse of `qpos_from_absolute` for the actuated set only."""
    import mujoco

    def get(name: str) -> float:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return float(qpos[model.jnt_qposadr[jid]])

    return get("q0_yaw"), get("q1_upper_arm"), get("drive_lever")

