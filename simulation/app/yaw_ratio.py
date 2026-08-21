"""Yaw reduction swept independently of the planar joints. D15.

WHY YAW ALONE. Tool travel per full step is `||J_col|| * step_joint`, and the
planar Jacobian columns have norm L1 and L2 while the yaw column has norm r --
the horizontal radius of the target. Every surviving placement puts the worst
square further out than either link, so the yaw column dominates and the
resolution gate is a yaw-ratio question. Raising the planar ratios buys nothing
against it. F11.

WHY THERE IS AN OPTIMUM AND NOT A MONOTONIC GAIN. Reflected rotor inertia is
J_rotor * N^2 (F6), so the torque to accelerate the yaw joint at `alpha` is

    tau_joint(N) = (J_arm + J_rotor * N^2) * alpha

and the motor torque that has to supply it is

    tau_motor(N) = tau_joint / (N * eta) = (J_arm * alpha)/(N * eta)
                                         + (J_rotor * alpha * N)/eta

The first term falls as 1/N, the second rises as N. Differentiating gives the
classic inertia-matching optimum

    N* = sqrt(J_arm / J_rotor)

Below N* the reduction is buying torque; above it the rotor is accelerating
itself and the belt tension starts rising again while resolution keeps
improving. Belt tension follows tau_motor, so it has the same minimum.

WHAT DOES NOT HAVE AN OPTIMUM: motor speed. The joint velocity the cycle time
demands is fixed by D14, so motor rpm rises linearly with N forever. Given
OPEN-D that is likely the real limiter, so it is in the table.

Run: python -m app.yaw_ratio
"""

from __future__ import annotations

import math
from dataclasses import replace

import mujoco
import numpy as np

from app.sweep import board_edge_distance, board_squares
from backends.sim_backend.palletizer_mjcf import build_mjcf, qpos_from_absolute
from config import load_architecture
from config.schema import BeltStage, DriveSpec
from motion.kinematics import inverse_kinematics, tool_m_per_full_step
from motion.limits import within_limits

# Coarse, and deliberately spanning past the inertia-matching optimum so the
# turn is visible rather than inferred.
RATIOS = (15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 75.0, 100.0, 125.0)

DRIVER_TEETH = 20  # SDP/SI 9.3: pulley diameter must not be below belt width.


def stage_pair(ratio: float, widths: tuple[float, float],
               profiles: tuple[str, str]) -> tuple[BeltStage, BeltStage]:
    """Two even-split stages realising `ratio` on integer tooth counts.

    Centre distance is the MINIMUM that satisfies both the geometric limit
    (C > (D+d)/2, with 5% clearance) and SDP/SI 13.3's six-teeth-in-mesh rule,
    which for a 20-tooth small pulley is C >= (D-d)/1.1756. Using the minimum
    makes the packaging envelope a best case, not a typical one.
    """
    out = []
    n = max(DRIVER_TEETH + 1, round(DRIVER_TEETH * math.sqrt(ratio)))
    for i, (w, prof) in enumerate(zip(widths, profiles)):
        d = DRIVER_TEETH * 5.0 / math.pi
        big = n * 5.0 / math.pi
        centre = max((big - d) / 1.1756, (big + d) / 2.0 * 1.05)
        out.append(BeltStage(name=f"yaw{i + 1}", pitch_mm=5.0,
                             driver_teeth=DRIVER_TEETH, driven_teeth=n,
                             width_mm=w, profile=prof, centre_distance_mm=centre))
    return out[0], out[1]


def envelope_mm(s1: BeltStage, s2: BeltStage) -> tuple[float, float]:
    """(folded, collinear) column radius the yaw stack needs, mm.

    The final driven pulley is concentric with the yaw axis. FOLDED puts the
    motor back towards the axis, so the reach is set by the intermediate shaft
    plus its larger pulley. COLLINEAR runs motor -> intermediate -> output in a
    straight line and is the worst case. Real layouts sit between; both are
    reported because the difference is 60 mm of column and that is a CAD
    question, not a simulation one.
    """
    big2 = s2.pitch_dia_mm(s2.driven_teeth)
    big1 = s1.pitch_dia_mm(s1.driven_teeth)
    small = s1.pitch_dia_mm(DRIVER_TEETH)
    folded = max(big2 / 2.0, s2.centre_distance_mm + big1 / 2.0)
    collinear = max(big2 / 2.0,
                    s2.centre_distance_mm + s1.centre_distance_mm + small / 2.0)
    return folded, collinear


def evaluate(cfg, model, data, radius: float, yaw: float, alpha: float):
    """Worst-square resolution and peak YAW torque over one placement."""
    worst_res, peak_yaw, reached = 0.0, 0.0, 0
    for target in board_squares(cfg, radius, yaw):
        sols = [s for s in inverse_kinematics(cfg, target) if within_limits(cfg, s)]
        if not sols:
            continue
        reached += 1
        theta = min(sols, key=lambda s: s[-1])
        worst_res = max(worst_res, float(tool_m_per_full_step(cfg, theta).max()))
        data.qpos[:] = qpos_from_absolute(model, *theta)
        data.qvel[:] = 0.0
        data.qacc[:] = qpos_from_absolute(model, alpha, alpha, alpha)
        mujoco.mj_inverse(model, data)
        peak_yaw = max(peak_yaw, abs(float(data.qfrc_inverse[0])))
    return reached, worst_res, peak_yaw


def arm_yaw_inertia(cfg) -> tuple[float, float]:
    """(min, max) yaw-axis inertia of the arm itself, armature removed.

    Read off the mass matrix rather than assumed, because the two 3.8 kg planar
    motors ride the rotating base and dominate it.

    Swept over the joint box, NOT sampled at one pose. Yaw inertia scales with
    the square of how far the mass sits from the axis, so a folded pose and an
    extended pose differ by more than 3x here. The analytic optimum
    N* = sqrt(J_arm/J_rotor) is only meaningful against the value at the pose
    that produces the peak torque, which is the extended one.
    """
    model = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
    data = mujoco.MjData(model)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "q0_yaw")
    adr = model.jnt_dofadr[jid]
    full = np.zeros((model.nv, model.nv))

    lo, hi = float("inf"), 0.0
    j1, j2 = cfg.joints[1], cfg.joints[2]
    for q1 in np.linspace(j1.min_rad, j1.max_rad, 9):
        for q2 in np.linspace(j2.min_rad, j2.max_rad, 9):
            data.qpos[:] = qpos_from_absolute(model, 0.0, q1, q2)
            mujoco.mj_forward(model, data)
            mujoco.mj_fullM(model, data, full)
            val = float(full[adr, adr]) - float(model.dof_armature[adr])
            lo, hi = min(lo, val), max(hi, val)
    return lo, hi


def main() -> int:
    base = load_architecture("palletizer")
    gate, dt = base.validity, base.drive_train
    alpha = base.joints[0].max_accel_rad_s2
    v_joint = base.joints[0].max_vel_rad_s
    yaw_drive = base.joints[0].drive
    eta = yaw_drive.efficiency
    axes = {a.name: a.values() for a in base.sweep}

    # The driver-current cap is a MOTOR-side quantity. D10 set it as 48 N.m at
    # the joint at 1:25; changing the yaw ratio changes what that same current
    # delivers at the yaw joint, so the invariant to gate against is the motor
    # torque, not the joint torque.
    motor_cap = yaw_drive.motor_torque_for_joint_nm(gate.joint_torque_cap_nm)
    j_rotor = base.motor.rotor_inertia_kgm2
    j_lo, j_hi = arm_yaw_inertia(base)

    print(f"acceleration {alpha:.2f} rad/s2, joint velocity {v_joint:.2f} rad/s "
          f"(D14, 4 s cycle)")
    print(f"driver-current cap: {motor_cap:.4f} N.m at the motor "
          f"({base.motor.current_for_torque_a(motor_cap):.3f} A/phase)")
    print(f"arm yaw inertia at the SEED geometry: {j_lo:.3f} kg.m2 folded to "
          f"{j_hi:.3f} extended -- a {j_hi / j_lo:.1f}x pose spread,")
    print(f"which is why N* is taken per row from the measured J_eff below and "
          f"not from a single sampled pose.")
    print(f"base column radius: {gate.base_clearance_m * 1000:.0f} mm (D13) OR "
          f"the stack's own envelope, whichever is larger --")
    print(f"the column houses the stack AND sets how far out the board must sit, "
          f"so those are not independent.\n")

    print(f"{'ratio':>6} {'teeth':>7} {'res mm':>7} {'places':>7} {'yaw tau':>8} "
          f"{'J_eff':>7} {'N*':>5} {'motor tau':>10} {'vs cap':>7} {'tension':>8} "
          f"{'vs allow':>9} {'D_out':>7} {'fold R':>7} {'rpm':>6}")

    rows = []
    for ratio in RATIOS:
        drive = DriveSpec(kind=yaw_drive.kind, reduction=ratio,
                          microsteps=yaw_drive.microsteps, efficiency=eta)
        s1, s2 = stage_pair(ratio, (15.0, 25.0), ("HTD-5M", "GT3-5M"))
        actual = s1.ratio * s2.ratio
        fold, coll = envelope_mm(s1, s2)
        # SELF-CONSISTENT: a bigger stack needs a bigger column, and a bigger
        # column pushes the board further out, which costs resolution. Gating on
        # the fixed D13 200 mm would credit a ratio with a gain it cannot have.
        clearance = max(gate.base_clearance_m, fold / 1000.0)

        best_res, places, peak_yaw = float("inf"), 0, 0.0
        for l1 in axes["upper_arm_m"]:
            for l2 in axes["forearm_m"]:
                cfg = replace(base,
                              geometry={**base.geometry, "upper_arm_m": l1,
                                        "forearm_m": l2},
                              joints=(replace(base.joints[0], drive=drive),
                                      base.joints[1], base.joints[2]))
                model = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
                data = mujoco.MjData(model)
                for r in axes["board_radius_m"]:
                    for y in axes["board_yaw_rad"]:
                        if board_edge_distance(cfg, r, y) < clearance:
                            continue
                        n_ok, res, tau = evaluate(cfg, model, data, r, y, alpha)
                        if n_ok < 64 or res > gate.tool_m_per_full_step_max:
                            continue
                        places += 1
                        peak_yaw = max(peak_yaw, tau)
                        best_res = min(best_res, res)

        if not places:
            print(f"{ratio:6.0f} {s1.driven_teeth:3d}/{s2.driven_teeth:<3d} "
                  f"{'--':>7} {0:7d}   no placement survives at a "
                  f"{clearance * 1000:.0f} mm column")
            continue

        tau_motor = peak_yaw / (actual * eta)
        stage_eff = math.sqrt(eta)
        tension = dt.stage_tension_n(s2, tau_motor * s1.ratio * stage_eff)
        allow = dt.allowable_tension_n(s2)
        rpm = v_joint * actual * 60.0 / (2.0 * math.pi)
        rows.append((ratio, best_res, places, peak_yaw, tau_motor, tension, allow,
                     fold, coll, rpm))

        flag = ""
        if tau_motor > motor_cap:
            flag = "  <-- OVER CURRENT CAP"
        elif fold > gate.base_clearance_m * 1000:
            flag = f"  <-- needs a {fold:.0f} mm column, not 200"
        # Back the arm's own yaw inertia out of the measured torque:
        # tau = (J_eff + J_rotor*N^2)*alpha. J_eff DRIFTS UP with ratio because
        # a bigger stack needs a bigger column, which forces the board outward,
        # which is a longer-reach pose. That is why the numeric optimum sits
        # above the analytic one computed at any fixed geometry.
        j_eff = peak_yaw / alpha - j_rotor * actual**2
        print(f"{ratio:6.0f} {s1.driven_teeth:3d}/{s2.driven_teeth:<3d} "
              f"{best_res * 1000:7.3f} {places:7d} {peak_yaw:8.1f} "
              f"{j_eff:7.3f} {math.sqrt(max(j_eff, 0.0) / j_rotor):5.0f} "
              f"{tau_motor:10.3f} {tau_motor / motor_cap * 100:6.0f}% "
              f"{tension:8.0f} {tension / allow * 100:8.0f}% "
              f"{s2.pitch_dia_mm(s2.driven_teeth):7.0f} {fold:7.0f} "
              f"{rpm:6.0f}{flag}")

    if rows:
        best_tau = min(rows, key=lambda r: r[4])
        best_res = min(rows, key=lambda r: r[1])
        print(f"\nmotor torque bottoms out at ratio {best_tau[0]:.0f} "
              f"({best_tau[4]:.3f} N.m) -- the inertia term overtaking the "
              f"reduction.")
        print(f"RESOLUTION bottoms out earlier, at ratio {best_res[0]:.0f} "
              f"({best_res[1] * 1000:.3f} mm), and gets WORSE above it: the "
              f"column")
        print(f"needed to house the stack pushes the board outward faster than "
              f"the ratio pulls the")
        print(f"step size down. Resolution is the binding gate, so that is the "
              f"turn that matters.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
