"""Specify and score SCARA's Z axis. D19 item 3.

The 800 N in F15 was an artefact: the shared 1:25 belt reduction copied onto a
100 mm lead gives 666 kg of reflected rotor mass. No one would build that. This
picks a real screw and scores it, because SCARA's one genuine weakness cannot be
argued around a placeholder.

THE QUANTITY THAT MATTERS IS MOTOR TORQUE, NOT JOINT FORCE. A screw with a fine
lead shows a huge generalised force at the joint and needs almost no torque to
produce it -- `tau = F * lead / (2*pi * eta)`. Quoting newtons beside the
revolute joints' newton-metres invites exactly the wrong conclusion, which is
what the 800 N figure did.

WHAT SIZES THE LEAD IS SPEED, NOT RESOLUTION. Every candidate lead below is far
inside the 1 mm/full-step budget. What separates them is the motor rpm needed to
lift the carriage fast enough for the D14 cycle, and OPEN-D says rpm is the
thing this project cannot cash cheques against.

Run: python -m app.z_drive
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from backends.sim_backend.scara_mjcf import build_mjcf, qpos_from_absolute
from config import load_architecture

# Stock ball-screw leads, 16-25 mm nominal diameter. All are catalogue parts.
BALL_SCREW_LEADS_MM = (5.0, 10.0, 16.0, 20.0, 25.0, 32.0)
BALL_SCREW_EFF = 0.90

# A self-locking alternative, for the OPEN-SAFETY-1 comparison. Trapezoidal
# thread self-locks while the lead angle stays under the friction angle, which
# for a 16 mm screw in steel/bronze means a lead of roughly 5 mm or less.
ACME_LEAD_MM = 4.0
ACME_EFF = 0.30

# The vertical move a chess piece needs: grip height to travel height. From the
# F12 move model, paid FOUR times per move (down, up, down, up).
HOP_M = 0.100


def carriage_mass_kg(cfg) -> float:
    """Everything the Z axis lifts, read off the model rather than assumed.

    Static hold force divided by g. The carriage carries both revolute motors,
    both links and the gripper, so this is the number that actually matters and
    it is not a figure anyone should be inventing.
    """
    model = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
    data = mujoco.MjData(model)
    data.qpos[:] = qpos_from_absolute(model, 0.0, 0.0, 0.1)
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_inverse(model, data)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "q2_lift")
    return float(data.qfrc_inverse[model.jnt_dofadr[jid]]) / 9.81


def score(lead_m: float, eff: float, mass_kg: float, payload_kg: float,
          j_rotor: float, hop_time_s: float, motor):
    """One candidate screw, against a hop of HOP_M completed in `hop_time_s`."""
    total_kg = mass_kg + payload_kg
    # Triangular profile, rest to rest: a = 4d/T^2, peak v = a*T/2.
    accel = 4.0 * HOP_M / hop_time_s**2
    v_peak = accel * hop_time_s / 2.0
    reflected_kg = j_rotor * (2.0 * math.pi / lead_m) ** 2
    force_n = (total_kg + reflected_kg) * accel + total_kg * 9.81
    torque_nm = force_n * lead_m / (2.0 * math.pi * eff)
    rpm = v_peak / lead_m * 60.0
    res_mm = lead_m / motor.full_steps_per_rev * 1000.0
    # Static hold with power on, and whether it holds with power off.
    hold_nm = total_kg * 9.81 * lead_m / (2.0 * math.pi * eff)
    return dict(lead_mm=lead_m * 1000, reflected_kg=reflected_kg, force_n=force_n,
                torque_nm=torque_nm, rpm=rpm, res_mm=res_mm, hold_nm=hold_nm,
                accel=accel, v_peak=v_peak)


def main() -> int:
    cfg = load_architecture("scara")
    motor = cfg.motor
    mass = carriage_mass_kg(cfg)
    payload = 0.035
    j_rotor = motor.rotor_inertia_kgm2

    # Four vertical hops per move. D14 gives 3.5 s of motion per move; the
    # traverse needs a share of it, so give the four hops half the budget.
    hop_time = 3.5 / 2.0 / 4.0

    print(f"carriage mass, measured from the model: {mass:.2f} kg "
          f"(2 revolute motors + links + gripper)")
    print(f"payload: {payload * 1000:.0f} g -- 0.3% of the carriage, and it is "
          f"NOT what sizes this axis")
    print(f"hop {HOP_M * 1000:.0f} mm in {hop_time:.3f} s, four per move "
          f"(half of D14's 3.5 s motion budget)")
    print(f"  -> {4.0 * HOP_M / hop_time**2:.2f} m/s2, peak "
          f"{4.0 * HOP_M / hop_time**2 * hop_time / 2:.2f} m/s\n")

    print(f"BALL SCREW, efficiency {BALL_SCREW_EFF:g}, backdrivable")
    print(f"{'lead':>6} {'refl kg':>8} {'joint N':>9} {'motor N.m':>10} "
          f"{'% of 8.4':>9} {'rpm':>7} {'mm/step':>8} {'hold N.m':>9}")
    rows = []
    for lead in BALL_SCREW_LEADS_MM:
        r = score(lead / 1000.0, BALL_SCREW_EFF, mass, payload, j_rotor,
                  hop_time, motor)
        rows.append(r)
        flag = ""
        if r["rpm"] > 1000:
            flag = "  <-- rpm, OPEN-D"
        elif r["res_mm"] > 1.0:
            flag = "  <-- resolution"
        print(f"{r['lead_mm']:5.0f}mm {r['reflected_kg']:8.1f} {r['force_n']:9.0f} "
              f"{r['torque_nm']:10.3f} {r['torque_nm'] / 8.4 * 100:8.1f}% "
              f"{r['rpm']:7.0f} {r['res_mm']:8.3f} {r['hold_nm']:9.3f}{flag}")

    a = score(ACME_LEAD_MM / 1000.0, ACME_EFF, mass, payload, j_rotor, hop_time,
              motor)
    print(f"\nACME / TRAPEZOIDAL, efficiency {ACME_EFF:g}, SELF-LOCKING")
    print(f"{a['lead_mm']:5.0f}mm {a['reflected_kg']:8.1f} {a['force_n']:9.0f} "
          f"{a['torque_nm']:10.3f} {a['torque_nm'] / 8.4 * 100:8.1f}% "
          f"{a['rpm']:7.0f} {a['res_mm']:8.3f} {a['hold_nm']:9.3f}"
          f"  <-- rpm impossible")
    print("Self-locking needs the lead angle under the friction angle, which on")
    print("a 16 mm screw caps the lead near 5 mm. At 5 mm the axis cannot move")
    print(f"{HOP_M * 1000:.0f} mm in {hop_time:.2f} s at any achievable motor speed.")

    ok = [r for r in rows if r["rpm"] <= 1000 and r["res_mm"] <= 1.0]
    print(f"\nBest inertia match (reflected ~= carriage, {mass + payload:.1f} kg):")
    match = min(rows, key=lambda r: abs(math.log(r["reflected_kg"] / (mass + payload))))
    print(f"  {match['lead_mm']:.0f} mm lead -> {match['reflected_kg']:.1f} kg "
          f"reflected against {mass + payload:.1f} kg real")
    if ok:
        pick = min(ok, key=lambda r: r["torque_nm"])
        print(f"\nLowest motor torque among leads that clear both rpm and "
              f"resolution: {pick['lead_mm']:.0f} mm")
        print(f"  {pick['torque_nm']:.3f} N.m peak "
              f"({pick['torque_nm'] / 8.4 * 100:.1f}% of holding), "
              f"{pick['rpm']:.0f} rpm, {pick['res_mm']:.3f} mm/full step")
    print("\nSCARA'S GENUINE WEAKNESS, stated plainly: no self-locking screw is")
    print("fast enough, so the Z axis is backdrivable and the whole arm drops on")
    print("a power cut. It needs a fail-safe brake. That is OPEN-SAFETY-1 again,")
    print("on a different axis, and it does not go away with a better lead.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
