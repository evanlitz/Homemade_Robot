"""Belt tension budget and the driver-current cap that produces it. D10, F7.

Reports, for the configured torque cap: the current setting per phase, the
standstill dissipation it implies, per-stage belt tension against the
manufacturer allowable, and the maximum joint torque each stage can actually
carry. Also shows what a profile change would buy.

Run: python -m app.belt_check
"""

from __future__ import annotations

from config import load_architecture


def main() -> int:
    cfg = load_architecture("palletizer")
    dt, motor, gate = cfg.drive_train, cfg.motor, cfg.validity
    drive = cfg.joints[1].drive
    n_stages = len(dt.stages)
    stage_eff = drive.efficiency ** (1.0 / n_stages)
    cap = gate.joint_torque_cap_nm

    print(f"motor      {motor.part}")
    print(f"           {motor.holding_torque_nm} N.m at {motor.current_a_per_phase} A/phase, "
          f"{motor.phase_resistance_ohm} ohm/phase")
    print(f"           standstill at rated current: {motor.dissipation_w(motor.current_a_per_phase):.2f} W "
          f"per motor, {3 * motor.dissipation_w(motor.current_a_per_phase):.1f} W for three")
    print(f"drive      1:{drive.reduction:g} over {n_stages} stages, "
          f"total efficiency {drive.efficiency:g} -> {stage_eff:.4f} per stage")

    tau_motor = drive.motor_torque_for_joint_nm(cap)
    current = motor.current_for_torque_a(tau_motor)
    print(f"\nTORQUE CAP (D10): {cap:.1f} N.m at the joint")
    print(f"  motor torque required      {tau_motor:.4f} N.m")
    print(f"  --> DRIVER CURRENT SETTING {current:.3f} A/phase   "
          f"({current / motor.current_a_per_phase * 100:.1f}% of rated)")
    print(f"  standstill dissipation     {motor.dissipation_w(current):.2f} W per motor, "
          f"{3 * motor.dissipation_w(current):.2f} W for three")
    print(f"  (down from {motor.dissipation_w(motor.current_a_per_phase):.1f} W -- capping for the "
          f"belt also removes the standstill heat problem)")

    print(f"\n{'stage':>7} {'profile':>8} {'width':>6} {'TIM':>5} {'derate':>7} "
          f"{'tension N':>10} {'allow N':>8} {'used':>7} {'max joint N.m':>14}")
    worst = None
    torque_at_stage = tau_motor
    for stage in dt.stages:
        tension = dt.stage_tension_n(stage, torque_at_stage)
        allow = dt.allowable_tension_n(stage)
        # Torque the joint could see if this stage ran exactly at its allowable.
        downstream = 1.0
        seen = False
        for s in dt.stages:
            if s is stage:
                seen = True
                continue
            if seen:
                downstream *= s.ratio * stage_eff
        max_joint = allow * dt.pitch_radius_m(stage, stage.driver_teeth) * stage.ratio * stage_eff * downstream
        flag = "" if tension <= allow else "  <-- VIOLATION"
        print(f"{stage.name:>7} {stage.profile:>8} {stage.width_mm:5.0f}mm {stage.teeth_in_mesh():5.2f} "
              f"{stage.mesh_derate():7.1f} {tension:10.1f} {allow:8.1f} "
              f"{tension / allow * 100:6.0f}% {max_joint:14.1f}{flag}")
        if worst is None or max_joint < worst[1]:
            worst = (stage.name, max_joint)
        torque_at_stage *= stage.ratio * stage_eff

    print(f"\nbinding stage: {worst[0]}, limits the joint to {worst[1]:.1f} N.m")
    if worst[1] < cap:
        print(f"GATE FAILS: the {cap:.0f} N.m cap needs {cap / worst[1] * 100 - 100:.0f}% more belt "
              f"capacity than the configured belt has.")
    else:
        print(f"GATE PASSES: {worst[1] / cap * 100 - 100:.0f}% margin over the cap.")

    # NOT "same pulleys". At 5 mm pitch HTD and GT3 share the pitch, and so the
    # pitch diameter and the packaging envelope, but the tooth profiles differ
    # and are NOT interchangeable -- Gates rates 5MGT-on-HTD-5M as not
    # recommended. Same blank, different groove toolpath. F10.
    print("\nwhat a profile change to the binding stage would buy "
          "(same pitch diameter and envelope, DIFFERENT pulley groove):")
    binding = next(s for s in dt.stages if s.name == worst[0])
    idx = dt.stages.index(binding)
    upstream_torque = tau_motor
    for s in dt.stages[:idx]:
        upstream_torque *= s.ratio * stage_eff
    for profile, rating in sorted(dt.rating_n_per_inch.items()):
        if not profile.endswith(binding.profile.split("-")[1]):
            continue
        allow = rating * binding.width_mm / 25.4 * binding.mesh_derate()
        max_joint = allow * dt.pitch_radius_m(binding, binding.driver_teeth) * binding.ratio * stage_eff
        verdict = "supports the cap" if max_joint >= cap else "still short"
        print(f"  {profile:>8} @ {binding.width_mm:.0f} mm: allowable {allow:6.1f} N -> "
              f"joint {max_joint:5.1f} N.m   {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
