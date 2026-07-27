"""Print a loaded architecture config and every placeholder it still contains.

Run: python -m app.show_config [name]
"""

from __future__ import annotations

import math
import sys

from config import available_architectures, load_architecture


def main(name: str = "placeholder_arm") -> int:
    cfg = load_architecture(name)

    print(f"architecture: {cfg.name}  (candidate {cfg.candidate_id}, closed_loop={cfg.closed_loop})")
    print(f"source: {cfg.source}")
    print(f"motor: {cfg.motor.part}  {cfg.motor.holding_torque_nm} N.m  "
          f"{cfg.motor.full_steps_per_rev:.0f} full steps/rev  rear_shaft={cfg.motor.rear_shaft}")
    print()

    print("links:")
    for link in cfg.links:
        print(f"  {link.name:<10} {link.length_m * 1000:7.1f} mm  {link.mass_kg:5.2f} kg")
    print()

    print("joints:")
    for j in cfg.joints:
        step_deg = math.degrees(j.drive.joint_rad_per_full_step(cfg.motor))
        torque = j.drive.joint_torque_limit_nm(cfg.motor)
        print(f"  {j.name:<14} range [{math.degrees(j.min_rad):7.1f}, {math.degrees(j.max_rad):7.1f}] deg"
              f"  1:{j.drive.reduction:g} {j.drive.kind}"
              f"  {step_deg:.4f} deg/full-step  {torque:.1f} N.m")
    print()

    placeholders = cfg.placeholder_fields()
    print(f"PLACEHOLDERS: {len(placeholders)} values in this config are invented, not chosen.")
    for p in placeholders:
        print(f"  {p}")
    print()
    print(f"available: {available_architectures()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(*sys.argv[1:]))
