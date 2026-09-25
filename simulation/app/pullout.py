"""OPEN-D: what each motor must deliver at speed, and whether a measured
pull-out curve delivers it.

Two modes.

    python -m app.pullout scara
        The BENCH TARGET. For every actuated joint, the motor-shaft torque it
        needs and the motor speed it needs it at, worst case over every
        geometry that survives the sweep's gates. This is what the pull-out
        test has to beat, stated before the test so it cannot be fitted to it.

    python -m app.pullout scara bench/pullout_34HS1456_60V.csv [--margin 2.0]
        The VERDICT. Scores a measured pull-out curve (see BENCH.md for how to
        take it and bench/pullout_TEMPLATE.csv for the format) against those
        operating points.

THE CHECK IS DELIBERATELY PESSIMISTIC: peak torque demanded at peak speed.
Those are different instants -- on a triangular profile the torque peak is at
segment start and the speed peak mid-segment (F12) -- so a PASS is conclusive
and a FAIL is not. A FAIL means "do the profile-level check", not "the motor
cannot do it".

The margin is the ratio pull-out / demand that counts as a pass. Stepper
practice runs at a half to two-thirds of pull-out, because a stepper that
reaches pull-out does not slow down, it loses sync and drops the load; 2.0 is
the conservative end. It is a parameter, not a decision -- see OPEN-D.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from app.sweep import evaluate_all
from config import load_architecture

DEFAULT_MARGIN = 2.0


@dataclass(frozen=True)
class OperatingPoint:
    joint: str
    linear: bool
    joint_demand: float   # N.m, or N for a linear joint
    motor_torque_nm: float
    motor_rpm: float


def motor_rpm(joint) -> float:
    """Motor speed at the joint's configured velocity limit.

    The limit is D14's derived requirement (F12), so this is the speed the
    cycle time asks for, not a capability. Linear joints carry m/s in the
    rad/s field (see scara.yaml); one lead of travel is one output revolution.
    """
    d = joint.drive
    if d.is_linear:
        return joint.max_vel_rad_s / d.lead_m_per_rev * d.reduction * 60.0
    return joint.max_vel_rad_s * d.reduction * 60.0 / (2.0 * math.pi)


def operating_points(name: str) -> tuple[list[OperatingPoint], int]:
    """Worst case over the sweep's survivors, per actuated joint.

    The sweep reports one peak over all revolute joints and one over all
    prismatic ones, so every revolute joint is scored against the worst
    revolute demand. Conservative by construction; per-joint peaks would
    only lower some of the rows.
    """
    cfg = load_architecture(name)
    survivors = [r for r in evaluate_all(cfg) if r.ok]
    if not survivors:
        raise SystemExit(f"{name}: no geometry survives the gates, nothing to score")
    peak_tau = max(r.peak_torque_nm for r in survivors)
    peak_force = max(r.peak_force_n for r in survivors)

    points = []
    for j in cfg.joints:
        demand = peak_force if j.drive.is_linear else peak_tau
        points.append(OperatingPoint(
            joint=j.name, linear=j.drive.is_linear, joint_demand=demand,
            motor_torque_nm=j.drive.motor_torque_for_joint_nm(demand),
            motor_rpm=motor_rpm(j)))
    return points, len(survivors)


def load_curve(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """rpm, torque_nm columns. '#' lines are comments (and should say what bus
    voltage, driver, current and microstep setting the curve was taken at)."""
    rpm, tq = [], []
    with open(path, newline="") as f:
        rows = csv.DictReader(line for line in f if not line.lstrip().startswith("#"))
        for row in rows:
            rpm.append(float(row["rpm"]))
            tq.append(float(row["torque_nm"]))
    if len(rpm) < 3:
        raise ValueError(f"{path}: a pull-out curve needs at least 3 points, "
                         f"got {len(rpm)}")
    order = np.argsort(rpm)
    rpm, tq = np.asarray(rpm)[order], np.asarray(tq)[order]
    if np.any(np.diff(rpm) == 0):
        # repeat runs belong in the file; keep the WORST at each speed
        u = np.unique(rpm)
        tq = np.array([tq[rpm == s].min() for s in u])
        rpm = u
    return rpm, tq


def available_nm(curve, rpm: float) -> float | None:
    """Pull-out torque at `rpm`, linearly interpolated. None if the curve was
    not measured that fast: extrapolating a stepper's torque curve is exactly
    the guess OPEN-D exists to stop making."""
    speeds, torque = curve
    if rpm > speeds[-1] or rpm < speeds[0]:
        return None
    return float(np.interp(rpm, speeds, torque))


def verdict(points, curve, margin: float = DEFAULT_MARGIN) -> list[dict]:
    rows = []
    for p in points:
        avail = available_nm(curve, p.motor_rpm)
        if avail is None:
            status, ratio = "NOT MEASURED", None
        else:
            ratio = avail / p.motor_torque_nm if p.motor_torque_nm > 0 else math.inf
            status = "PASS" if ratio >= margin else "FAIL"
        rows.append(dict(point=p, available=avail, ratio=ratio, status=status))
    return rows


def _print_points(name, points, n):
    print(f"{name}: worst case over {n} surviving geometries, peak torque at "
          f"peak speed\n")
    print(f"{'joint':<14}{'joint demand':>16}{'motor N.m':>12}{'motor rpm':>11}")
    for p in points:
        unit = "N" if p.linear else "N.m"
        print(f"{p.joint:<14}{p.joint_demand:>12.2f} {unit:<3}"
              f"{p.motor_torque_nm:>12.3f}{p.motor_rpm:>11.0f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="OPEN-D bench target and verdict")
    ap.add_argument("architecture", nargs="?", default="scara")
    ap.add_argument("curve", nargs="?", help="measured pull-out CSV")
    ap.add_argument("--margin", type=float, default=DEFAULT_MARGIN,
                    help="pull-out / demand needed to pass (default %(default)s)")
    args = ap.parse_args(argv)

    points, n = operating_points(args.architecture)
    _print_points(args.architecture, points, n)

    if args.curve is None:
        print(f"\nBENCH TARGET at margin {args.margin:g}: the pull-out curve must "
              f"clear each of these, measured at the operating bus voltage and "
              f"driver current.")
        for p in points:
            print(f"  >= {p.motor_torque_nm * args.margin:6.3f} N.m at "
                  f"{p.motor_rpm:5.0f} rpm   ({p.joint})")
        print("\nMeasure past the highest speed above -- the verdict refuses to "
              "extrapolate.")
        return 0

    curve = load_curve(args.curve)
    print(f"\ncurve: {args.curve}, {len(curve[0])} speeds, "
          f"{curve[0][0]:.0f}-{curve[0][-1]:.0f} rpm, margin {args.margin:g}\n")
    print(f"{'joint':<14}{'needs N.m':>10}{'@ rpm':>7}{'has N.m':>9}{'ratio':>7}  verdict")
    rows = verdict(points, curve, args.margin)
    for r in rows:
        p = r["point"]
        has = "-" if r["available"] is None else f"{r['available']:.3f}"
        ratio = "-" if r["ratio"] is None else f"{r['ratio']:.1f}x"
        print(f"{p.joint:<14}{p.motor_torque_nm:>10.3f}{p.motor_rpm:>7.0f}"
              f"{has:>9}{ratio:>7}  {r['status']}")

    if all(r["status"] == "PASS" for r in rows):
        print("\nOPEN-D closes for this architecture: every joint clears the "
              "margin even with peak torque and peak speed taken together.")
        return 0
    print("\nOPEN-D stays open. A FAIL here is not conclusive -- peak torque "
          "and peak speed are different instants (F12) -- so the next step is a "
          "profile-level check on the failing joint, not a new motor.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
