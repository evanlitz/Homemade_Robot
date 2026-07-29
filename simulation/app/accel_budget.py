"""What acceleration limit a given chess-move cycle time actually implies.

`max_accel_rad_s2` was a placeholder at 4.0. Peak joint torque is affine in it,
so it is load-bearing: a guess there is a guess at the torque that sizes the
belts, the pulleys and the driver current. This derives it from a cycle-time
requirement instead, which is a spec that can be chosen rather than invented.

MOVE MODEL. One chess move, rest to rest, five motion segments and two dwells:

    descend over source -> [close] -> ascend -> traverse -> descend over
    destination -> [open] -> ascend

Every segment is a coordinated joint-space move at a common acceleration limit
`a`, triangular velocity profile (accelerate to the midpoint, decelerate to the
end). For one segment whose limiting joint travels d radians, T = 2*sqrt(d/a).
Summing over segments and solving:

    a = ( 2 * SUM_segments sqrt(d_seg) / T_motion )^2        T_motion = T - 2*dwell

A triangular profile is the fastest rest-to-rest move under an acceleration
limit alone, so this is the LOWEST acceleration that meets the cycle time. Any
real profile -- trapezoidal, S-curve -- needs more. The figures below are a
floor, not an estimate.

Run: python -m app.accel_budget
"""

from __future__ import annotations

import math
import statistics
from dataclasses import replace

import mujoco
import numpy as np

from app.sweep import board_edge_distance, board_squares
from backends.sim_backend.palletizer_mjcf import build_mjcf, qpos_from_absolute
from config import load_architecture
from motion.kinematics import inverse_kinematics, tool_m_per_full_step
from motion.limits import within_limits

CYCLE_TIMES_S = (2.0, 4.0, 8.0)



def _solve(cfg, target: np.ndarray):
    sols = [s for s in inverse_kinematics(cfg, target) if within_limits(cfg, s)]
    return min(sols, key=lambda s: s[-1]) if sols else None


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _span(a: np.ndarray, b: np.ndarray) -> float:
    """Limiting-joint displacement between two joint vectors, radians.

    Yaw is wrapped: the arm takes the short way round, so a 179-degree swing
    and a 181-degree swing are both under half a turn.
    """
    d = np.abs(b - a)
    d[0] = abs(_wrap(float(b[0] - a[0])))
    return float(d.max())


def move_costs(cfg, radius: float, yaw: float) -> list[tuple[float, float]] | None:
    """Per ordered source/destination pair: (SUM sqrt(d_seg), longest d_seg).

    The sum sets the acceleration needed to hit a cycle time. The longest single
    segment sets the peak velocity that acceleration produces, because velocity
    peaks inside a segment and returns to zero at its end -- so it is the
    biggest segment that matters, not the biggest move.

    Returns None if any square, at either height, is out of reach: a geometry
    that cannot do every move has no cycle time to quote.
    """
    squares = board_squares(cfg, radius, yaw)
    z_grip = cfg.gripper["grip_height_m"]
    z_travel = cfg.gripper["travel_height_m"]

    low, high = [], []
    for sq in squares:
        a = _solve(cfg, np.array([sq[0], sq[1], z_grip]))
        b = _solve(cfg, np.array([sq[0], sq[1], z_travel]))
        if a is None or b is None:
            return None
        low.append(a)
        high.append(b)

    # Vertical hop at each square, paid twice per visit (down then up).
    hop = [_span(high[i], low[i]) for i in range(64)]
    out = []
    for i in range(64):
        for j in range(64):
            if i == j:
                continue
            traverse = _span(high[i], high[j])
            total = 2.0 * math.sqrt(hop[i]) + math.sqrt(traverse) + 2.0 * math.sqrt(hop[j])
            out.append((total, max(hop[i], hop[j], traverse)))
    return out


def peak_torque_nm(cfg, model, data, radius: float, yaw: float, alpha: float) -> float:
    worst = 0.0
    for sq in board_squares(cfg, radius, yaw):
        theta = _solve(cfg, sq)
        if theta is None:
            continue
        data.qpos[:] = qpos_from_absolute(model, *theta)
        data.qvel[:] = 0.0
        data.qacc[:] = qpos_from_absolute(model, alpha, alpha, alpha)
        mujoco.mj_inverse(model, data)
        worst = max(worst, float(np.abs(data.qfrc_inverse[:3]).max()))
    return worst


def _table(candidates, dwell: float, label: str) -> dict[float, float]:
    """Acceleration implied by each cycle time, over one candidate set."""
    print(f"\nACCELERATION IMPLIED BY THE CYCLE TIME -- {label}")
    print(f"  {len(candidates)} geometries")
    print(f"{'cycle T':>8} {'dwell':>6} {'motion':>7} | {'alpha worst move':>17} "
          f"{'alpha median move':>18} | {'peak joint vel':>15}")
    print(f"{'s':>8} {'s':>6} {'s':>7} | {'rad/s2':>17} {'rad/s2':>18} | "
          f"{'rad/s':>15}")

    picked: dict[float, float] = {}
    for total in CYCLE_TIMES_S:
        motion = total - 2.0 * dwell
        if motion <= 0:
            print(f"{total:8.1f} {2 * dwell:6.2f} {motion:7.2f} |  "
                  f"dwell alone exceeds the cycle time")
            continue
        # Worst move over every pair, per geometry; then the worst over
        # geometries. Worst-move is the honest reading if EVERY legal move must
        # fit the cycle time -- a rook crossing the board is a legal move.
        a_worst = max((2.0 * max(s for s, _ in c) / motion) ** 2
                      for _, _, _, c in candidates)
        a_med = max((2.0 * statistics.median([s for s, _ in c]) / motion) ** 2
                    for _, _, _, c in candidates)
        # Peak velocity: triangular profile on the longest single segment,
        # v = sqrt(a * d). Velocity returns to zero at each segment end, so the
        # biggest SEGMENT sets this, not the biggest move.
        longest_seg = max(max(d for _, d in c) for _, _, _, c in candidates)
        print(f"{total:8.1f} {2 * dwell:6.2f} {motion:7.2f} | {a_worst:17.2f} "
              f"{a_med:18.2f} | {math.sqrt(a_worst * longest_seg):15.2f}")
        picked[total] = a_worst
    return picked


def main() -> int:
    cfg0 = load_architecture("palletizer")
    gate = cfg0.validity
    dwell = cfg0.gripper["actuation_s"]
    axes = {a.name: a.values() for a in cfg0.sweep}

    print(f"move model: descend, [close {dwell:.2f} s], ascend, traverse, "
          f"descend, [open {dwell:.2f} s], ascend")
    print(f"            grip height {cfg0.gripper['grip_height_m'] * 1000:.0f} mm, "
          f"travel height {cfg0.gripper['travel_height_m'] * 1000:.0f} mm "
          f"(both PLACEHOLDER)")
    print(f"            triangular profile -- the acceleration floor, not an estimate")

    # Geometries that clear the kinematic gates: all 64 squares reachable at both
    # heights, worst-square resolution inside budget. Torque is not a filter here
    # -- torque is the OUTPUT of this analysis, so gating on it would be circular.
    candidates = []
    for l1 in axes["upper_arm_m"]:
        for l2 in axes["forearm_m"]:
            cfg = replace(cfg0, geometry={**cfg0.geometry,
                                          "upper_arm_m": l1, "forearm_m": l2})
            for radius in axes["board_radius_m"]:
                for yaw in axes["board_yaw_rad"]:
                    costs = move_costs(cfg, radius, yaw)
                    if costs is None:
                        continue
                    squares = board_squares(cfg, radius, yaw)
                    res = max(float(tool_m_per_full_step(cfg, _solve(cfg, s)).max())
                              for s in squares)
                    if res > gate.tool_m_per_full_step_max:
                        continue
                    candidates.append((cfg, radius, yaw, costs))

    if not candidates:
        print("\nno geometry reaches every square at both heights")
        return 1

    print(f"\n{len(candidates)} geometries reach every square at both heights and "
          f"pass the resolution gate.")

    # Geometries with the board sitting on top of the base column have the
    # largest yaw swings and would otherwise set the acceleration spec. D13
    # gates them out; both sets are shown so the difference stays visible.
    column = gate.base_clearance_m
    buildable = [c for c in candidates
                 if board_edge_distance(c[0], c[1], c[2]) >= column]
    picked = _table(candidates, dwell, "ALL geometries passing the kinematic gates")
    chosen = candidates
    if buildable:
        picked = _table(buildable, dwell,
                        f"only those clearing the D13 {column * 1000:.0f} mm base column")
        chosen = buildable

    v_limit = cfg0.joints[0].max_vel_rad_s
    print(f"\njoint velocity limit in config: {v_limit:.2f} rad/s (PLACEHOLDER). "
          f"Where peak exceeds it the")
    print(f"profile clips to a trapezoid and the real acceleration is HIGHER "
          f"than shown.")
    print(f"1 rad/s at the joint is {cfg0.joints[0].drive.reduction * 60 / (2 * math.pi):.0f} "
          f"motor rpm, so the peak column is also a motor-speed check.")

    print(f"\nWHAT EACH CHOICE COSTS IN TORQUE -- over the {len(chosen)} "
          f"geometries in the last table")
    print(f"{'cycle T':>8} {'alpha':>9} {'peak torque':>12} {'vs cap':>8} "
          f"{'stage2 tension':>15} {'vs allow':>9}")
    dt = cfg0.drive_train
    drive = cfg0.joints[1].drive
    stage_eff = drive.efficiency ** (1.0 / len(dt.stages))
    stage2 = dt.stages[-1]
    allow2 = dt.allowable_tension_n(stage2)
    for total, alpha in picked.items():
        peak = 0.0
        for cfg, radius, yaw, _ in chosen:
            model = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
            data = mujoco.MjData(model)
            peak = max(peak, peak_torque_nm(cfg, model, data, radius, yaw, alpha))
        torque2 = drive.motor_torque_for_joint_nm(peak) * stage2.ratio * stage_eff
        tension2 = dt.stage_tension_n(stage2, torque2)
        print(f"{total:8.1f} {alpha:9.2f} {peak:12.1f} "
              f"{peak / gate.joint_torque_cap_nm * 100:7.0f}% {tension2:15.0f} "
              f"{tension2 / allow2 * 100:8.0f}%")

    print(f"\ncap is {gate.joint_torque_cap_nm:.0f} N.m, stage2 allowable is "
          f"{allow2:.0f} N ({stage2.profile}, {stage2.width_mm:.0f} mm).")
    print("Pick a cycle time; the acceleration and the torque follow from it.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
