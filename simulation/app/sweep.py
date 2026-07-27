"""D8/D9 joint sweep over link lengths and board placement, with HARD gates.

D11: geometries violating a gate are REJECTED, not ranked. Coverage rewards
exactly the direction that breaks the tension and resolution budgets -- longer
links, board pushed outward -- so a sweep that reports violations post hoc
hands back an optimum that is invalid.

Coarse by design (D9). The deliverable is the shape of the surface and the
trade curve, not a fine grid.

Run: python -m app.sweep
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import mujoco
import numpy as np

from backends.sim_backend.palletizer_mjcf import build_mjcf, qpos_from_absolute
from config import load_architecture
from motion.kinematics import forward_kinematics, inverse_kinematics, tool_m_per_full_step
from motion.limits import within_limits

FILES = "abcdefgh"


@dataclass
class Result:
    l1: float
    l2: float
    radius: float
    yaw: float
    coverage: float
    worst_radius_m: float
    worst_resolution_m: float
    peak_torque_nm: float
    worst_tension_n: float
    allowable_tension_n: float
    min_clearance_m: float
    rejections: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.rejections


def board_squares(cfg, radius: float, yaw: float) -> list[np.ndarray]:
    """64 square centres in the world frame. Board centred at (radius, 0),
    grid rotated by `yaw` about its own centre."""
    sq = cfg.task.board_square_m
    z = cfg.gripper["grip_height_m"]
    out = []
    for i in range(8):
        for j in range(8):
            u = (i - 3.5) * sq
            v = (j - 3.5) * sq
            out.append(np.array([radius + u * math.cos(yaw) - v * math.sin(yaw),
                                 u * math.sin(yaw) + v * math.cos(yaw), z]))
    return out


def gripper_clearance(cfg, target: np.ndarray, neighbours: list[np.ndarray]) -> float:
    """Least gap between the gripper footprint and any neighbouring piece.

    D3-REQ. Yaw is pinned to atan2(y, x), so the gripper is always oriented
    RADIALLY -- its footprint rotates relative to the rank/file grid as a
    function of where on the board the target sits. Negative means collision.

    Footprint is a rectangle: jaw span across the jaw axis (taken perpendicular
    to radial, so the body extends radially), body width along radial.
    """
    half_jaw = (cfg.gripper["jaw_span_open_m"] + 2 * cfg.gripper["jaw_thickness_m"]) / 2.0
    half_body = cfg.gripper["body_width_m"] / 2.0
    piece_r = cfg.gripper["piece_diameter_m"] / 2.0
    g = math.atan2(target[1], target[0])
    c, s = math.cos(g), math.sin(g)

    worst = float("inf")
    for n in neighbours:
        d = n[:2] - target[:2]
        # into the gripper frame: +x radial, +y tangential
        local = np.array([d[0] * c + d[1] * s, -d[0] * s + d[1] * c])
        clamped = np.array([np.clip(local[0], -half_body, half_body),
                            np.clip(local[1], -half_jaw, half_jaw)])
        worst = min(worst, float(np.linalg.norm(local - clamped)) - piece_r)
    return worst


def evaluate(cfg, model, data, radius: float, yaw: float) -> Result:
    gate, dt = cfg.validity, cfg.drive_train
    drive = cfg.joints[1].drive
    stage_eff = drive.efficiency ** (1.0 / len(dt.stages))
    squares = board_squares(cfg, radius, yaw)

    reached = 0
    worst_r = 0.0
    worst_res = 0.0
    peak_tau = 0.0
    min_clear = float("inf")

    for idx, target in enumerate(squares):
        sols = [s for s in inverse_kinematics(cfg, target) if within_limits(cfg, s)]
        if not sols:
            continue
        reached += 1
        theta = min(sols, key=lambda s: s[-1])  # elbow-down
        worst_r = max(worst_r, float(math.hypot(target[0], target[1])))
        worst_res = max(worst_res, float(tool_m_per_full_step(cfg, theta).max()))

        data.qpos[:] = qpos_from_absolute(model, *theta)
        data.qvel[:] = 0.0
        # Gravity hold PLUS the inertial term at the configured acceleration
        # limit. Gravity alone understates the peak: reflected rotor inertia is
        # 0.169 kg.m^2 per joint (F6), comparable to the arm itself, so
        # accelerating costs real torque. qacc is built with the same linear
        # absolute->hinge map as qpos, which keeps it on the constraint
        # manifold -- an arbitrary qacc would fight the loop constraints and
        # produce nonsense.
        alpha = [j.max_accel_rad_s2 for j in cfg.joints]
        data.qacc[:] = qpos_from_absolute(model, *alpha)
        mujoco.mj_inverse(model, data)
        peak_tau = max(peak_tau, float(np.abs(data.qfrc_inverse[:3]).max()))

        neighbours = [squares[k] for k in _neighbours(idx)]
        min_clear = min(min_clear, gripper_clearance(cfg, target, neighbours))

    coverage = reached / len(squares)

    # Belt tension at the peak torque this geometry actually demands.
    torque = drive.motor_torque_for_joint_nm(peak_tau)
    worst_tension, allowable, worst_ratio = 0.0, float("inf"), -1.0
    for stage in dt.stages:
        tension = dt.stage_tension_n(stage, torque)
        allow = dt.allowable_tension_n(stage)
        if tension / allow > worst_ratio:
            worst_ratio, worst_tension, allowable = tension / allow, tension, allow
        torque *= stage.ratio * stage_eff

    rej = gate.rejections(peak_torque_nm=peak_tau, worst_resolution_m=worst_res,
                          worst_tension_n=worst_tension, allowable_tension_n=allowable)
    if coverage < 1.0:
        rej.append(f"coverage {coverage * 100:.0f}% < 100%")

    return Result(cfg.g("upper_arm_m"), cfg.g("forearm_m"), radius, yaw, coverage,
                  worst_r, worst_res, peak_tau, worst_tension, allowable,
                  0.0 if min_clear == float("inf") else min_clear, tuple(rej))


def _neighbours(idx: int) -> list[int]:
    i, j = divmod(idx, 8)
    return [a * 8 + b for a in (i - 1, i, i + 1) for b in (j - 1, j, j + 1)
            if 0 <= a < 8 and 0 <= b < 8 and (a, b) != (i, j)]


def main() -> int:
    base = load_architecture("palletizer")
    axes = {a.name: a.values() for a in base.sweep}
    results: list[Result] = []

    print(f"sweep: {len(axes['upper_arm_m'])} x {len(axes['forearm_m'])} link combos "
          f"x {len(axes['board_radius_m'])} radii x {len(axes['board_yaw_rad'])} yaws "
          f"= {np.prod([len(v) for v in axes.values()]):.0f} evaluations\n")

    for l1 in axes["upper_arm_m"]:
        for l2 in axes["forearm_m"]:
            cfg = replace(base, geometry={**base.geometry,
                                          "upper_arm_m": l1, "forearm_m": l2})
            model = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
            data = mujoco.MjData(model)
            for radius in axes["board_radius_m"]:
                for yaw in axes["board_yaw_rad"]:
                    results.append(evaluate(cfg, model, data, radius, yaw))

    survivors = [r for r in results if r.ok]
    print(f"{len(results)} evaluated, {len(survivors)} survive all gates, "
          f"{len(results) - len(survivors)} REJECTED\n")

    reasons: dict[str, int] = {}
    for r in results:
        for x in r.rejections:
            reasons[x.split()[0]] = reasons.get(x.split()[0], 0) + 1
    print("rejection reasons (a geometry can fail several):")
    for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print(f"  {k:<12} {v:4d}")

    if not survivors:
        print("\nNO GEOMETRY SURVIVES. The gates and the sweep ranges are "
              "mutually unsatisfiable -- see the trade curve below.")
        _trade_curve(results)
        return 1

    print(f"\n{'L1':>6} {'L2':>6} {'R':>6} {'yaw':>5} {'cov':>5} {'worstR':>7} "
          f"{'res mm':>7} {'torque':>7} {'tension':>8} {'clear mm':>9}")
    for r in sorted(survivors, key=lambda r: -r.coverage)[:20]:
        print(f"{r.l1:6.3f} {r.l2:6.3f} {r.radius:6.3f} {math.degrees(r.yaw):5.0f} "
              f"{r.coverage * 100:4.0f}% {r.worst_radius_m:7.3f} "
              f"{r.worst_resolution_m * 1000:7.3f} {r.peak_torque_nm:7.1f} "
              f"{r.worst_tension_n:8.0f} {r.min_clearance_m * 1000:9.1f}")
    _trade_curve(results)
    return 0


def _trade_curve(results: list[Result]) -> None:
    """Coverage against the binding constraints, over full-coverage geometries."""
    full = [r for r in results if r.coverage >= 1.0]
    print(f"\nTRADE CURVE -- {len(full)} placements reach all 64 squares. "
          f"For those, what the gates cost:")
    if not full:
        print("  none")
        return
    print(f"{'L1':>6} {'L2':>6} {'R':>6} {'yaw':>5} {'worstR':>7} {'res mm':>7} "
          f"{'torque':>7} {'tension/allow':>14} {'clear mm':>9}  verdict")
    for r in sorted(full, key=lambda r: (r.peak_torque_nm, r.worst_resolution_m))[:25]:
        verdict = "OK" if r.ok else "; ".join(r.rejections)
        print(f"{r.l1:6.3f} {r.l2:6.3f} {r.radius:6.3f} {math.degrees(r.yaw):5.0f} "
              f"{r.worst_radius_m:7.3f} {r.worst_resolution_m * 1000:7.3f} "
              f"{r.peak_torque_nm:7.1f} {r.worst_tension_n:6.0f}/{r.allowable_tension_n:<7.0f} "
              f"{r.min_clearance_m * 1000:9.1f}  {verdict}")


if __name__ == "__main__":
    raise SystemExit(main())
