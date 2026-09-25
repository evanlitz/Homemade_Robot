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

from backends.sim_backend import backend_for
from config import load_architecture
from motion.architectures import kinematics_for
from motion.collision import clearance
from motion.kinematics import inverse_kinematics, tool_m_per_full_step
from motion.limits import within_limits

FILES = "abcdefgh"


# Acceleration probes, as MULTIPLES of each joint's configured limit. Multiples
# rather than absolute values because a prismatic joint's limit is m/s^2 and a
# revolute one's is rad/s^2, and a shared absolute number would silently mix
# them. 0.0 is the gravity-only row.
ALPHA_SCALES = (0.0, 1.0, 2.0, 4.0)


@dataclass
class Result:
    l1: float
    l2: float
    radius: float
    yaw: float
    coverage: float
    worst_radius_m: float
    near_radius_m: float
    worst_resolution_m: float
    peak_torque_nm: float
    peak_force_n: float
    torque_by_alpha: tuple[float, ...]
    worst_tension_n: float
    allowable_tension_n: float
    min_clearance_m: float
    min_self_clear_m: float
    self_culprit: str
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


def board_edge_distance(cfg, radius: float, yaw: float) -> float:
    """Distance from the yaw axis to the board outline. 0 if the axis is inside.

    The board is a square of side `board_span_m` centred at (radius, 0) and
    rotated by `yaw`. Point-to-rectangle distance in the board's own frame.
    """
    h = cfg.task.board_span_m / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.array([-radius * c, radius * s])
    outside = np.maximum(np.abs(local) - h, 0.0)
    return float(np.linalg.norm(outside))


def gripper_clearance(cfg, target: np.ndarray, neighbours: list[np.ndarray],
                      tool_yaw: float) -> float:
    """Least gap between the gripper footprint and any neighbouring piece.

    D3-REQ. `tool_yaw` is the gripper's heading, which is architecture-specific
    and must come from the kinematics rather than be assumed: the palletizer's
    is pinned RADIALLY to atan2(y, x) by D3, while a SCARA's gripper is bolted
    to link 2 and points wherever the forearm points. Same check, same
    footprint, different orientation -- assuming radial for both would test the
    wrong geometry for one of them. Negative means collision.

    Footprint is a rectangle: jaw span across the jaw axis, body width along the
    heading.
    """
    half_jaw = (cfg.gripper["jaw_span_open_m"] + 2 * cfg.gripper["jaw_thickness_m"]) / 2.0
    half_body = cfg.gripper["body_width_m"] / 2.0
    piece_r = cfg.gripper["piece_diameter_m"] / 2.0
    c, s = math.cos(tool_yaw), math.sin(tool_yaw)

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
    kin = kinematics_for(cfg)
    backend = backend_for(cfg)
    qpos_from_absolute = backend.qpos_from_absolute
    # Which generalised forces are torques (N.m) and which are forces (N). A max
    # taken across both would be a max over mixed units, which is meaningless
    # and would still print.
    rotary = [i for i, j in enumerate(cfg.joints) if not j.drive.is_linear]
    linear = [i for i, j in enumerate(cfg.joints) if j.drive.is_linear]
    alpha = np.array([j.max_accel_rad_s2 for j in cfg.joints])

    reached = 0
    worst_r = 0.0
    # Distance from the yaw axis to the board's physical outline -- NOT to the
    # nearest square centre, and NOT filtered by reachability. The base column
    # runs from the table up to the shoulder, so it occupies this space for the
    # board's whole thickness; anything below the column radius means the board
    # and the machine are in the same place. Zero means the axis is inside the
    # board outline. There is no gate on this yet -- see the report at the end.
    near_r = board_edge_distance(cfg, radius, yaw)
    worst_res = 0.0
    peak_tau = 0.0
    peak_force = 0.0
    peak_by_alpha = [0.0] * len(ALPHA_SCALES)
    min_clear = float("inf")
    min_self, self_culprit = float("inf"), "none"

    for idx, target in enumerate(squares):
        sols = [s for s in inverse_kinematics(cfg, target) if within_limits(cfg, s)]
        if not sols:
            continue
        reached += 1
        theta = min(sols, key=kin.branch_key)
        worst_r = max(worst_r, float(math.hypot(target[0], target[1])))
        worst_res = max(worst_res, float(tool_m_per_full_step(cfg, theta).max()))

        data.qpos[:] = qpos_from_absolute(model, *theta)
        data.qvel[:] = 0.0
        # Gravity hold PLUS the inertial term at the configured acceleration
        # limit. Gravity alone understates the peak: reflected rotor inertia is
        # comparable to the arm itself (F6), so accelerating costs real torque.
        # qacc is built with the same linear absolute->joint map as qpos, which
        # keeps it on the constraint manifold -- an arbitrary qacc would fight
        # the loop constraints and produce nonsense. Harmless but still correct
        # for an open chain, where the map is nearly the identity.
        for k, scale in enumerate(ALPHA_SCALES):
            data.qacc[:] = qpos_from_absolute(model, *(alpha * scale))
            mujoco.mj_inverse(model, data)
            # By name and in ABSOLUTE coordinates, per architecture. Slicing
            # qfrc_inverse positionally reads whichever joints the tree happened
            # to declare first, which is not the actuated set for either
            # candidate.
            f = np.abs(backend.actuated_forces(model, data))
            if scale == 1.0:
                peak_tau = max(peak_tau, float(f[rotary].max()))
                if linear:
                    peak_force = max(peak_force, float(f[linear].max()))
            peak_by_alpha[k] = max(peak_by_alpha[k], float(f[rotary].max()))

        neighbours = [squares[k] for k in _neighbours(idx)]
        min_clear = min(min_clear, gripper_clearance(
            cfg, target, neighbours, kin.tool_yaw(cfg, theta)))

        # Self-collision, geometric and outside the physics -- collision is off
        # in both models and cannot be turned back on (F1). Checked at the grip
        # pose AND at the travel pose above it: the arm folds differently to
        # reach a near square than it does to hover over one.
        for z in (target[2], cfg.gripper["travel_height_m"]):
            above = [s for s in inverse_kinematics(
                cfg, np.array([target[0], target[1], z]))
                if within_limits(cfg, s)]
            if not above:
                continue
            moving, static = kin.capsules(cfg, min(above, key=kin.branch_key))
            gap, who = clearance(moving, static, kin.COLLISION_PAIRS)
            if gap < min_self:
                min_self, self_culprit = gap, who

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
                          worst_tension_n=worst_tension, allowable_tension_n=allowable,
                          board_gap_m=near_r)
    if coverage < 1.0:
        rej.append(f"coverage {coverage * 100:.0f}% < 100%")
    # Gated at zero -- interpenetration, not a margin. A build margin would be
    # an invented number, and the actual gap is reported so one can be chosen
    # later from evidence.
    if min_self < 0.0:
        rej.append(f"selfcollide {min_self * 1000:.0f} mm at {self_culprit}")

    return Result(cfg.g("upper_arm_m"), cfg.g("forearm_m"), radius, yaw, coverage,
                  worst_r, near_r, worst_res, peak_tau, peak_force,
                  tuple(peak_by_alpha), worst_tension, allowable,
                  0.0 if min_clear == float("inf") else min_clear,
                  0.0 if min_self == float("inf") else min_self, self_culprit,
                  tuple(rej))


def _neighbours(idx: int) -> list[int]:
    i, j = divmod(idx, 8)
    return [a * 8 + b for a in (i - 1, i, i + 1) for b in (j - 1, j, j + 1)
            if 0 <= a < 8 and 0 <= b < 8 and (a, b) != (i, j)]


def evaluate_all(base) -> list[Result]:
    """Every point of the configured sweep box, gated. No printing."""
    build_mjcf = backend_for(base).build_mjcf
    axes = {a.name: a.values() for a in base.sweep}
    results: list[Result] = []
    for l1 in axes["upper_arm_m"]:
        for l2 in axes["forearm_m"]:
            cfg = replace(base, geometry={**base.geometry,
                                          "upper_arm_m": l1, "forearm_m": l2})
            model = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
            data = mujoco.MjData(model)
            for radius in axes["board_radius_m"]:
                for yaw in axes["board_yaw_rad"]:
                    results.append(evaluate(cfg, model, data, radius, yaw))
    return results


def main(name: str = "palletizer") -> int:
    base = load_architecture(name)
    axes = {a.name: a.values() for a in base.sweep}

    print(f"architecture: {base.name} (candidate {base.candidate_id}, "
          f"closed_loop={base.closed_loop})")
    print(f"sweep: {len(axes['upper_arm_m'])} x {len(axes['forearm_m'])} link combos "
          f"x {len(axes['board_radius_m'])} radii x {len(axes['board_yaw_rad'])} yaws "
          f"= {np.prod([len(v) for v in axes.values()]):.0f} evaluations\n")

    results = evaluate_all(base)

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

    # `lift N` only exists for an architecture with a prismatic joint. Printing
    # it as a blank column for the palletizer keeps the two tables the same
    # shape, which is the point of a comparison.
    lift = "lift N" if any(j.drive.is_linear for j in base.joints) else ""
    print(f"\n{'L1':>6} {'L2':>6} {'R':>6} {'yaw':>5} {'cov':>5} {'baseGap':>7}"
          f"{'worstR':>7} {'res mm':>7} {'torque':>7} {lift:>7} {'tension':>8} "
          f"{'clear mm':>9} {'self mm':>8}  worst pair")
    for r in sorted(survivors, key=lambda r: (r.worst_resolution_m, r.peak_torque_nm))[:20]:
        force = f"{r.peak_force_n:7.0f}" if lift else " " * 7
        print(f"{r.l1:6.3f} {r.l2:6.3f} {r.radius:6.3f} {math.degrees(r.yaw):5.0f} "
              f"{r.coverage * 100:4.0f}% {r.near_radius_m:7.3f}{r.worst_radius_m:7.3f} "
              f"{r.worst_resolution_m * 1000:7.3f} {r.peak_torque_nm:7.1f} {force} "
              f"{r.worst_tension_n:8.0f} {r.min_clearance_m * 1000:9.1f} "
              f"{r.min_self_clear_m * 1000:8.1f}  {r.self_culprit}")

    _pinning(axes, survivors)
    _base_clearance(survivors)
    _trade_curve(results)
    _alpha_sensitivity(survivors)
    return 0


def _base_clearance(survivors: list[Result]) -> None:
    """What each base-column radius costs. Now GATED at `base_clearance_m` (D13),
    so the survivors passed in have already cleared it; this shows the cost of
    the choice and what a larger column would cost if the yaw stack needs one.
    """
    print("\nBASE-COLUMN CLEARANCE -- survivors remaining at each column radius:")
    print(f"{'column radius':>14} {'survivors':>10} {'best res mm':>12} {'best torque':>12}")
    for column in (0.0, 0.10, 0.15, 0.20, 0.25):
        keep = [r for r in survivors if r.near_radius_m >= column]
        if not keep:
            print(f"{column:14.3f} {0:10d} {'--':>12} {'--':>12}")
            continue
        best = min(keep, key=lambda r: (r.worst_resolution_m, r.peak_torque_nm))
        print(f"{column:14.3f} {len(keep):10d} {best.worst_resolution_m * 1000:12.3f} "
              f"{best.peak_torque_nm:12.1f}")


def _pinning(axes: dict[str, list[float]], survivors: list[Result]) -> None:
    """Does the surviving set touch the edge of the box?

    A survivor sitting on an axis minimum or maximum means the box was drawn in
    the wrong place -- the optimum is outside it. That is a different failure
    from a grid being too coarse, and it is not fixed by subdividing.
    """
    got = {"upper_arm_m": [r.l1 for r in survivors],
           "forearm_m": [r.l2 for r in survivors],
           "board_radius_m": [r.radius for r in survivors],
           "board_yaw_rad": [r.yaw for r in survivors]}
    print("\nBOX CHECK -- is the optimum inside the swept ranges?")
    for name, vals in got.items():
        lo, hi = min(axes[name]), max(axes[name])
        at_lo = sum(1 for v in vals if abs(v - lo) < 1e-9)
        at_hi = sum(1 for v in vals if abs(v - hi) < 1e-9)
        note = []
        if at_lo:
            note.append(f"{at_lo} at min")
        if at_hi:
            note.append(f"{at_hi} at max")
        verdict = "PINNED -- extend" if (at_lo or at_hi) else "interior"
        print(f"  {name:<16} [{lo:.3f}, {hi:.3f}]  "
              f"survivors {min(vals):.3f}-{max(vals):.3f}  "
              f"{', '.join(note) or 'none on the edge':<22} {verdict}")


def _alpha_sensitivity(survivors: list[Result]) -> None:
    """Peak torque against the acceleration limit, over the surviving set.

    `max_accel_rad_s2` is still a placeholder and torque is affine in it, so
    every torque figure above is conditional on that number. This is the table
    that says how conditional.
    """
    print("\nACCELERATION SENSITIVITY -- peak REVOLUTE joint torque over "
          "surviving geometries, N.m")
    print(f"{'x configured':>13} {'min':>7} {'max':>7}")
    for k, scale in enumerate(ALPHA_SCALES):
        vals = [r.torque_by_alpha[k] for r in survivors]
        label = "gravity only" if scale == 0.0 else f"{scale:.0f}x"
        print(f"{label:>13} {min(vals):7.1f} {max(vals):7.1f}")


def _trade_curve(results: list[Result]) -> None:
    """Coverage against the binding constraints, over full-coverage geometries."""
    full = [r for r in results if r.coverage >= 1.0]
    print(f"\nTRADE CURVE -- {len(full)} placements reach all 64 squares. "
          f"For those, what the gates cost:")
    if not full:
        print("  none")
        return
    print(f"{'L1':>6} {'L2':>6} {'R':>6} {'yaw':>5} {'baseGap':>7}{'worstR':>7} "
          f"{'res mm':>7} {'torque':>7} {'tension/allow':>14} {'clear mm':>9}  verdict")
    for r in sorted(full, key=lambda r: (r.worst_resolution_m, r.peak_torque_nm))[:25]:
        verdict = "OK" if r.ok else "; ".join(r.rejections)
        print(f"{r.l1:6.3f} {r.l2:6.3f} {r.radius:6.3f} {math.degrees(r.yaw):5.0f} "
              f"{r.near_radius_m:7.3f}{r.worst_radius_m:7.3f} "
              f"{r.worst_resolution_m * 1000:7.3f} "
              f"{r.peak_torque_nm:7.1f} {r.worst_tension_n:6.0f}/{r.allowable_tension_n:<7.0f} "
              f"{r.min_clearance_m * 1000:9.1f}  {verdict}")


if __name__ == "__main__":
    import sys

    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1 else "palletizer"))
