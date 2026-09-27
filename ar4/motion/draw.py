"""Pen drawing: a page in space, strokes on it, and joint paths that trace them.

Pure motion code, like the rest of motion/: nothing here opens a port or a
viewer. tools/draw.py is the executable end.

The pipeline:

    load_svg -> fit_to_page -> simplify -> order_strokes -> plan_drawing
    (SVG units)  (page mm)     (fewer pts)  (less pen-up)    (joint segments)

THE PAGE IS MEASURED, NOT ASSUMED. Tormach users drawing with a 6-axis arm
found the pen-to-paper height varying 20-25 mm across a page -- far more than
any spring in a pen holder absorbs. So the page is a frame fitted to points
the pen tip was actually touched to (Page.from_touches), and every pose is
built in that frame. Touching off with the drawing tool also cancels any error
in the tool length: it moves the fitted plane along its own normal, which is
the direction the pen is pointed anyway.

STROKES ARE CONTINUOUS. plan_path in motion.trajectory stops at every waypoint,
which is right for chess and hopeless for a curve made of 400 short segments.
plan_stroke runs the whole polyline as one move: the tip stays exactly on the
line, at constant speed, slowing only where a corner or tight curve demands
it (grbl's junction-deviation rule plus a curvature limit).

Units: page coordinates (u, v) in mm from the page's origin corner; heights
along the page normal in mm, positive away from the paper; poses in metres,
like everywhere else in motion/.
"""
from __future__ import annotations

import json
import math

import numpy as np

from motion.ik import ik, ik_multistart
from motion.kinematics import fk, jacobian
from motion.trajectory import (DT, PlanningError, Trajectory, check_waypoint,
                               plan_line)

# --- defaults ---------------------------------------------------------------

# Drawing speed and acceleration along the stroke, mm/s and mm/s^2. Slow
# enough for a felt-tip to lay down a solid line. On the real arm the firmware
# limits will usually stretch this further (hw.required_slowdown).
DRAW_V_MM_S = 40.0
DRAW_A_MM_S2 = 300.0
# grbl's junction deviation: how far the path may notionally cut a corner when
# choosing a cornering speed. The planned path never actually cuts it -- this
# only sets how fast the tip may pass through the vertex. 0.05 mm is grbl's
# default and gives ~6 mm/s through a right angle at DRAW_A_MM_S2.
JUNCTION_MM = 0.05
# Pen-up moves.
TRAVEL_V_MM_S = 120.0
TRAVEL_A_MM_S2 = 400.0
PLUNGE_V_MM_S = 20.0
# Height of the pen above the page between strokes, mm.
LIFT_MM = 10.0
# How far below the touched surface to aim while drawing. A pen needs contact
# pressure, and the only safe way to get it is a compliant holder (spring or
# foam) taking up this much travel. With a rigid mount set this to 0 -- and
# expect gaps, because the page is never flatter than the arm is accurate.
PRESS_MM = 1.0
# Resolution of the stroke's speed profile along its length, mm.
PROFILE_DS_MM = 0.5
# Chord tolerance when flattening SVG curves and simplifying, mm on the page.
FLATTEN_MM = 0.05

# The J2 axis sits ~170 mm above the base. The page normal is oriented to face
# it, which is "up" for a page on the table and "toward the robot" for one on
# an easel -- the side the pen arrives from, either way.
_SHOULDER_MM = np.array([0.0, 0.0, 170.0])


# --- the page -----------------------------------------------------------------

class Page:
    """A rectangular drawing surface: origin corner, in-plane axes, normal.

    `origin_mm` is the corner (u, v) = (0, 0), in the robot base frame, mm.
    `rotation` has columns (u axis, v axis, normal); the normal points out of
    the paper toward the pen. `residual_mm` is the worst distance of a touched
    point from the fitted plane -- if it exceeds PRESS_MM, the pen holder
    cannot absorb it and parts of the drawing will skip.
    """

    def __init__(self, origin_mm, rotation, width_mm, height_mm, residual_mm=0.0):
        self.origin_mm = np.asarray(origin_mm, dtype=float)
        self.rotation = np.asarray(rotation, dtype=float)
        self.width_mm = float(width_mm)
        self.height_mm = float(height_mm)
        self.residual_mm = float(residual_mm)

    @property
    def normal(self):
        return self.rotation[:, 2]

    @classmethod
    def flat(cls, center_mm, width_mm, height_mm, z_mm=None):
        """A page lying on a horizontal table, for the simulator and for
        planning before anything has been touched off.

        Oriented to read correctly from behind the robot looking along +x:
        u runs to the right (world -y), v away from the robot (world +x).
        """
        c = np.asarray(center_mm, dtype=float)
        if z_mm is not None:
            c[2] = z_mm
        r = np.column_stack(([0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]))
        origin = c - r[:, 0] * width_mm / 2.0 - r[:, 1] * height_mm / 2.0
        return cls(origin, r, width_mm, height_mm)

    @classmethod
    def from_touches(cls, origin_mm, u_point_mm, others_mm, width_mm, height_mm):
        """Fit a page to touched pen-tip positions, base frame, mm.

        `origin_mm` is the corner the drawing is measured from, `u_point_mm`
        any point along the page's bottom edge (the u direction), `others_mm`
        at least one more point anywhere on the page. With more than three
        points the plane is a least-squares fit and `residual_mm` says how flat
        the page actually is.
        """
        pts = np.array([origin_mm, u_point_mm, *others_mm], dtype=float)
        if len(pts) < 3:
            raise ValueError("need at least three touched points")
        centroid = pts.mean(axis=0)
        _, sv, vt = np.linalg.svd(pts - centroid)
        if sv[1] < 1e-6 * max(sv[0], 1.0):
            raise ValueError("touched points are collinear; spread them over the page")
        n = vt[2]
        if np.dot(n, _SHOULDER_MM - centroid) < 0:
            n = -n
        residual = float(np.max(np.abs((pts - centroid) @ n)))

        # Project the two edge points into the plane so the frame is exactly
        # orthonormal even when the touches are not exactly coplanar.
        def onto(p):
            return p - np.dot(p - centroid, n) * n
        o, up = onto(pts[0]), onto(pts[1])
        u = up - o
        if np.linalg.norm(u) < 1.0:
            raise ValueError("origin and u-direction touches are under 1 mm apart")
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        return cls(o, np.column_stack((u, v, n)), width_mm, height_mm, residual)

    def point_mm(self, u, v, h=0.0):
        """Base-frame point, mm, for page (u, v) at height h above the paper."""
        return self.origin_mm + self.rotation @ np.array([u, v, h], dtype=float)

    def to_page(self, p_mm):
        """Inverse of point_mm: base-frame mm -> (u, v, h)."""
        return self.rotation.T @ (np.asarray(p_mm, dtype=float) - self.origin_mm)

    def toward_base(self):
        """Unit in-plane direction from the page centre toward the shoulder."""
        c = self.point_mm(self.width_mm / 2.0, self.height_mm / 2.0)
        d = _SHOULDER_MM - c
        d -= np.dot(d, self.normal) * self.normal
        if np.linalg.norm(d) < 1e-9:      # page directly under the shoulder
            return -self.rotation[:, 1]
        return d / np.linalg.norm(d)

    def pen_rotation(self, yaw_deg, tilt_deg=0.0):
        """Tool orientation for a pen pointed into the paper.

        `tilt_deg` leans the pen off the normal with its tip toward the robot
        base (the flange further out than the tip). `yaw_deg` spins the tool
        about the pen axis. The pen draws the same line at any yaw and at a
        modest tilt; both are chosen for the arm's sake -- see choose_pen.
        """
        t = np.deg2rad(tilt_deg)
        z = -math.cos(t) * self.normal + math.sin(t) * self.toward_base()
        a = np.deg2rad(yaw_deg)
        x0 = math.cos(a) * self.rotation[:, 0] + math.sin(a) * self.rotation[:, 1]
        x = x0 - np.dot(x0, z) * z
        x /= np.linalg.norm(x)
        return np.column_stack((x, np.cross(z, x), z))

    def pose(self, u, v, h, pen=(0.0, 0.0)):
        """Tool pose at page (u, v), height h mm, pen = (yaw_deg, tilt_deg)."""
        m = np.eye(4)
        m[:3, :3] = self.pen_rotation(*pen)
        m[:3, 3] = self.point_mm(u, v, h) / 1000.0
        return m

    def to_json(self):
        return {"origin_mm": self.origin_mm.tolist(),
                "rotation": self.rotation.tolist(),
                "width_mm": self.width_mm, "height_mm": self.height_mm,
                "residual_mm": self.residual_mm}

    @classmethod
    def from_json(cls, d):
        return cls(d["origin_mm"], d["rotation"], d["width_mm"], d["height_mm"],
                   d.get("residual_mm", 0.0))

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_json(), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.from_json(json.load(f))


# --- strokes ------------------------------------------------------------------

def load_svg(path, curve_step=0.5):
    """Every visible outline in an SVG as a list of (N, 2) polylines.

    Units are the SVG's user units after its own transforms and viewBox; only
    the shape matters, since fit_to_page rescales. Curves are sampled at
    `curve_step` user units and later simplified to FLATTEN_MM on the page.
    Fills are ignored -- a plotter draws outlines. Text must be converted to
    paths first (Inkscape: Path > Object to Path).
    """
    from svgelements import SVG, Close, Line, Move, Path, Shape

    svg = SVG.parse(str(path), reify=True)
    strokes = []
    for el in svg.elements():
        if not isinstance(el, Shape):
            continue
        if el.values.get("visibility") in ("hidden", "collapse") \
                or el.values.get("display") == "none":
            continue
        cur = []
        for seg in Path(el):
            if isinstance(seg, Move):
                if len(cur) > 1:
                    strokes.append(np.array(cur))
                cur = [(seg.end.x, seg.end.y)]
            elif isinstance(seg, (Line, Close)):
                if seg.start is not None and not cur:
                    cur = [(seg.start.x, seg.start.y)]
                cur.append((seg.end.x, seg.end.y))
            else:
                n = max(2, int(math.ceil(seg.length() / curve_step)))
                for t in np.linspace(0.0, 1.0, n + 1)[1:]:
                    p = seg.point(t)
                    cur.append((p.x, p.y))
        if len(cur) > 1:
            strokes.append(np.array(cur))
    return [s for s in strokes if len(s) > 1]


def fit_to_page(strokes, width_mm, height_mm, margin_mm=15.0, flip_y=True):
    """Scale uniformly and centre inside the page's margins.

    flip_y because SVG's y runs down the screen and the page's v runs up it;
    without it every drawing comes out mirrored top to bottom.
    """
    pts = np.vstack(strokes)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    size = np.maximum(hi - lo, 1e-9)
    avail = np.array([width_mm, height_mm]) - 2.0 * margin_mm
    if np.any(avail <= 0):
        raise ValueError("margins leave no room on the page")
    scale = float(np.min(avail / size))
    centre_in = (lo + hi) / 2.0
    centre_out = np.array([width_mm, height_mm]) / 2.0
    out = []
    for s in strokes:
        p = (s - centre_in) * scale
        if flip_y:
            p[:, 1] = -p[:, 1]
        out.append(p + centre_out)
    return out


def simplify(stroke, tol=FLATTEN_MM):
    """Ramer-Douglas-Peucker, iterative. Drops points within `tol` of the line
    their neighbours make, so a straight run of 200 samples becomes 2 points
    and the stroke planner has fewer, longer segments to cross."""
    p = np.asarray(stroke, dtype=float)
    # consecutive duplicates first: zero-length segments have no direction
    keep = np.concatenate(([True], np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-9))
    p = p[keep]
    if len(p) < 3:
        return p
    mask = np.zeros(len(p), dtype=bool)
    mask[0] = mask[-1] = True
    stack = [(0, len(p) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = p[i], p[j]
        ab = b - a
        L = np.linalg.norm(ab)
        seg = p[i + 1:j] - a
        if L < 1e-12:
            d = np.linalg.norm(seg, axis=1)
        else:
            d = np.abs(ab[0] * seg[:, 1] - ab[1] * seg[:, 0]) / L
        k = int(np.argmax(d))
        if d[k] > tol:
            mask[i + 1 + k] = True
            stack += [(i, i + 1 + k), (i + 1 + k, j)]
    return p[mask]


def order_strokes(strokes, start=(0.0, 0.0)):
    """Greedy nearest-neighbour ordering, reversing strokes where that helps.

    Pen-up travel is pure waste and on a stepper arm it is slow waste. Greedy
    is not optimal but typically removes most of the travel of an SVG's
    arbitrary document order, and it is O(n^2) on stroke count, not points.
    """
    remaining = list(range(len(strokes)))
    heads = np.array([s[0] for s in strokes])
    tails = np.array([s[-1] for s in strokes])
    pos = np.asarray(start, dtype=float)
    out = []
    while remaining:
        idx = np.array(remaining)
        dh = np.linalg.norm(heads[idx] - pos, axis=1)
        dt_ = np.linalg.norm(tails[idx] - pos, axis=1)
        k = int(np.argmin(np.minimum(dh, dt_)))
        s = strokes[idx[k]]
        if dt_[k] < dh[k]:
            s = s[::-1]
        out.append(s)
        pos = s[-1]
        remaining.pop(k)
    return out


def travel_mm(strokes, start=(0.0, 0.0)):
    """Total pen-up distance for strokes drawn in the given order."""
    pos, total = np.asarray(start, dtype=float), 0.0
    for s in strokes:
        total += float(np.linalg.norm(s[0] - pos))
        pos = s[-1]
    return total


# --- the arm ------------------------------------------------------------------

# Worst |J5| over the page that choose_pen is satisfied with. Five times
# motion.trajectory.MIN_J5_DEG, because that is a hard floor for rejecting a
# waypoint, not a comfortable place to draw from: near J5 = 0 the wrist has to
# spin J4 and J6 fast for small tip motions, and on this firmware fast means
# falling behind.
TARGET_J5_DEG = 20.0


def choose_pen(page, tool, tilts=(0, 10, 15, 20, 25, 30), yaws=(0, 90, 180, 270),
               grid=7, h=LIFT_MM, target_j5=TARGET_J5_DEG):
    """(yaw_deg, tilt_deg, worst |J5|) for drawing this page with this tool.

    WHY THE PEN LEANS. With the pen straight down, J5 = 0 wherever the forearm
    lines up with the pen, and on an A4 page on the table that happens
    somewhere on the page at EVERY yaw -- measured, the best vertical-pen yaw
    still came within 0.6 deg of the singularity. Leaning the tip toward the
    base moves that line off the page: clearance grows ~1.4 deg per degree of
    tilt, and leaning away does not reach the page at all. A felt-tip or
    ballpoint draws the same line at 20 deg of lean.

    Walks a grid over the page at pen-up height for each (tilt, yaw) and
    returns the SMALLEST tilt whose worst |J5| reaches target_j5 (the most
    natural pen angle that is comfortably clear), or failing that the most
    clearance on offer. Raises if nothing reaches every corner.
    """
    us = np.linspace(0.0, page.width_mm, grid)
    vs = np.linspace(0.0, page.height_mm, grid)
    best = None
    for tilt in sorted(tilts):
        for yaw in yaws:
            q, worst, ok = None, float("inf"), True
            # serpentine, so each IK warm-starts from a near neighbour
            for i, v in enumerate(vs):
                for u in (us if i % 2 == 0 else us[::-1]):
                    pose = page.pose(u, v, h, (yaw, tilt))
                    q = (ik_multistart(pose, tool=tool) if q is None
                         else ik(pose, q, tool=tool))
                    if q is None:
                        ok = False
                        break
                    worst = min(worst, abs(q[4]))
                if not ok:
                    break
            if ok and (best is None or worst > best[2]):
                best = (float(yaw), float(tilt), float(worst))
        if best is not None and best[2] >= target_j5:
            return best
    if best is None:
        raise PlanningError("no pen orientation reaches the whole page -- move "
                            "the page or make it smaller", 0, 0.0)
    return best


def _speed_profile(lengths, turn, v_max, a_max, junction, ds_max):
    """Arc-length grid s and the speed allowed at each grid point.

    lengths: the polyline's segment lengths; turn: the heading change at each
    interior vertex, radians. Standard two-pass planner: cap each vertex,
    then enforce |dv^2/ds| <= 2a forwards and backwards.
    """
    s, cap = [0.0], [0.0]
    for k, L in enumerate(lengths):
        n = max(1, int(math.ceil(L / ds_max)))
        for j in range(1, n + 1):
            s.append(s[-1] + L / n)
            cap.append(v_max)
        if k < len(turn):
            phi = turn[k]
            # junction deviation: radius of the circle tangent to both
            # segments whose closest approach to the vertex is `junction`
            c = math.cos(phi / 2.0)
            v_j = v_max if c >= 1.0 - 1e-12 else math.sqrt(a_max * junction * c / (1.0 - c))
            # and the curvature of a finely sampled curve: sagitta-free
            # estimate r = segment / turn angle, using the shorter neighbour
            r = min(L, lengths[k + 1]) / max(phi, 1e-12)
            v_c = math.sqrt(a_max * r)
            cap[-1] = min(cap[-1], v_j, v_c)
    cap[-1] = 0.0
    s, v = np.array(s), np.array(cap)
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_max * (s[i] - s[i - 1])))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_max * (s[i + 1] - s[i])))
    return s, v


def plan_stroke(q_start, points_mm, rotation, tool, v_max=DRAW_V_MM_S,
                a_max=DRAW_A_MM_S2, junction=JUNCTION_MM, dt=DT,
                ds_max=PROFILE_DS_MM, validate=True, **checks):
    """One continuous move along a polyline, base-frame mm, fixed orientation.

    The tip is on the polyline at every waypoint -- the profile only decides
    how fast it moves along it -- and starts and ends at rest. q_start must
    already put the tool on points_mm[0].
    """
    p = np.asarray(points_mm, dtype=float)
    keep = np.concatenate(([True], np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-6))
    p = p[keep]
    q_start = np.asarray(q_start, dtype=float)
    if len(p) < 2:
        return Trajectory(q_start.reshape(1, 6), dt)

    d = np.diff(p, axis=0)
    lengths = np.linalg.norm(d, axis=1)
    unit = d / lengths[:, None]
    turn = np.arccos(np.clip(np.sum(unit[:-1] * unit[1:], axis=1), -1.0, 1.0))
    s_grid, v_grid = _speed_profile(lengths, turn, v_max, a_max, junction, ds_max)

    # time at each grid point from the average speed across each step
    ds = np.diff(s_grid)
    vsum = v_grid[:-1] + v_grid[1:]
    with np.errstate(divide="ignore"):
        step_t = np.where(vsum > 0, 2.0 * ds / np.where(vsum > 0, vsum, 1.0),
                          2.0 * np.sqrt(ds / a_max))
    t_grid = np.concatenate(([0.0], np.cumsum(step_t)))

    n = max(1, int(math.ceil(t_grid[-1] / dt)))
    t = np.linspace(0.0, t_grid[-1], n + 1)
    # Constant acceleration within each grid step, NOT linear interpolation of
    # s against t: at low speed one 0.5 mm step lasts tens of milliseconds,
    # and interpolating across it would jump from rest to the step's mean
    # speed in a single waypoint.
    i = np.clip(np.searchsorted(t_grid, t, side="right") - 1, 0, len(ds) - 1)
    tau = t - t_grid[i]
    v0 = v_grid[i]
    acc = (v_grid[i + 1] ** 2 - v0 ** 2) / (2.0 * ds[i])
    s = np.minimum(s_grid[i] + v0 * tau + 0.5 * acc * tau ** 2, s_grid[i + 1])
    s[-1] = s_grid[-1]
    s_vertex = np.concatenate(([0.0], np.cumsum(lengths)))
    pts = np.column_stack([np.interp(s, s_vertex, p[:, k]) for k in range(3)])

    out = [q_start]
    pose = np.eye(4)
    pose[:3, :3] = rotation
    for i in range(1, len(pts)):
        pose[:3, 3] = pts[i] / 1000.0
        q = ik(pose, out[-1], tool=tool)
        if q is None:
            raise PlanningError("unreachable", i, s[i] / s[-1])
        if validate:
            check_waypoint(q, out[-1], i, s[i] / s[-1], **checks)
        out.append(q)
    return Trajectory(np.array(out), t[1] - t[0] if n > 0 else dt)


def plan_drawing(q_start, strokes, page, tool, pen=None, lift_mm=LIFT_MM,
                 press_mm=PRESS_MM, draw_v=DRAW_V_MM_S, draw_a=DRAW_A_MM_S2,
                 travel_v=TRAVEL_V_MM_S, travel_a=TRAVEL_A_MM_S2,
                 plunge_v=PLUNGE_V_MM_S, dt=DT):
    """Strokes in page mm -> a list of (kind, payload) steps for a backend.

    kind "approach": payload is a joint target for move_joints -- the first
    move from wherever the arm is to above the first stroke, which has no
    reason to be a straight line. Every other kind ("travel", "down", "draw",
    "up") is a Trajectory for follow(), and each one ends at rest.

    `pen` is (yaw_deg, tilt_deg); None asks choose_pen. Every stroke is
    planned before anything is returned, so a drawing that cannot be
    completed fails here rather than halfway down the page.
    """
    if pen is None:
        pen = choose_pen(page, tool)[:2]
    rot = page.pen_rotation(*pen)
    for s in strokes:
        u, v = s[:, 0], s[:, 1]
        if u.min() < -1e-6 or v.min() < -1e-6 or \
                u.max() > page.width_mm + 1e-6 or v.max() > page.height_mm + 1e-6:
            raise ValueError("a stroke leaves the page; use fit_to_page")

    def pose(u, v, h):
        return page.pose(u, v, h, pen)

    steps = []
    first = strokes[0][0]
    q = ik(pose(first[0], first[1], lift_mm), q_start, tool=tool)
    if q is None:
        q = ik_multistart(pose(first[0], first[1], lift_mm), tool=tool)
    if q is None:
        raise PlanningError("first stroke unreachable", 0, 0.0)
    steps.append(("approach", q))

    line = dict(tool=tool, dt=dt)
    for k, s in enumerate(strokes):
        u0, v0 = s[0]
        if k > 0:
            seg = plan_line(q, pose(u0, v0, lift_mm), v_max=travel_v,
                            a_max=travel_a, **line)
            steps.append(("travel", seg))
            q = seg.q[-1]
        seg = plan_line(q, pose(u0, v0, -press_mm), v_max=plunge_v,
                        a_max=travel_a, **line)
        steps.append(("down", seg))
        q = seg.q[-1]
        pts = np.array([page.point_mm(u, v, -press_mm) for u, v in s])
        seg = plan_stroke(q, pts, rot, tool, v_max=draw_v, a_max=draw_a, dt=dt)
        steps.append(("draw", seg))
        q = seg.q[-1]
        u1, v1 = s[-1]
        seg = plan_line(q, pose(u1, v1, lift_mm), v_max=plunge_v,
                        a_max=travel_a, **line)
        steps.append(("up", seg))
        q = seg.q[-1]
    return steps


def pen_trace(steps, page, tool):
    """(kind, (N, 3) page coordinates u, v, h in mm) per step, from FK of the
    planned joints -- what the pen tip will actually do, not what was asked."""
    out = []
    for kind, payload in steps:
        if kind == "approach":
            continue
        uvh = np.array([page.to_page(fk(q, tool=tool)[:3, 3] * 1000.0)
                        for q in payload.q])
        out.append((kind, uvh))
    return out


def preview_svg(steps, page, tool, path, source=None):
    """Write an SVG of the planned pen path over the page outline: drawn
    strokes solid, pen-up travel dashed. Opens in any browser."""
    w, h = page.width_mm, page.height_mm

    def pts(a):
        return " ".join(f"{u:.2f},{h - v:.2f}" for u, v in a[:, :2])

    body = [f'<rect x="0" y="0" width="{w}" height="{h}" fill="white" '
            f'stroke="#bbb" stroke-width="0.5"/>']
    if source is not None:
        for s in source:
            body.append(f'<polyline points="{pts(s)}" fill="none" stroke="#f5b041" '
                        f'stroke-width="1.6" stroke-opacity="0.5"/>')
    for kind, uvh in pen_trace(steps, page, tool):
        if kind == "draw":
            body.append(f'<polyline points="{pts(uvh)}" fill="none" stroke="#1b2631" '
                        f'stroke-width="0.4"/>')
        elif kind == "travel":
            body.append(f'<polyline points="{pts(uvh)}" fill="none" stroke="#5dade2" '
                        f'stroke-width="0.3" stroke-dasharray="2,2"/>')
    with open(path, "w") as f:
        f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-5 -5 {w + 10} '
                f'{h + 10}" width="{(w + 10) * 3:.0f}" height="{(h + 10) * 3:.0f}">'
                + "".join(body) + "</svg>\n")


def drawing_time(steps, move_joints_s=0.0):
    """Planned seconds of follow() motion, plus a caller's estimate for the
    approach move_joints, which is not planned here."""
    return move_joints_s + sum(p.duration for k, p in steps if k != "approach")
