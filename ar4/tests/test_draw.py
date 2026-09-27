"""motion.draw and the drawing tools, against the planner and the fake Teensy.

The property that matters most is asserted directly: the pen tip, computed by
forward kinematics from the planned joints, lies on the requested line at the
requested height. Everything else is in service of that.
"""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from backends.hw import HwBackend, required_slowdown
from fake_teensy import FakeClock, FakeTeensy
from motion import draw
from motion.draw import Page
from motion.ik import ik_multistart
from motion.kinematics import TOOL_MM, axial_tool, fk
from motion.trajectory import Trajectory

TOOLS = Path(__file__).resolve().parents[1] / "tools"
PEN = axial_tool(TOOL_MM + 40.0)
A4 = dict(center_mm=(380.0, 0.0, 0.0), width_mm=297.0, height_mm=210.0)

SVG = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 200 140">
  <rect x="10" y="10" width="60" height="40" fill="none" stroke="black"/>
  <circle cx="130" cy="40" r="30" fill="none" stroke="black"/>
  <g transform="translate(20,80) scale(1.5)">
    <path d="M0 0 C 20 -30, 40 30, 60 0 S 100 -30, 110 0" fill="none" stroke="black"/>
  </g>
  <polyline points="10,130 30,110 50,130 70,110 90,130" fill="none" stroke="black"/>
  <rect x="0" y="0" width="5" height="5" style="display:none"/>
</svg>
"""


def _tool_module(name):
    import sys
    sys.path.insert(0, str(TOOLS))
    spec = importlib.util.spec_from_file_location(f"tool_{name}", TOOLS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _dist_to_polyline(p, poly):
    a, b = poly[:-1], poly[1:]
    ab = b - a
    t = np.clip(np.sum((p - a) * ab, 1) / np.maximum(np.sum(ab * ab, 1), 1e-12), 0, 1)
    return float(np.min(np.linalg.norm(a + ab * t[:, None] - p, axis=1)))


@pytest.fixture(scope="module")
def page():
    return Page.flat(**A4)


@pytest.fixture(scope="module")
def pen(page):
    return draw.choose_pen(page, PEN)


@pytest.fixture(scope="module")
def svg_file(tmp_path_factory):
    p = tmp_path_factory.mktemp("svg") / "art.svg"
    p.write_text(SVG)
    return p


# --- the page -------------------------------------------------------------------

def test_flat_page_round_trips_and_faces_up(page):
    assert np.allclose(page.normal, [0, 0, 1])
    for u, v, h in [(0, 0, 0), (297, 210, 0), (100, 50, 7.5)]:
        assert np.allclose(page.to_page(page.point_mm(u, v, h)), [u, v, h])
    # centred where it was asked to be
    assert np.allclose(page.point_mm(148.5, 105.0), [380, 0, 0])


def test_pen_rotation_is_a_rotation_that_leans_toward_the_base(page):
    r = page.pen_rotation(37.0, 20.0)
    assert np.allclose(r.T @ r, np.eye(3), atol=1e-12)
    assert np.linalg.det(r) == pytest.approx(1.0)
    z = r[:, 2]
    assert math.degrees(math.acos(-z @ page.normal)) == pytest.approx(20.0)
    # tip leans toward the robot: the pen axis has a -x component here
    assert z[0] < 0


def test_touches_recover_a_tilted_page():
    """An easel page, leaning back 60 deg, touched at four corners."""
    truth = Page(origin_mm=[350, 120, 80],
                 rotation=np.column_stack((
                     [0, -1, 0],
                     [math.cos(math.radians(60)), 0, math.sin(math.radians(60))],
                     [-math.sin(math.radians(60)), 0, math.cos(math.radians(60))])),
                 width_mm=200, height_mm=150)
    pts = [truth.point_mm(u, v) for u, v in [(0, 0), (200, 0), (200, 150), (0, 150)]]
    got = Page.from_touches(pts[0], pts[1], pts[2:], 200, 150)
    assert np.allclose(got.rotation, truth.rotation, atol=1e-9)
    assert np.allclose(got.origin_mm, truth.origin_mm, atol=1e-9)
    assert got.residual_mm < 1e-9


def test_touch_flatness_is_reported():
    pts = [[300, 100, 0], [300, -100, 0], [450, -100, 0.0], [450, 100, 0.8]]
    got = Page.from_touches(pts[0], pts[1], pts[2:], 200, 150)
    assert 0.1 < got.residual_mm < 0.8
    assert got.normal[2] > 0.99   # still faces up, toward the shoulder


def test_collinear_touches_refused():
    with pytest.raises(ValueError, match="collinear"):
        Page.from_touches([300, 0, 0], [350, 0, 0], [[400, 0, 0]], 100, 100)


def test_page_json_round_trip(tmp_path, page):
    page.save(tmp_path / "p.json")
    back = Page.load(tmp_path / "p.json")
    assert np.allclose(back.rotation, page.rotation)
    assert np.allclose(back.origin_mm, page.origin_mm)


# --- strokes --------------------------------------------------------------------

def test_load_svg_applies_transforms_and_skips_hidden(svg_file):
    strokes = draw.load_svg(svg_file)
    assert len(strokes) == 4          # hidden rect dropped
    circle = max(strokes, key=lambda s: np.ptp(s[:, 1]) * (np.ptp(s[:, 0]) < 70))
    r = np.linalg.norm(circle - [130, 40], axis=1)
    assert np.allclose(r, 30, atol=0.05)
    curve = next(s for s in strokes if s[0][0] == pytest.approx(20))
    assert np.ptp(curve[:, 0]) == pytest.approx(165, abs=0.01)   # 110 * 1.5


def test_fit_to_page_centres_scales_and_flips(svg_file):
    raw = draw.load_svg(svg_file)
    out = draw.fit_to_page(raw, 297, 210, margin_mm=15)
    pts = np.vstack(out)
    assert pts.min() >= 15 - 1e-9
    assert pts[:, 0].max() <= 297 - 15 + 1e-9 and pts[:, 1].max() <= 210 - 15 + 1e-9
    # aspect preserved: the circle is still a circle
    circle = out[[len(s) for s in raw].index(max(len(s) for s in raw[:2]))]
    assert np.ptp(circle[:, 0]) == pytest.approx(np.ptp(circle[:, 1]), rel=1e-3)
    # SVG y runs down: the rect at the top of the file is at the top of the page
    rect = out[0]
    assert rect[:, 1].min() > 100


def test_simplify():
    line = np.column_stack((np.linspace(0, 100, 200), np.linspace(0, 50, 200)))
    assert len(draw.simplify(line)) == 2
    t = np.linspace(0, 2 * np.pi, 2000)
    circ = np.column_stack((40 * np.cos(t), 40 * np.sin(t)))
    s = draw.simplify(circ, tol=0.05)
    assert len(s) < 200
    dense = np.column_stack((40 * np.cos(t), 40 * np.sin(t)))
    assert max(_dist_to_polyline(p, s) for p in dense) <= 0.05 + 1e-9


def test_order_strokes_cuts_travel_and_keeps_every_stroke():
    # ten dashes along a line, shuffled and half of them reversed
    rng = np.random.default_rng(3)
    dashes = [np.array([[10.0 * i, 0.0], [10.0 * i + 5, 0.0]]) for i in range(10)]
    strokes = [d[::-1] if rng.random() < 0.5 else d
               for d in (dashes[k] for k in rng.permutation(10))]
    ordered = draw.order_strokes(strokes)
    assert draw.travel_mm(ordered) == pytest.approx(45.0)   # 9 gaps of 5 mm
    assert draw.travel_mm(strokes) > 150.0
    key = lambda s: tuple(np.round(s[0], 6)) + tuple(np.round(s[-1], 6))
    before = {frozenset([key(s), key(s[::-1])]) for s in strokes}
    after = {frozenset([key(s), key(s[::-1])]) for s in ordered}
    assert before == after


# --- the arm ----------------------------------------------------------------------

def test_vertical_pen_cannot_clear_the_wrist_singularity(page):
    """The finding that made the pen lean. If this ever starts passing, the
    tilt in choose_pen may no longer be needed."""
    yaw, tilt, worst = draw.choose_pen(page, PEN, tilts=(0,), yaws=range(0, 360, 30))
    assert worst < 5.0


def test_choose_pen_clears_the_target(pen):
    yaw, tilt, worst = pen
    assert worst >= draw.TARGET_J5_DEG
    assert 0 < tilt <= 30


def _start_on(page, pen, uv, h):
    q = ik_multistart(page.pose(uv[0], uv[1], h, pen[:2]), tool=PEN)
    assert q is not None
    return q


def test_stroke_stays_on_the_line_and_slows_for_corners(page, pen):
    # an L with a right-angle corner, then a gentle arc
    arc = [(150 + 40 * math.cos(a), 60 + 40 * math.sin(a))
           for a in np.linspace(math.pi, math.pi / 2, 40)]
    uv = np.array([(60, 60), (110, 60), (110, 100)] + [(110, 100)] + arc[1:])
    uv = uv[np.r_[True, np.linalg.norm(np.diff(uv, axis=0), axis=1) > 1e-9]]
    q0 = _start_on(page, pen, uv[0], -1.0)
    pts = np.array([page.point_mm(u, v, -1.0) for u, v in uv])
    traj = draw.plan_stroke(q0, pts, page.pen_rotation(*pen[:2]), PEN)

    tips = np.array([fk(q, tool=PEN)[:3, 3] * 1000 for q in traj.q])
    uvh = np.array([page.to_page(t) for t in tips])
    assert max(_dist_to_polyline(p, uv) for p in uvh[:, :2]) < 0.02
    assert np.allclose(uvh[:, 2], -1.0, atol=0.02)

    speed = np.linalg.norm(np.diff(tips, axis=0), axis=1) / traj.dt
    assert speed.max() <= draw.DRAW_V_MM_S * 1.02
    # Tangential acceleration within the limit. The 25% allowance is the IK's
    # 0.01 mm tolerance seen through a second difference at dt = 20 ms, which
    # alone is worth ~+/-100 mm/s^2 -- not a profile overshoot. Before the
    # constant-acceleration fix the first waypoint alone read 430.
    s_arc = np.concatenate(([0.0], np.cumsum(speed * traj.dt)))
    assert np.abs(np.diff(s_arc, 2)).max() / traj.dt ** 2 <= 1.25 * draw.DRAW_A_MM_S2
    assert speed[0] < 5 and speed[-1] < 5          # starts and ends at rest
    corner = np.argmin(np.linalg.norm(uvh[:, :2] - [110, 60], axis=1))
    assert speed[max(corner - 1, 0):corner + 1].min() < 10
    # but the arc is not crawled: a 40 mm radius allows the full speed
    on_arc = (uvh[1:, 0] > 125) & (uvh[1:, 1] > 75)
    assert speed[on_arc].max() > 0.9 * draw.DRAW_V_MM_S


def test_plan_drawing_shape_and_page_bounds(page, pen, svg_file):
    strokes = draw.order_strokes([draw.simplify(s) for s in draw.fit_to_page(
        draw.load_svg(svg_file), page.width_mm, page.height_mm)])
    steps = draw.plan_drawing(np.zeros(6), strokes, page, PEN, pen=pen[:2])
    kinds = [k for k, _ in steps]
    assert kinds[0] == "approach"
    assert kinds.count("draw") == len(strokes)
    assert kinds[1:4] == ["down", "draw", "up"]
    # every drawn waypoint on its stroke, at press depth
    trace = [uvh for k, uvh in draw.pen_trace(steps, page, PEN) if k == "draw"]
    for s, uvh in zip(strokes, trace):
        assert max(_dist_to_polyline(p, s) for p in uvh[:, :2]) < 0.02
        assert np.allclose(uvh[:, 2], -draw.PRESS_MM, atol=0.02)
    with pytest.raises(ValueError, match="leaves the page"):
        draw.plan_drawing(np.zeros(6), [np.array([[-5.0, 10.0], [20.0, 10.0]])],
                          page, PEN, pen=pen[:2])


def test_preview_is_written(tmp_path, page, pen):
    steps = draw.plan_drawing(np.zeros(6), [np.array([[50.0, 50.0], [100.0, 80.0]])],
                              page, PEN, pen=pen[:2])
    out = tmp_path / "p.svg"
    draw.preview_svg(steps, page, PEN, out)
    assert out.read_text().count("<polyline") == 1


# --- the tools, on the fake Teensy -------------------------------------------------

def _fake_arm():
    clock = FakeClock()
    arm = HwBackend(transport=FakeTeensy(clock), clock=clock, sleep=clock.sleep)
    arm.connect()
    arm.calibrate()

    def fit(traj):
        return Trajectory(traj.q, traj.dt * required_slowdown(traj.q, traj.dt))
    return arm, fit


def test_draw_tool_runs_on_hardware_and_draws_the_line(page, svg_file):
    tool = _tool_module("draw")
    arm, fit = _fake_arm()
    strokes, steps, pen = tool.prepare(svg_file, page, PEN, q_start=arm.get_joints())
    measured = []
    real = arm.follow

    def follow(traj):
        out = real(traj)
        _, _, meas = arm.last_follow
        measured.append(meas)
        return out
    arm.follow = follow
    tool.execute(arm, fit, steps, log=lambda *_: None)

    drawn = [m for (k, _), m in zip([s for s in steps if s[0] != "approach"], measured)
             if k == "draw"]
    assert len(drawn) == len(strokes)
    worst = 0.0
    for s, meas in zip(strokes, drawn):
        uvh = np.array([page.to_page(fk(q, tool=PEN)[:3, 3] * 1000) for q in meas])
        worst = max(worst, max(_dist_to_polyline(p, s) for p in uvh[:, :2]))
    # measured joints, so this includes the firmware's tracking lag
    assert worst < 0.5, f"pen left the line by {worst:.2f} mm"


def test_touch_page_fits_what_it_touched(tmp_path):
    tool = _tool_module("touch_page")
    arm, fit = _fake_arm()
    start = ik_multistart(Page.flat(**A4).pose(50, 50, 20, (0, 15)), tool=PEN)
    arm.move_joints(start, speed=100)
    log = []
    jog = tool.Jogger(arm, fit, PEN, log=log.append)

    assert jog.command("o")
    z0 = jog.tip_mm()[2]
    for cmd in (f"z {-z0:.4f}", "p", "y -150", "p", "x 100", "p", "y 150", "p"):
        assert jog.command(cmd), cmd
    out = tmp_path / "page.json"
    assert jog.command(f"w 150 100 {out}")
    page = Page.load(out)
    assert page.residual_mm < 0.1
    assert page.normal[2] > 0.999
    # a bad command is reported, not raised, and does not end the session
    assert jog.command("x")
    assert jog.command("zz 3")
    assert not jog.command("q")
