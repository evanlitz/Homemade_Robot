"""Draw an SVG with a pen held in the SG1. Run from ar4/:

    python tools/draw.py art.svg                       # plan + preview only
    python tools/draw.py art.svg --sim                 # and run it in MuJoCo
    python tools/draw.py art.svg --page page.json --port COM5 --gripper-port COM6

Without --sim or --port nothing moves: the drawing is planned end to end and
written to <art>.preview.svg -- the pen path from forward kinematics of the
planned joints, drawn over the input. Look at it before running anything.

The page comes from tools/touch_page.py (a page.json fitted to touched
points). Without --page, a flat page on the table in front of the robot is
assumed, which is right for the simulator and wrong for any real table.

--pen-mm is how far the pen tip sticks out past the SG1 fingertips. It does not
need to be exact provided the page was touched off with the same pen in the
same grip; the error lands in the page height and cancels.

Hardware needs a compliant pen holder (or a spring-loaded pen): the pen is
driven PRESS_MM below the touched surface to guarantee contact. See
motion/draw.py.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from arm import add_arm_args, open_arm, run  # noqa: E402
from motion import draw  # noqa: E402
from motion.kinematics import TOOL_MM, axial_tool  # noqa: E402

DEFAULT_PEN_MM = 40.0
# A4 landscape on the table, centred where the pen has the most wrist
# clearance -- see choose_pen. Only a stand-in until touch_page has run.
DEFAULT_PAGE = dict(center_mm=(380.0, 0.0, 0.0), width_mm=297.0, height_mm=210.0)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("svg")
    p.add_argument("--page", help="page.json from tools/touch_page.py")
    p.add_argument("--pen-mm", type=float, default=DEFAULT_PEN_MM,
                   help="pen tip past the SG1 fingertips, mm (default %(default)s)")
    p.add_argument("--margin", type=float, default=15.0, help="page margin, mm")
    p.add_argument("--speed", type=float, default=draw.DRAW_V_MM_S,
                   help="drawing speed, mm/s (default %(default)s)")
    p.add_argument("--press", type=float, default=draw.PRESS_MM,
                   help="mm below the touched surface while drawing "
                        "(default %(default)s; 0 for a rigid pen mount)")
    p.add_argument("--preview", help="preview SVG path (default <svg>.preview.svg)")
    p.add_argument("--sim", action="store_true", help="run it in the MuJoCo viewer")
    p.add_argument("--grip", action="store_true",
                   help="close the gripper on the pen before starting")
    add_arm_args(p)
    return p.parse_args(argv)


def prepare(svg, page, tool, margin=15.0, speed=draw.DRAW_V_MM_S,
            press=draw.PRESS_MM, q_start=None):
    """SVG file -> (strokes in page mm, plan steps, pen (yaw, tilt, |J5|))."""
    strokes = draw.load_svg(svg)
    if not strokes:
        raise SystemExit(f"{svg}: nothing to draw (text must be converted to paths)")
    strokes = draw.fit_to_page(strokes, page.width_mm, page.height_mm, margin)
    strokes = [s for s in (draw.simplify(s) for s in strokes) if len(s) > 1]
    strokes = draw.order_strokes(strokes)
    pen = draw.choose_pen(page, tool)
    q0 = np.zeros(6) if q_start is None else np.asarray(q_start, dtype=float)
    steps = draw.plan_drawing(q0, strokes, page, tool, pen=pen[:2],
                              press_mm=press, draw_v=speed)
    return strokes, steps, pen


def execute(arm, fit, steps, log=print):
    """Run planned steps on a backend. The approach is a joint move; every
    other step is a Trajectory, fitted to the hardware when there is one."""
    n = sum(1 for k, _ in steps if k == "draw")
    done = 0
    for kind, payload in steps:
        if kind == "approach":
            arm.move_joints(payload)
        else:
            run(arm, fit, payload)
            if kind == "draw":
                done += 1
                log(f"stroke {done}/{n}")


def main(argv=None):
    args = parse_args(argv)
    page = (draw.Page.load(args.page) if args.page
            else draw.Page.flat(**DEFAULT_PAGE))
    if page.residual_mm > args.press:
        print(f"WARNING: the touched page is {page.residual_mm:.2f} mm out of flat, "
              f"more than the {args.press:.2f} mm press depth; expect gaps")
    tool = axial_tool(TOOL_MM + args.pen_mm)

    strokes, steps, pen = prepare(args.svg, page, tool, args.margin, args.speed,
                                  args.press)
    preview = args.preview or str(Path(args.svg).with_suffix(".preview.svg"))
    draw.preview_svg(steps, page, tool, preview, source=strokes)
    print(f"{len(strokes)} strokes, {draw.travel_mm(strokes):.0f} mm pen-up travel")
    print(f"pen yaw {pen[0]:.0f} deg, tilt {pen[1]:.0f} deg, worst |J5| {pen[2]:.1f} deg")
    print(f"{draw.drawing_time(steps):.0f} s of planned motion "
          f"(longer on hardware: the firmware limits stretch it)")
    print(f"preview: {preview}")

    if not (args.sim or args.port):
        return 0

    arm, fit = open_arm(args.port, args.gripper_port, args.home)
    try:
        if args.grip:
            input("put the pen in the jaws, then press Enter to close them ")
            arm.set_gripper(0.0)
        if args.port:
            input("pen above the page and e-stop in hand? Enter to draw ")
        # replan from where the arm actually is, so the approach starts there
        _, steps, _ = prepare(args.svg, page, tool, args.margin, args.speed,
                              args.press, q_start=arm.get_joints())
        execute(arm, fit, steps)
        print("done")
    finally:
        arm.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
