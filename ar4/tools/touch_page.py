"""Measure a page by touching the pen to it. Run from ar4/:

    python tools/touch_page.py --port COM5 --pen-mm 40        # real arm
    python tools/touch_page.py                                # simulator

Jog the pen tip onto the paper and record where it is, three or more times,
then fit and save a page.json for tools/draw.py. Record in this order:

    1. the page's origin corner -- the drawing's bottom-left as you want it read
    2. a point along the bottom edge, well away from the origin (sets +u)
    3. anything else on the page; more points, spread out, give a better fit
       and an honest flatness figure

The pen is held pointing down and leaning its tip toward the robot, like it
will while drawing (see motion.draw.choose_pen); `o` restores that. Use the
same pen, in the same grip, that will draw -- then any error in --pen-mm
cancels.

Commands (distances in mm; x/y/z are the robot base frame, x away from it):

    x 10   y -5   z -2     jog the tip
    a 0.5                  along the pen axis (positive = toward the paper)
    o                      re-orient the pen for drawing
    p                      record the tip as the next point
    u                      forget the last point
    w 297 210 page.json    fit, report flatness, save (width, height)
    ?                      this help        q   quit
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from arm import add_arm_args, open_arm, run  # noqa: E402
from motion.draw import Page  # noqa: E402
from motion.kinematics import TOOL_MM, axial_tool, fk  # noqa: E402
from motion.trajectory import plan_line  # noqa: E402

JOG_V_MM_S = 20.0
JOG_A_MM_S2 = 100.0
TILT_DEG = 15.0


def touch_rotation(tip_mm, tilt_deg=TILT_DEG):
    """Pen pointing down, tip leaning toward the base: the orientation
    choose_pen settles on for a page on the table."""
    d = -np.array([tip_mm[0], tip_mm[1], 0.0])
    d = d / np.linalg.norm(d) if np.linalg.norm(d) > 1e-9 else np.array([-1.0, 0.0, 0.0])
    t = math.radians(tilt_deg)
    z = np.array([0.0, 0.0, -math.cos(t)]) + math.sin(t) * d
    x = np.cross([0.0, 0.0, 1.0], d)
    x -= np.dot(x, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


class Jogger:
    def __init__(self, arm, fit, tool, tilt_deg=TILT_DEG, log=print):
        self.arm, self.fit, self.tool = arm, fit, tool
        self.tilt = tilt_deg
        self.points = []
        self.log = log

    def pose(self):
        return fk(np.asarray(self.arm.get_joints()), tool=self.tool)

    def tip_mm(self):
        return self.pose()[:3, 3] * 1000.0

    def _go(self, target):
        traj = plan_line(np.asarray(self.arm.get_joints()), target, tool=self.tool,
                         v_max=JOG_V_MM_S, a_max=JOG_A_MM_S2)
        run(self.arm, self.fit, traj)

    def jog(self, dx=0.0, dy=0.0, dz=0.0):
        target = self.pose()
        target[:3, 3] += np.array([dx, dy, dz]) / 1000.0
        self._go(target)

    def along(self, d):
        target = self.pose()
        target[:3, 3] += target[:3, 2] * d / 1000.0
        self._go(target)

    def orient(self):
        target = self.pose()
        target[:3, :3] = touch_rotation(target[:3, 3] * 1000.0, self.tilt)
        self._go(target)

    def record(self):
        self.points.append(self.tip_mm())
        return self.points[-1]

    def fit_page(self, width, height):
        if len(self.points) < 3:
            raise ValueError(f"need 3 points, have {len(self.points)}")
        return Page.from_touches(self.points[0], self.points[1], self.points[2:],
                                 width, height)

    def command(self, line):
        """Run one command line. Returns False to quit."""
        parts = line.split()
        if not parts:
            return True
        c, args = parts[0].lower(), parts[1:]
        try:
            if c in ("x", "y", "z"):
                d = float(args[0])
                self.jog(**{"d" + c: d})
            elif c == "a":
                self.along(float(args[0]))
            elif c == "o":
                self.orient()
            elif c == "p":
                p = self.record()
                name = {1: "origin", 2: "u-direction"}.get(len(self.points), "extra")
                self.log(f"point {len(self.points)} ({name}): "
                         f"{p[0]:.2f} {p[1]:.2f} {p[2]:.2f}")
                return True
            elif c == "u":
                if self.points:
                    self.points.pop()
                self.log(f"{len(self.points)} points")
                return True
            elif c == "w":
                w, h = float(args[0]), float(args[1])
                out = args[2] if len(args) > 2 else "page.json"
                page = self.fit_page(w, h)
                page.save(out)
                self.log(f"saved {out}: {len(self.points)} points, flat to "
                         f"{page.residual_mm:.2f} mm, normal "
                         f"{np.array2string(page.normal, precision=3)}")
                return True
            elif c == "?":
                self.log(__doc__[__doc__.index("Commands"):])
                return True
            elif c == "q":
                return False
            else:
                self.log(f"unknown command {c!r}; ? for help")
                return True
        except (IndexError, ValueError) as exc:
            self.log(f"{c}: {exc}")
            return True
        except Exception as exc:  # a PlanningError should not end the session
            self.log(f"{c} refused: {type(exc).__name__}: {exc}")
            return True
        t = self.tip_mm()
        self.log(f"tip {t[0]:.2f} {t[1]:.2f} {t[2]:.2f}")
        return True


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--pen-mm", type=float, default=40.0,
                   help="pen tip past the SG1 fingertips, mm")
    p.add_argument("--tilt", type=float, default=TILT_DEG)
    add_arm_args(p)
    args = p.parse_args(argv)

    arm, fit = open_arm(args.port, args.gripper_port, args.home)
    jog = Jogger(arm, fit, axial_tool(TOOL_MM + args.pen_mm), args.tilt)
    print("? for help. Start with o to point the pen down.")
    try:
        while True:
            try:
                line = input("touch> ")
            except EOFError:
                break
            if not jog.command(line):
                break
    finally:
        arm.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
