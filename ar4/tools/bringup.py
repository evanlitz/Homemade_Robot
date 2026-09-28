"""First power-up of the real arm, one checked step at a time. Run from ar4/:

    python tools/bringup.py --port COM5 --gripper-port COM6
    python tools/bringup.py --rehearse          # same script, fake Teensy

Turns the checklist at the bottom of backends/hw.py into a guided session.
Every step either passes, or stops and says what to change. Nothing moves
without a prompt first, and 'q' at any prompt stops the session cleanly.

    1. connect    firmware version and model handshake; no motion
    2. home       limit-switch calibration -- every joint moves to its switch
    3. park       after homing the MK5 firmware parks at DH zero, so every
                  joint must read ~0; a joint that does not has a wrong offset
    4. direction  each joint +10 deg and back, slowly, one at a time; the
                  script says where the fingertip should go, you say whether
                  it did; a joint that goes the wrong way has a wrong sign
    5. tracking   one short straight line at each tracking gain, measuring
                  how far the arm actually lags the plan
    6. gripper    open and close, if a gripper port was given

Everything is written to bringup-<date>.json: what was measured, what you
confirmed, and the exact edits to backends/hw.py any failure calls for.
Directions are in the robot base frame: forward is where the arm points at
J1 = 0, left is the robot's left seen from behind it, up is up.
"""
import argparse
import csv
import datetime
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from motion.kinematics import JOINT_LIMITS, fk, fk_all  # noqa: E402
from motion.trajectory import Trajectory, plan_line  # noqa: E402

JOG_DEG = 10.0
JOG_SPEED = 10            # percent of the firmware limits
PARK_TOL_DEG = 1.0        # firmware parks from rest steps to ~0.01 deg
WRIST_TEST_J5 = 30.0      # J4 and J6 are collinear at J5 = 0, so test them here
TRACK_POSE = (0.0, 20.0, -10.0, 0.0, 40.0, 0.0)
TRACK_LINE_MM = (0.0, 60.0, 0.0)
DEFAULT_GAINS = (2.0, 4.0, 8.0)


class Abort(Exception):
    """The operator typed q, or a step failed in a way that makes the next
    one unsafe."""


# --- what a joint move should look like -------------------------------------

def _words(v, names):
    """Largest components of a vector, as words: '31 mm left, 8 mm up'."""
    parts = []
    for k in np.argsort(-np.abs(v)):
        if abs(v[k]) < 3.0 or len(parts) == 2:
            break
        parts.append(f"{abs(v[k]):.0f} mm {names[k][int(v[k] > 0)]}")
    return ", ".join(parts)


_DIRS = (("back", "forward"), ("right", "left"), ("down", "up"))


def describe_jog(q, joint, deg=JOG_DEG):
    """Plain-language prediction of what +deg on `joint` (0-based) does from q.

    Worked out from the kinematics the sim and the planner both use, so a
    mismatch on the real arm means the arm disagrees with the model -- which
    is exactly what this step is looking for.
    """
    q0 = np.asarray(q, dtype=float)
    q1 = q0.copy()
    q1[joint] += deg
    a, b = fk(q0, tool=True), fk(q1, tool=True)
    move = (b[:3, 3] - a[:3, 3]) * 1000.0
    dr = b[:3, :3] @ a[:3, :3].T
    axis = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]])

    if joint == 0:
        sense = "counterclockwise" if axis[2] > 0 else "clockwise"
        return f"the whole arm swings {sense} seen from above; fingertip {_words(move, _DIRS)}"
    if joint in (3, 5):
        # spin about the forearm / flange: say which way, seen from behind
        elbow = fk_all(q0)[2][:3, 3]
        view = a[:3, 3] - elbow
        sense = "clockwise" if axis @ view > 0 else "counterclockwise"
        part = ("the wrist (link 4 onward) spins" if joint == 3
                else "only the flange and gripper spin")
        return (f"{part} {sense}, seen from behind the elbow looking "
                f"toward the gripper")
    return f"the fingertip moves {_words(move, _DIRS)}"


# --- the session -------------------------------------------------------------

class Bringup:
    def __init__(self, arm, ask=input, log=print, gains=DEFAULT_GAINS,
                 out_dir=Path(".")):
        self.arm, self._ask, self.log = arm, ask, log
        self.gains = tuple(gains)
        self.out_dir = Path(out_dir)
        self.report = {"started": datetime.datetime.now().isoformat(timespec="seconds"),
                       "model": getattr(arm, "model", None),
                       "steps": {}, "code_changes": []}

    def ask(self, prompt):
        a = self._ask(prompt + " ").strip().lower()
        if a == "q":
            raise Abort("stopped by operator")
        return a

    def confirm(self, prompt):
        """y/n; anything but y is no."""
        return self.ask(prompt + " [y/n/q]") == "y"

    def safe_to_move(self, what):
        if not self.confirm(f"about to {what}. Arm clear, e-stop in hand?"):
            raise Abort(f"not cleared to {what}")

    def change(self, text):
        self.report["code_changes"].append(text)
        self.log(f"  -> CHANGE: {text}")

    # 1
    def connect(self):
        self.arm.connect()
        self.report["steps"]["connect"] = {"ok": True}
        self.log(f"connected: firmware accepted model {self.arm.model!r}")

    # 2
    def home(self):
        self.safe_to_move("home every joint against its limit switch")
        self.arm.calibrate()
        self.report["steps"]["home"] = {"ok": True}
        self.log("homed")

    # 3
    def park(self):
        q = np.array(self.arm.get_joints())
        bad = np.flatnonzero(np.abs(q) > PARK_TOL_DEG)
        rec = {"reading_deg": q.round(3).tolist(), "ok": not len(bad)}
        self.log("after homing, joints read " + " ".join(f"J{i + 1} {v:+.2f}"
                                                         for i, v in enumerate(q)))
        for j in bad:
            # parked at true zero, dh = s * (fw + off)  =>  off_true = off - q/s
            fixed = self.arm.offsets[j] - q[j] / self.arm.signs[j]
            self.change(f"J{j + 1} reads {q[j]:+.2f} deg at park: offset "
                        f"{self.arm.offsets[j]:.2f} -> {fixed:.2f} in "
                        f"JOINT_OFFSETS_BY_MODEL[{self.arm.model!r}]")
        rec["looks_parked"] = self.confirm(
            "does the arm stand in the zero pose (upper arm vertical, forearm "
            "level and pointing forward, gripper pointing forward)?")
        if not rec["looks_parked"]:
            self.log("  the offsets may all be consistent with a wrong reference; "
                     "stop here and compare with the sim before moving further")
        self.report["steps"]["park"] = rec
        if len(bad) or not rec["looks_parked"]:
            raise Abort("park check failed; fix the offsets before any motion")

    def _jog(self, j, base):
        target = np.array(base, dtype=float)
        target[j] += JOG_DEG
        lo, hi = JOINT_LIMITS[j]
        if not lo <= target[j] <= hi:
            target[j] = base[j] - JOG_DEG     # never jog into a limit
        expect = describe_jog(base, j, target[j] - base[j])
        sign = "+" if target[j] > base[j] else "-"
        self.log(f"\nJ{j + 1} {sign}{JOG_DEG:.0f} deg: expect {expect}")
        self.ask("Enter to move (q to stop)")
        self.arm.move_joints(target, speed=JOG_SPEED)
        ok = self.confirm(f"did J{j + 1} do that?")
        self.arm.move_joints(base, speed=JOG_SPEED)
        if not ok:
            self.change(f"J{j + 1} turns the wrong way: JOINT_SIGNS[{j}] "
                        f"{self.arm.signs[j]:+.0f} -> {-self.arm.signs[j]:+.0f}")
        return {"expected": expect, "confirmed": ok}

    # 4
    def directions(self):
        self.safe_to_move(f"move each joint {JOG_DEG:.0f} deg and back, slowly")
        res = {}
        park = np.zeros(6)
        for j in (0, 1, 2, 4):
            res[f"J{j + 1}"] = self._jog(j, park)
        wrist = park.copy()
        wrist[4] = WRIST_TEST_J5
        self.log(f"\nmoving J5 to {WRIST_TEST_J5:.0f} deg: J4 and J6 spin about "
                 f"the same axis at J5 = 0, so they are told apart from here")
        self.arm.move_joints(wrist, speed=JOG_SPEED)
        for j in (3, 5):
            res[f"J{j + 1}"] = self._jog(j, wrist)
        self.arm.move_joints(park, speed=JOG_SPEED)
        self.report["steps"]["direction"] = res
        if not all(r["confirmed"] for r in res.values()):
            raise Abort("a joint turns the wrong way; fix JOINT_SIGNS before "
                        "running any Cartesian motion")

    # 5
    def tracking(self, fit):
        self.safe_to_move("run a 60 mm straight line, back and forth, per gain")
        self.arm.move_joints(TRACK_POSE, speed=25)
        start = fk(np.array(TRACK_POSE), tool=True)
        end = start.copy()
        end[:3, 3] += np.array(TRACK_LINE_MM) / 1000.0
        rows, results = [], {}
        original = self.arm.tracking_gain
        try:
            for gain in self.gains:
                self.arm.tracking_gain = gain
                worst = 0.0
                for a, b in ((start, end), (end, start)):
                    q = np.array(self.arm.get_joints())
                    traj = fit(plan_line(q, b, tool=True))
                    self.arm.follow(traj)
                    t, cmd, meas = self.arm.last_follow
                    for ti, c, m in zip(t, cmd, meas):
                        err = np.linalg.norm(fk(c, tool=True)[:3, 3]
                                             - fk(m, tool=True)[:3, 3]) * 1000.0
                        worst = max(worst, err)
                        rows.append([gain, round(ti, 4), round(err, 4),
                                     *np.round(c, 4), *np.round(m, 4)])
                results[gain] = worst
                self.log(f"  gain {gain:g}: worst fingertip lag {worst:.2f} mm")
        finally:
            self.arm.tracking_gain = original

        csv_path = self.out_dir / "bringup-tracking.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["gain", "t_s", "tip_err_mm"]
                       + [f"cmd_j{i}" for i in range(1, 7)]
                       + [f"meas_j{i}" for i in range(1, 7)])
            w.writerows(rows)
        best = min(results, key=results.get)
        self.report["steps"]["tracking"] = {
            "worst_tip_mm_by_gain": {str(k): round(v, 3) for k, v in results.items()},
            "csv": str(csv_path)}
        if best != original and results[best] < 0.8 * results.get(original, np.inf):
            self.change(f"TRACKING_GAIN {original:g} -> {best:g} "
                        f"({results[best]:.2f} mm worst lag)")
        if results[best] > 5.0:
            self.log("  WARNING: even the best gain lags by more than the 5 mm "
                     "budget; slow the drawing speed before drawing")
        self.arm.move_joints(np.zeros(6), speed=25)

    # 6
    def gripper(self):
        self.arm.set_gripper(1.0)
        opened = self.confirm("did the jaws open?")
        self.arm.set_gripper(0.0)
        closed = self.confirm("did the jaws close?")
        self.report["steps"]["gripper"] = {"opened": opened, "closed": closed}
        if not (opened and closed):
            self.change("gripper: check SERVO_OPEN_DEG / SERVO_CLOSED_DEG in "
                        "backends/hw.py against the SG1's travel")

    def write(self):
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        path = self.out_dir / f"bringup-{stamp}.json"
        self.report["finished"] = datetime.datetime.now().isoformat(timespec="seconds")
        with open(path, "w") as f:
            json.dump(self.report, f, indent=2)
        return path

    def run(self, fit, gripper=False):
        steps = [("connect", self.connect), ("home", self.home),
                 ("park", self.park), ("direction", self.directions),
                 ("tracking", lambda: self.tracking(fit))]
        if gripper:
            steps.append(("gripper", self.gripper))
        try:
            for name, step in steps:
                self.log(f"\n=== {name} ===")
                step()
            self.report["result"] = "passed"
        except Abort as exc:
            self.report["result"] = f"stopped: {exc}"
            self.log(f"\nSTOPPED: {exc}")
        except Exception as exc:
            self.report["result"] = f"error: {type(exc).__name__}: {exc}"
            self.log(f"\nERROR: {type(exc).__name__}: {exc}")
        finally:
            self.arm.disconnect()
        path = self.write()
        self.log(f"\nreport: {path}")
        if self.report["code_changes"]:
            self.log("changes to make:\n  " + "\n  ".join(self.report["code_changes"]))
        return self.report


def rehearsal_arm(gripper=True):
    """HwBackend on the fake Teensy, on a simulated clock: the whole session
    runs instantly with nothing attached, answers and all."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
    from fake_teensy import FakeClock, FakeNano, FakeTeensy
    from backends.hw import HwBackend
    clock = FakeClock()
    return HwBackend(transport=FakeTeensy(clock),
                     gripper_transport=FakeNano() if gripper else None,
                     clock=clock, sleep=clock.sleep)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", help="Teensy serial port")
    p.add_argument("--gripper-port", help="Arduino Nano serial port")
    p.add_argument("--model", default="mk5")
    p.add_argument("--gains", default=",".join(f"{g:g}" for g in DEFAULT_GAINS),
                   help="tracking gains to compare, comma-separated")
    p.add_argument("--rehearse", action="store_true",
                   help="run against a simulated Teensy; nothing is attached")
    args = p.parse_args(argv)
    if not (args.port or args.rehearse):
        p.error("give --port, or --rehearse to practise without the arm")

    from backends.hw import HwBackend, required_slowdown
    if args.rehearse:
        arm = rehearsal_arm(gripper=True)
        gripper = True
        print("REHEARSAL: fake Teensy, nothing attached. The tracking numbers "
              "come from a fake with no serial lag and say nothing about the "
              "real arm.")
    else:
        arm = HwBackend(port=args.port, gripper_port=args.gripper_port,
                        model=args.model)
        gripper = args.gripper_port is not None

    def fit(traj):
        return Trajectory(traj.q, traj.dt * required_slowdown(traj.q, traj.dt))

    gains = [float(g) for g in args.gains.split(",") if g]
    report = Bringup(arm, gains=gains).run(fit, gripper=gripper)
    return 0 if report["result"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
