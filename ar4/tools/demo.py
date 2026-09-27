"""Vertical pick-and-place against the tool frame. Run from ar4/:

    ../.venv/Scripts/python.exe tools/demo.py                     # simulator
    ../.venv/Scripts/python.exe tools/demo.py --port COM5 --gripper-port COM6 --home

On hardware each move is slowed to fit the Teensy firmware's 60 deg/s and
30 deg/s^2 -- the simulator's planning limits ask for up to 125 deg/s^2 on J5
during a descent -- and one lap runs unless --laps says otherwise. Read the
bring-up list at the bottom of backends/hw.py before the first powered run.

Every pose here is a FINGERTIP pose built by grasp_pose, with the approach
axis pointing straight down at the board -- which is what a chess grasp
actually is. Before the tool frame existed the demo translated whatever
orientation HOME happened to have, and HOME points the gripper 150 degrees
away from down, so the jaws closed on air above the flange.

Squares are 57 mm apart. Loops until you close the window.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from arm import add_arm_args, open_arm
from motion.kinematics import grasp_pose
from motion.ik import ik_multistart
from motion.trajectory import plan_path

SQUARE_MM = 57.0
# Jaw yaw. NOT zero, and that is not cosmetic: with the jaws square to the
# board the wrist sits exactly on a solution-branch boundary at y = 0, so J5
# has to flip sign to cross the centre file and any straight path between the
# two halves must pass through the J5 = 0 singularity. Any yaw of ~30 deg or
# more keeps one branch across the whole board (J5 stays near 34 deg).
# 45 deg also puts the jaws on the square diagonal, where the nearest
# neighbouring piece is 80 mm away instead of 57 mm.
GRASP_YAW = 45.0
BOARD_X = 360.0            # inside the straight-down grasp envelope, x 250..550
GRASP_Z = 30.0             # fingertip height when closed on a piece
CLEAR_Z = 130.0            # safe traverse height above the pieces

OPEN, CLOSE = 1.0, 0.0

PICK_Y = -SQUARE_MM
PLACE_Y = SQUARE_MM


def sequence():
    """(fingertip targets, gripper action once the arm arrives)."""
    above_pick = grasp_pose(BOARD_X, PICK_Y, CLEAR_Z, yaw_deg=GRASP_YAW)
    at_pick = grasp_pose(BOARD_X, PICK_Y, GRASP_Z, yaw_deg=GRASP_YAW)
    above_place = grasp_pose(BOARD_X, PLACE_Y, CLEAR_Z, yaw_deg=GRASP_YAW)
    at_place = grasp_pose(BOARD_X, PLACE_Y, GRASP_Z, yaw_deg=GRASP_YAW)
    return [
        ([above_pick], OPEN),
        ([at_pick], CLOSE),        # descend, then grip
        ([above_pick], None),      # lift
        ([above_place], None),     # traverse at clearance height
        ([at_place], OPEN),        # descend, then release
        ([above_place], None),     # lift
    ]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_arm_args(p)
    p.add_argument("--laps", type=int, default=None,
                   help="stop after this many laps (hardware default 1)")
    return p.parse_args()


def main():
    args = parse_args()
    steps = sequence()
    home = ik_multistart(grasp_pose(BOARD_X, 0.0, CLEAR_Z, yaw_deg=GRASP_YAW),
                         tool=True)
    if home is None:
        sys.exit("home pose is unreachable -- check BOARD_X against the envelope")

    arm, fit = open_arm(args.port, args.gripper_port, args.home)
    laps = args.laps if args.laps is not None else (None if fit is None else 1)

    def running():
        viewer = getattr(arm, "viewer", None)
        return viewer is None or viewer.is_running()

    try:
        arm.move_joints(home)
        if fit is None or args.gripper_port:
            arm.set_gripper(OPEN)
        lap = 0
        while running() and (laps is None or lap < laps):
            q = np.array(arm.get_joints())
            for targets, grip in steps:
                traj = plan_path(q, targets, tool=True)
                if fit is not None:
                    traj = fit(traj)
                q = np.array(arm.follow(traj))
                if grip is not None and (fit is None or args.gripper_port):
                    arm.set_gripper(grip)
            lap += 1
            print(f"lap {lap} done")
    except Exception as exc:
        print(f"stopped: {type(exc).__name__}: {exc}")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
