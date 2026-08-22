"""Vertical pick-and-place against the tool frame. Run from ar4/:

    ../.venv/Scripts/python.exe tools/demo.py

Every pose here is a FINGERTIP pose built by grasp_pose, with the approach
axis pointing straight down at the board -- which is what a chess grasp
actually is. Before the tool frame existed the demo translated whatever
orientation HOME happened to have, and HOME points the gripper 150 degrees
away from down, so the jaws closed on air above the flange.

Squares are 57 mm apart. Loops until you close the window.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from backends.sim import SimBackend
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


def main():
    steps = sequence()
    home = ik_multistart(grasp_pose(BOARD_X, 0.0, CLEAR_Z, yaw_deg=GRASP_YAW),
                         tool=True)
    if home is None:
        sys.exit("home pose is unreachable -- check BOARD_X against the envelope")

    model = str(Path(__file__).resolve().parents[1] / "models" / "ar4.xml")
    arm = SimBackend(model_path=model, render=True, realtime=True)
    arm.connect()
    print("viewer open -- close the window to stop")

    try:
        arm.move_joints(home)
        arm.set_gripper(OPEN)
        lap = 0
        while arm.viewer is not None and arm.viewer.is_running():
            q = np.array(arm.get_joints())
            for targets, grip in steps:
                traj = plan_path(q, targets, tool=True)
                q = np.array(arm.follow(traj))
                if grip is not None:
                    arm.set_gripper(grip)
            lap += 1
            print(f"lap {lap} done")
    except Exception as exc:
        print(f"stopped: {type(exc).__name__}: {exc}")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
