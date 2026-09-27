"""Open the simulator or the real arm, the same way for every tool script.

    arm, fit = open_arm(port=None)            # MuJoCo viewer
    arm, fit = open_arm(port="COM5", home=True)

`fit` is None for the simulator. On hardware it stretches a Trajectory's time
-- never its path -- just enough to fit the Teensy firmware's speed and
acceleration limits; follow() refuses anything that does not. Pass every
trajectory through it: `arm.follow(fit(traj) if fit else traj)`.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motion.trajectory import Trajectory  # noqa: E402


def add_arm_args(parser):
    parser.add_argument("--port", help="Teensy serial port; omit for the simulator")
    parser.add_argument("--gripper-port", help="Arduino Nano serial port for the SG1")
    parser.add_argument("--home", action="store_true",
                        help="run limit-switch homing first (needed after power-on)")


def open_arm(port=None, gripper_port=None, home=False, render=True):
    if port is None:
        from backends.sim import SimBackend
        arm = SimBackend(model_path=str(ROOT / "models" / "ar4.xml"),
                         render=render, realtime=render)
        arm.connect()
        if render:
            print("viewer open -- close the window to stop")
        return arm, None

    from backends.hw import HwBackend, required_slowdown
    arm = HwBackend(port=port, gripper_port=gripper_port,
                    assume_calibrated=not home)
    arm.connect()
    if home:
        print("homing -- the arm will move to every limit switch")
        arm.calibrate()

    def fit(traj):
        return Trajectory(traj.q, traj.dt * required_slowdown(traj.q, traj.dt))
    return arm, fit


def run(arm, fit, traj):
    """follow() with the hardware fit applied when there is one."""
    return arm.follow(fit(traj) if fit is not None else traj)
