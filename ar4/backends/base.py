"""The contract every backend implements.

Nothing above this layer knows whether it is driving MuJoCo or a Teensy.
Angles are always degrees, joint order J1..J6, and move_joints blocks
until the motion is complete.
"""
from abc import ABC, abstractmethod


class Backend(ABC):
    @abstractmethod
    def connect(self):
        """Open the connection or load the model. Safe to call once."""

    @abstractmethod
    def disconnect(self):
        """Release the port or viewer. Safe to call twice."""

    @abstractmethod
    def get_joints(self):
        """Current joint angles, degrees, list of 6."""

    @abstractmethod
    def move_joints(self, angles_deg, speed=25):
        """Move to absolute joint angles. Blocks until settled."""

    @abstractmethod
    def follow(self, trajectory):
        """Execute a trajectory of joint waypoints, continuously.

        `trajectory` needs exactly two attributes: `q`, an (N, 6) array of
        joint angles in degrees, and `dt`, the seconds between consecutive
        rows. Nothing else -- backends do not import motion.trajectory.

        Timing comes from the trajectory, so there is no speed argument; plan
        a slower path if you want a slower move.

        Blocks until the motion is complete and the arm is at rest on q[-1],
        and returns the joint angles actually reached, in degrees.

        How the setpoints are tracked is the backend's own business. MuJoCo's
        position servo lags a streamed setpoint and has to lead it; the Teensy
        does its own ramping and will need something different. Callers are
        promised only that the tool follows the planned path and that the arm
        ends at rest on the final waypoint.
        """

    @abstractmethod
    def set_gripper(self, open_frac):
        """0.0 fully closed, 1.0 fully open."""

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()
        return False