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
    def set_gripper(self, open_frac):
        """0.0 fully closed, 1.0 fully open."""

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()
        return False