"""This module provides common enumerations for FRC robots."""

class Direction(object):
    """Enumerates movement directions

    This enumeration is used to describe the direction of an class that
    provides physical movement (e.g., an Arm or Drive Train).  All directions
    are from the perspective of the center of the robot facing towards the
    front.

    Attributes:
        LEFT.
        RIGHT.
        FORWARD.
        BACKWARD.
        UP.
        DOWN.

    """
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    BACKWARD = 4
    UP = 5
    DOWN = 6


class ProgramState(object):
    """Enumerates robot game states.

    This enumeration is used to keep track of the current game state as
    provided by the playing field (FMS).

    Attributes:
        DISABLED.
        AUTONOMOUS.
        TELEOP.

    """
    DISABLED = 1
    AUTONOMOUS = 2
    TELEOP = 3
