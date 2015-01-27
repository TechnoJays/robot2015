"""This module describes information about vision targets."""


class Side(object):
    """Enumeration for which side of the wall a target is on."""
    LEFT = 0
    RIGHT = 1
    UNKNOWN = 2
    EITHER = 3


class Target(object):
    """Target information."""
    side = None
    distance = None
    angle = None
    is_hot = False
    confidence = None
    no_targets = False

    def __init__(self, **values):
        """Create a target using a dictionary.

        Args:
            **values: a keyword dictionary with the values.

        """
        if values:
            self.__dict__.update(values)
        else:
            self.side = None
            self.distance = None
            self.angle = None
            self.is_hot = None
            self.confidence = None
            self.no_targets = False
