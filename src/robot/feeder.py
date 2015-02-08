"""This module provides a feeder class."""

# Imports
import common
import feeder_arm
import logging
import logging.config


class Feeder(object):
    """A mechanism that pulls objects into the robot.

    This class desribes a feeder mechanism that uses two arms with spinning
    arms to pull objects into the robot.

    Attributes:
        feeder_enabled: True if the feeder is fully functional (default False).
        left_arm_enabled: True if the left arm is functional (default False).
        right_arm_enabled: True if the right arm is functional (default False).

    """
    # Public member variables
    feeder_enabled = False
    left_arm_enabled = False
    right_arm_enabled = False

    # Private member objects
    _log = None
    _parameters = None
    _left_arm = None
    _right_arm = None

    # Private parameters

    # Private member variables
    _log_enabled = False
    _parameters_file = None
    _robot_state = common.ProgramState.DISABLED

    def __init__(self, params="/home/lvuser/par/feeder.par",
                 logging_enabled=False):
        """Create and initialize a feeder.

        Instantiate a feeder and specify a parameters file and whether logging
        is enabled or disabled.

        Args:
            params: The parameters filename to use for Feeder configuration.
            logging_enabled: True if logging should be enabled.

        """
        self._initialize(params, logging_enabled)

    def dispose(self):
        """Dispose of a feeder object.

        Dispose of a feeder object when it is no longer required by removing
        references to any internal objects.

        """
        self._log = None
        self._parameters = None
        self._left_arm = None
        self._right_arm = None

    def _initialize(self, params, logging_enabled):
        """Initialize and configure a Feeder object.

        Initialize instance variables to defaults, read parameter values from
        the specified file, instantiate required objects (left arm, right arm,
        etc), and update status variables.

        Args:
            params: The parameters filename to use for Feeder configuration.
            logging_enabled: True if logging should be enabled.

        """
        # Initialize public member variables
        self.feeder_enabled = False
        self.left_arm_enabled = False
        self.right_arm_enabled = False

        # Initialize private member objects
        self._log = None
        self._parameters = None
        self._left_arm = None
        self._right_arm = None

        # Initialize private parameters

        # Initialize private member variables
        self._log_enabled = False
        self._robot_state = common.ProgramState.DISABLED
        self._parameters_file = None

        # Enable logging if specified
        if logging_enabled:
            # Create a new data log object
            self._log = logging.getLogger('feeder')
            self._log.setLevel(logging.DEBUG)
            fh = logging.FileHandler('/home/lvuser/log/feeder.log')
            fh.setLevel(logging.DEBUG)
            formatter = logging.Formatter(
                    '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            fh.setFormatter(formatter)
            self._log.addHandler(fh)

            if self._log:
                self._log_enabled = True
            else:
                self._log = None

        # Read parameters file
        self._parameters_file = params
        self.load_parameters()

        # Create the motor controller objects if the channels are greater than 0
        self.right_arm_enabled = False
        self.left_arm_enabled = False
        self._right_arm = feeder_arm.FeederArm("/home/lvuser/par/right_arm.par",
                                               self._log_enabled)
        self._left_arm = feeder_arm.FeederArm("/home/lvuser/par/left_arm.par",
                                              self._log_enabled)
        if self._right_arm and self._right_arm.arm_enabled:
            self.right_arm_enabled = True
        if self._left_arm and self._left_arm.arm_enabled:
            self.left_arm_enabled = True

        # If the arms are enabled, the feeder is fully functional
        self.feeder_enabled = False
        if self.left_arm_enabled and self.right_arm_enabled:
            self.feeder_enabled = True

        if self._log_enabled:
            if self.feeder_enabled:
                self._log.debug("Feeder enabled")
            else:
                self._log.debug("Feeder disabled")

    def load_parameters(self):
        """Load values from a parameter file and create and initialize objects.

        Read parameter values from the specified file, instantiate required
        objects, and update status variables.

        Returns:
            True if the parameter file was processed successfully.

        """
        return True

    def set_robot_state(self, state):
        """Set the current game state of the robot.

        Store the state of the robot/game mode (disabled, teleop, autonomous)
        and perform any actions that are state related.

        Args:
            state: current robot state (ProgramState enum).

        """
        self._robot_state = state

        if state == common.ProgramState.DISABLED:
            pass
        if state == common.ProgramState.TELEOP:
            pass
        if state == common.ProgramState.AUTONOMOUS:
            pass

    def set_log_state(self, state):
        """Set the logging state for this object.

        Args:
            state: True if logging should be enabled.

        """
        if state and self._log:
            self._log_enabled = True
        else:
            self._log_enabled = False

    def read_sensors(self):
        """Read and store current sensor values."""
        pass

    def reset_sensors(self):
        """Reset sensors."""
        pass

    def get_current_state(self):
        """Return a string containing sensor and status variables.

        Returns:
            ''
        """
        return ''

    def log_current_state(self):
        """Log sensor and status variables."""
        pass

    def feed(self, direction, speed):
        """Use the arms to feed an object into the robot.

        Args:
            direction: Direction enumeration of in, out, stop.
            speed: The speed that you feed.

        """
        if self.feeder_enabled:
            if direction == common.Direction.IN:
                self._right_arm.spin(common.Direction.COUNTERCLOCKWISE, speed)
                self._left_arm.spin(common.Direction.CLOCKWISE, speed)
            elif direction == common.Direction.OUT:
                self._right_arm.spin(common.Direction.CLOCKWISE, speed)
                self._left_arm.spin(common.Direction.COUNTERCLOCKWISE, speed)
            elif direction == common.Direction.STOP:
                self._right_arm.spin(common.Direction.STOP, 0)
                self._left_arm.spin(common.Direction.STOP, 0)

    def feed_time(self, time, direction, speed):
        """Controls the feeder arms for a time duration.

        Using a timer, spins the feeder arms in or out for a certain time
        duration.

        Args:
            time: the amount of time to feed.
            direction: the direction to feed.
            speed: the motor speed ratio.

        Returns:
            True when the time duration has been reached.
        """
        # Abort if feeder is not available
        if not self.feeder_enabled:
            return True

        left_direction = None
        right_direction = None
        if direction == common.Direction.IN:
            right_direction = common.Direction.COUNTERCLOCKWISE
            left_direction = common.Direction.CLOCKWISE
        elif direction == common.Direction.OUT:
            right_direction = common.Direction.CLOCKWISE
            left_direction = common.Direction.COUNTERCLOCKWISE
        elif direction == common.Direction.STOP:
            right_direction = common.Direction.STOP
            left_direction = common.Direction.STOP

        right_finished = self._right_arm.spin_time(time, direction, speed)
        left_finished = self._left_arm.spin_time(time, direction, speed)

        if right_finished and left_finished:
            return True

        return False

