"""This module provides a feeder class."""

# Imports
import wpilib
import common
import feeder_arm
import logging
import logging.config
import parameters
import stopwatch


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
    arms_control_enabled = False

    # Private member objects
    _log = None
    _parameters = None
    _left_arm = None
    _right_arm = None
    _arms_controller = None
    _movement_timer = None

    # Private parameters
    _time_threshold = 0
    _open_direction = None
    _close_direction = None
    _open_speed_ratio = None
    _close_speed_ratio = None

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
        self._arms_controller = None
        self._movement_timer = None

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
        self.arms_control_enabled = False

        # Initialize private member objects
        self._log = None
        self._parameters = None
        self._left_arm = None
        self._right_arm = None
        self._arms_controller = None
        self._movement_timer = None

        # Initialize private parameters
        self._time_threshold = 0.1
        self._open_direction = 1.0
        self._close_direction = -1.0
        self._open_speed_ratio = 1.0
        self._close_speed_ratio = 1.0

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

        self._movement_timer = stopwatch.Stopwatch()

        # Read parameters file
        self._parameters_file = params
        self.load_parameters()

    def load_parameters(self):
        """Load values from a parameter file and create and initialize objects.

        Read parameter values from the specified file, instantiate required
        objects, and update status variables.

        Returns:
            True if the parameter file was processed successfully.

        """
        # Define and initialize local variables
        motor_channel = -1

        # Close and delete old objects
        self._parameters = None
        self._arms_controller = None

        # Read the parameters file
        self._parameters = parameters.Parameters(self._parameters_file)
        section = __name__.lower()

        # Store parameters from the file to local variables
        if self._parameters:
            motor_channel = self._parameters.get_value(section,
                                            "MOTOR_CHANNEL")
            self._time_threshold = self._parameters.get_value(section,
                                            "TIME_THRESHOLD")
            self._open_direction = self._parameters.get_value(section,
                                            "OPEN_DIRECTION")
            self._close_direction = self._parameters.get_value(
                                            section,
                                            "CLOSE_DIRECTION")
            self._open_speed_ratio = self._parameters.get_value(section,
                                            "OPEN_SPEED_RATIO")
            self._close_speed_ratio = self._parameters.get_value(
                                            section,
                                            "CLOSE_SPEED_RATIO")

        # Create motor controllers
        if motor_channel >= 0:
            self._arms_controller = wpilib.Talon(motor_channel)
            self.arms_control_enabled = True

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
            if self.arms_control_enabled:
                self._log.debug("Arms control enabled")
            else:
                self._log.debug("Arms control disabled")

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

    def reset_and_start_timer(self):
        """Resets and restarts the timer for time based movement."""
        if self._movement_timer:
            self._movement_timer.stop()
            self._movement_timer.start()
        if self._right_arm:
            self._right_arm.reset_and_start_timer()
        if self._left_arm:
            self._left_arm.reset_and_start_timer()

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
            elif direction == common.Direction.CLOCKWISE:
                self._right_arm.spin(common.Direction.CLOCKWISE, speed)
                self._left_arm.spin(common.Direction.CLOCKWISE, speed)
            elif direction == common.Direction.COUNTERCLOCKWISE:
                self._right_arm.spin(common.Direction.COUNTERCLOCKWISE, speed)
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
        elif direction == common.Direction.CLOCKWISE:
            right_direction = common.Direction.CLOCKWISE
            left_direction = common.Direction.CLOCKWISE
        elif direction == common.Direction.COUNTERCLOCKWISE:
            right_direction = common.Direction.COUNTERCLOCKWISE
            left_direction = common.Direction.COUNTERCLOCKWISE
        elif direction == common.Direction.STOP:
            right_direction = common.Direction.STOP
            left_direction = common.Direction.STOP

        right_finished = self._right_arm.spin_time(time, right_direction, speed)
        left_finished = self._left_arm.spin_time(time, left_direction, speed)

        if right_finished and left_finished:
            return True

        return False

    def move_arms(self, direction, speed):
        """Move the feeder arms.

        Args:
            direction: Direction enumeration of open, close, stop.
            speed: The speed that the arms move.

        """
        if self.arms_control_enabled:
            if direction == common.Direction.OPEN:
                self._arms_controller.set((self._open_direction * speed *
                    self._open_speed_ratio), 0)
            elif direction == common.Direction.CLOSE:
                self._arms_controller.set((self._close_direction * speed *
                    self._close_speed_ratio), 0)
            elif direction == common.Direction.STOP:
                self._arms_controller.set(0.0, 0)

    def arms_time(self, time, direction, speed):
        """Controls the arms for a time duration.

        Using a timer, moves the arms for a certain time duration.

        Args:
            time: the amount of time to move.
            direction: the direction to move.
            speed: the motor speed ratio.

        Returns:
            True when the time duration has been reached.
        """
        # Abort if arm controller or timer is not available
        if not self.arms_control_enabled or not self._movement_timer:
            return True

        # Get the timer value since we started moving
        elapsed_time = self._movement_timer.elapsed_time_in_secs()

        # Calculate time left
        time_left = time - elapsed_time

        # Check if we've moved long enough
        if time_left < self._time_threshold or time_left < 0:
            self._arms_controller.set(0.0, 0)
            self._movement_timer.stop()
            return True
        else:
            if direction == common.Direction.OPEN:
                self._arms_controller.set((self._open_direction * speed *
                                            self._open_speed_ratio), 0)
            elif direction == common.Direction.CLOSE:
                self._arms_controller.set((self._close_direction * speed *
                                           self._close_speed_ratio), 0)
            else:
                self._arms_controller.set(0.0, 0)
                self._movement_timer.stop()
                return True

        return False
