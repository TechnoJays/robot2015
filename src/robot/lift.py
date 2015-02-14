"""This module provides a (fork)lift class."""

# Imports
import wpilib
import common
import logging
import logging.config
import math
import parameters
import stopwatch


class Lift(object):
    """A mechanism that lifts things off of the ground.

    This class desribes a lift mechanism that uses a metal bracket attached to
    a vertical rail to lift things using their handles.

    Attributes:
        lift_enabled: True if the Lift is fully functional (default False).
        encoder_enabled: true if the lift encoder is present and initialized.

    """

    # Public member variables
    lift_enabled = False
    encoder_enabled = False

    # Private member objects
    _log = None
    _parameters = None
    _lift_controller = None
    _encoder = None
    _movement_timer = None

    # Private parameters
    _encoder_threshold = None
    _auto_medium_encoder_threshold = None
    _auto_far_encoder_threshold = None
    _encoder_max_limit = None
    _encoder_min_limit = None
    _time_threshold = None
    _auto_medium_time_threshold = None
    _auto_far_time_threshold = None
    _auto_far_speed_ratio = None
    _auto_medium_speed_ratio = None
    _auto_near_speed_ratio = None
    _up_direction = None
    _down_direction = None
    _up_speed_ratio = None
    _down_speed_ratio = None

    # Private member variables
    _encoder_count = None
    _log_enabled = False
    _parameters_file = None
    _ignore_encoder_limits = None
    _robot_state = common.ProgramState.DISABLED

    def __init__(self, params="lift.par", logging_enabled=False):
        """Create and initialize a lift.

        Instantiate a lift and specify a parameters file and whether logging
        is enabled or disabled.

        Args:
            params: The parameters filename to use for Lift configuration.
            logging_enabled: True if logging should be enabled.

        """
        self._initialize(params, logging_enabled)

    def dispose(self):
        """Dispose of a lift object.

        Dispose of a lift object when it is no longer required by removing
        references to any internal objects.

        """
        self._log = None
        self._parameters = None
        self._encoder = None
        self._lift_controller = None
        self._movement_timer = None

    def _initialize(self, params, logging_enabled):
        """Initialize and configure a Lift object.

        Initialize instance variables to defaults, read parameter values from
        the specified file, instantiate required objects and update status
        variables.

        Args:
            params: The parameters filename to use for Lift configuration.
            logging_enabled: True if logging should be enabled.

        """
        # Initialize public member variables
        self.encoder_enabled = False
        self.lift_enabled = False

        # Initialize private member objects
        self._log = None
        self._parameters = None
        self._encoder = None
        self._lift_controller = None
        self._movement_timer = None

        # Initialize private parameters
        self._encoder_threshold = 10
        self._auto_medium_encoder_threshold = 50
        self._auto_far_encoder_threshold = 100
        self._encoder_max_limit = 10000
        self._encoder_min_limit = 0
        self._time_threshold = 0.1
        self._auto_medium_time_threshold = 0.5
        self._auto_far_time_threshold = 1.0
        self._auto_far_speed_ratio = 1.0
        self._auto_medium_speed_ratio = 1.0
        self._auto_near_speed_ratio = 1.0
        self._up_direction = 0.1
        self._down_direction = 0.1
        self._up_speed_ratio = 1.0
        self._down_speed_ratio = 1.0

        # Initialize private member variables
        self._encoder_count = 0
        self._ignore_encoder_limits = False
        self._log_enabled = False
        self._robot_state = common.ProgramState.DISABLED

        # Enable logging if specified
        if logging_enabled:
            # Create a new data log object
            self._log = logging.getLogger('lift')
            self._log.setLevel(logging.DEBUG)
            fh = logging.FileHandler('/home/lvuser/log/lift.log')
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
        lift_motor_channel = -1
        encoder_a_channel = -1
        encoder_b_channel = -1
        encoder_reverse = 0
        encoder_type = 2

        # Close and delete old objects
        self._parameters = None
        self._encoder = None
        self._lift_controller = None

        # Read the parameters file
        self._parameters = parameters.Parameters(self._parameters_file)
        section = __name__.lower()

        # Store parameters from the file to local variables
        if self._parameters:
            encoder_a_channel = self._parameters.get_value(section,
                                                "ENCODER_A_CHANNEL")
            encoder_b_channel = self._parameters.get_value(section,
                                                "ENCODER_B_CHANNEL")
            encoder_reverse = self._parameters.get_value(section,
                                                "ENCODER_REVERSE")
            encoder_type = self._parameters.get_value(section,
                                                "ENCODER_TYPE")
            lift_motor_channel = self._parameters.get_value(section,
                                            "LIFT_MOTOR_CHANNEL")
            self._up_direction = self._parameters.get_value(section,
                                            "UP_DIRECTION")
            self._down_direction = self._parameters.get_value(section,
                                            "DOWN_DIRECTION")
            self._up_speed_ratio = self._parameters.get_value(section,
                                                "UP_SPEED_RATIO")
            self._down_speed_ratio = self._parameters.get_value(section,
                                                "DOWN_SPEED_RATIO")
            self._auto_far_speed_ratio = self._parameters.get_value(
                                            section,
                                            "AUTO_FAR_SPEED_RATIO")
            self._auto_medium_speed_ratio = self._parameters.get_value(
                                            section,
                                            "AUTO_MEDIUM_SPEED_RATIO")
            self._auto_near_speed_ratio = self._parameters.get_value(
                                            section,
                                            "AUTO_NEAR_SPEED_RATIO")
            self._encoder_threshold = self._parameters.get_value(section,
                                            "ENCODER_THRESHOLD")
            self._encoder_max_limit = self._parameters.get_value(section,
                                                "ENCODER_MAX_LIMIT")
            self._encoder_min_limit = self._parameters.get_value(section,
                                                "ENCODER_MIN_LIMIT")
            self._time_threshold = self._parameters.get_value(section,
                                            "TIME_THRESHOLD")
            self._auto_medium_time_threshold = self._parameters.get_value(
                                            section,
                                            "AUTO_MEDIUM_TIME_THRESHOLD")
            self._auto_far_time_threshold = self._parameters.get_value(section,
                                            "AUTO_FAR_TIME_THRESHOLD")
            self._auto_medium_encoder_threshold = self._parameters.get_value(
                                            section,
                                            "AUTO_MEDIUM_ENCODER_THRESHOLD")
            self._auto_far_encoder_threshold = self._parameters.get_value(
                                            section,
                                            "AUTO_FAR_ENCODER_THRESHOLD")

        # Create the encoder object if the channel is valid
        self.encoder_enabled = False
        if encoder_a_channel >= 0 and encoder_b_channel >= 0:
            self._encoder = wpilib.Encoder(encoder_a_channel,
                                           encoder_b_channel,
                                           encoder_reverse,
                                           encoder_type)
            if self._encoder:
                self.encoder_enabled = True

        # Create motor controller
        if lift_motor_channel >= 0:
            self._lift_controller = wpilib.Talon(lift_motor_channel)
            self.lift_enabled = True

        if self._log_enabled:
            if self.encoder_enabled:
                self._log.debug("Encoder enabled")
            else:
                self._log.debug("Encoder disabled")
            if self.lift_enabled:
                self._log.debug("Lift enabled")
            else:
                self._log.debug("Lift disabled")

        return True

    def set_robot_state(self, state):
        """Set the current game state of the robot.

        Store the state of the robot/game mode (disabled, teleop, autonomous)
        and perform any actions that are state related.

        Args:
            state: current robot state (ProgramState enum).

        """
        self._robot_state = state

        # Clear the movement time
        if self._movement_timer:
            self._movement_timer.stop()

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
        if self.encoder_enabled:
            self._encoder_count = self._encoder.get()
            wpilib.SmartDashboard.putNumber("Lift Encoder", self._encoder_count)

    def reset_sensors(self):
        """Reset sensor values."""
        if self.encoder_enabled:
            self._encoder.reset()
            self._encoder_count = self._encoder.get()

    def reset_and_start_timer(self):
        """Resets and restarts the timer for time based movement."""
        if self._movement_timer:
            self._movement_timer.stop()
            self._movement_timer.start()

    def get_current_state(self):
        """Return a string containing sensor and status variables.

        Returns:
            A string with the encoder value.
        """
        return 'Encoder: %(enc)3.0f' % {'enc':self._encoder_count}

    def log_current_state(self):
        """Log sensor and status variables."""
        if self._log:
            if self.encoder_enabled:
                self._log.debug("Encoder: " + str(self._encoder_count))

    def set_lift_position(self, position, speed):
        """Sets the lift to a specified position.

        Args:
            position: The desired position in encoder counts.
            speed: The motor speed ratio.

        Returns:
            True when the desired position is reached.

        """
        # Abort if we don't have the encoder or motors
        if not self.encoder_enabled or not self.lift_enabled:
            return True

        movement_direction = 0.0

        # Check the encoder position against the boundaries (if enabled)
        # Check max boundary
        if (not self._ignore_encoder_limits and self._encoder_max_limit > 0 and
            position > self._encoder_count and
            self._encoder_count > self._encoder_max_limit):
            self._lift_controller.set(0, 0)
            return True
        # Check min boundary
        if (not self._ignore_encoder_limits and position < self._encoder_count
            and self._encoder_count < self._encoder_min_limit):
            self._lift_controller.set(0, 0)
            return True

        # Check to see if we've reached the correct position
        if math.fabs(position - self._encoder_count) <= self._encoder_threshold:
            self._lift_controller.set(0, 0)
            return True

        # Continue moving
        if (position - self._encoder_count) < 0:
            direction = (self._down_direction * self._down_speed_ratio)
        else:
            direction = (self._up_direction * self._up_speed_ratio)

        if (math.fabs(position - self._encoder_count) >
            self._auto_far_encoder_threshold):
            movement_direction = (direction * speed *
                                  self._auto_far_speed_ratio)
        elif (math.fabs(position - self._encoder_count) >
            self._auto_medium_encoder_threshold):
            movement_direction = (direction * speed *
                                  self._auto_medium_speed_ratio)
        else:
            movement_direction = (direction  * speed *
                                  self._auto_near_speed_ratio)

        self._lift_controller.set(movement_direction, 0)
        return False

    def lift_time(self, time, direction, speed):
        """Moves the lift for a certain time and speed.

        Args:
            time: the time to move the lift.
            direction: the common.Direction enum in which to move.
            speed: the motor speed ratio.

        Returns:
            True when finished.

        """
        # Abort if we don't have the timer or motors
        if not self._movement_timer or not self.lift_enabled:
            return True

        # Get the timer value since we started moving
        elapsed_time = self._movement_timer.elapsed_time_in_secs()

        # Calculate time left to move
        time_left = time - elapsed_time

        # Check the encoder position against the boundaries (if enabled)
        if self.encoder_enabled:
            # Check max boundary
            if (not self._ignore_encoder_limits and self._encoder_max_limit > 0
                and direction == common.Direction.UP and
                self._encoder_count > self._encoder_max_limit):
                self._lift_controller.set(0, 0)
                return True
            # Check min boundary
            if (not self._ignore_encoder_limits and self._encoder_min_limit > 0
                and direction == common.Direction.DOWN and
                self._encoder_count < self._encoder_min_limit):
                self._lift_controller.set(0, 0)
                return True

        # Check if we've reached the time duration
        if time_left < self._time_threshold or time_left < 0:
            self._lift_controller.set(0, 0)
            self._movement_timer.stop()
            return True
        directional_speed = 0
        if direction == common.Direction.DOWN:
            directional_speed = (self._down_direction * self._down_speed_ratio)
        else:
            directional_speed = (self._up_direction * self._up_speed_ratio)

        if time_left > self._auto_far_time_threshold:
            directional_speed = (directional_speed * speed *
                    self._auto_far_speed_ratio)
        elif time_left > self._auto_medium_time_threshold:
            directional_speed = (directional_speed * speed *
                    self._auto_medium_speed_ratio)
        else:
            directional_speed = (directional_speed * speed *
                    self._auto_near_speed_ratio)

        self._lift_controller.set(directional_speed, 0)
        return False

    def move_lift(self, directional_speed):
        """Moves the lift at a specified speed and direction.

        Args:
            directional_speed: the speed and direction for moving.

        """
        # Abort if the lift is not available
        if not self.lift_enabled:
            return

        # Check the encoder position against the boundaries (if enabled)
        if self.encoder_enabled:
            # Check max boundary
            if (not self._ignore_encoder_limits and self._encoder_max_limit > 0
                and self._up_direction * directional_speed > 0 and
                self._encoder_count > self._encoder_max_limit):
                self._lift_controller.set(0, 0)
                return True
            # Check min boundary
            if (not self._ignore_encoder_limits and self._encoder_min_limit > 0
                and self._down_direction * directional_speed > 0 and
                self._encoder_count < self._encoder_min_limit):
                self._lift_controller.set(0, 0)
                return True

        if self._up_direction * directional_speed > 0:
            directional_speed = directional_speed * self._up_speed_ratio
        else:
            directional_speed = directional_speed * self._down_speed_ratio

        self._lift_controller.set(directional_speed, 0)

    def ignore_encoder_limits(self, state):
        """Notify lift to ignore encoder limits.

        Args:
            state: True if limits should be ignored.

        """
        self._ignore_encoder_limits = state

