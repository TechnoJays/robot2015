"""This module describes a catapult."""

# Imports

# If wpilib not available use pyfrc
try:
    import wpilib
except ImportError:
    from pyfrc import wpilib
import common
import datalog
import math
import parameters
import stopwatch


class Shooter(object):
    """Controls catapult shooting mechanisms.

    Provides a simple interface to shoot an exercise ball. Main functions
    are: move_shooter, shoot and set_shooter_position.

    Hardware: 2 Motor Controllers and 1 Encoder

    Attributes:
        encoder_enabled: true if the shooter encoder is present and initialized.
        shooter_enabled: true if the shooter is present and initialized.

    """
    # Public member variables
    encoder_enabled = None
    shooter_enabled = None

    # Private member objects
    _encoder = None
    _left_shooter_controller = None
    _right_shooter_controller = None
    _log = None
    _parameters = None
    _timer = None

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
    _invert_left_shooter_motor = None
    _invert_right_shooter_motor = None
    _shooter_up_direction = None
    _shooter_down_direction = None
    _normal_up_speed_ratio = None
    _normal_down_speed_ratio = None
    _alternate_up_speed_ratio = None
    _alternate_down_speed_ratio = None

    # Private member variables
    _encoder_count = None
    _log_enabled = None
    _parameters_file = None
    _ignore_encoder_limits = None
    _left_shooter_controller_enabled = False
    _right_shooter_controller_enabled = False
    _robot_state = None

    def __init__(self, params="shooter.par", logging_enabled=False):
        """Create and initialize shooter.

        Initialize default values and read parameters from the parameter file.

        Args:
            params: The parameters filename to use for Shooter configuration.
            logging_enabled: True if logging should be enabled.

        """
        self._initialize(params, logging_enabled)

    def dispose(self):
        """Dispose of a shooter object.

        Dispose of a shooter object when it is no longer required by closing an
        open log file if it exists, and removing references to any internal
        objects.

        """
        if self._log:
            self._log.close()
        self._log = None
        self._parameters = None
        self._encoder = None
        self._left_shooter_controller = None
        self._right_shooter_controller = None
        self._timer = None

    def _initialize(self, params, logging_enabled):
        """Initialize and configure a Shooter object.

        Initialize instance variables to defaults, read parameter values from
        the specified file, instantiate required objects (encoder, controllers
        etc), and update status variables.

        Args:
            params: The parameters filename to use for Shooter configuration.
            logging_enabled: True if logging should be enabled.

        """
        # Initialize public member variables
        self.encoder_enabled = False
        self.shooter_enabled = False

        # Initialize private member objects
        self._encoder = None
        self._left_shooter_controller = None
        self._right_shooter_controller = None
        self._log = None
        self._parameters = None
        self._timer = None

        # Initialize private parameters
        self._encoder_threshold = 10
        self._auto_far_encoder_threshold = 100
        self._auto_medium_encoder_threshold = 50
        self._encoder_max_limit = 10000
        self._encoder_min_limit = 0
        self._time_threshold = 0.1
        self._auto_medium_time_threshold = 0.5
        self._auto_far_time_threshold = 1.0
        self._auto_far_speed_ratio = 1.0
        self._auto_medium_speed_ratio = 1.0
        self._auto_near_speed_ratio = 1.0
        self._invert_left_shooter_motor = 1.0
        self._invert_right_shooter_motor = 1.0
        self._shooter_up_direction = 1.0
        self._shooter_down_direction = -1.0
        self._normal_up_speed_ratio = 1.0
        self._normal_down_speed_ratio = 0.3
        self._alternate_up_speed_ratio = 1.0
        self._alternate_down_speed_ratio = 0.5

        # Initialize private member variables
        self._encoder_count = 0
        self._ignore_encoder_limits = False
        self._log_enabled = False
        self._robot_state = common.ProgramState.DISABLED
        self._left_shooter_controller_enabled = False
        self._right_shooter_controller_enabled = False

        # Enable logging if specified
        if logging_enabled:
            # Create a new data log object
            self._log = datalog.DataLog("shooter.log")

            if self._log and self._log.file_opened:
                self._log_enabled = True
            else:
                self._log = None

        self._timer = stopwatch.Stopwatch()

        # Read parameters file
        self._parameters_file = params
        self.load_parameters()

    def load_parameters(self):
        """Load values from a parameter file and create and initialize objects.

        Read parameter values from the specified file, instantiate required
        objects (encoder, controller, etc), and update status variables.

        Returns:
            True if the parameter file was processed successfully.

        """
        # Define and initialize local variables
        left_shooter_channel = -1
        right_shooter_channel = -1
        encoder_a_slot = -1
        encoder_a_channel = -1
        encoder_b_slot = -1
        encoder_b_channel = -1
        encoder_reverse = 0
        encoder_type = 2

        # Initialize private parameters
        self._encoder_threshold = 10
        self._auto_far_encoder_threshold = 100
        self._auto_medium_encoder_threshold = 50
        self._encoder_max_limit = 10000
        self._encoder_min_limit = 0
        self._time_threshold = 0.1
        self._auto_medium_time_threshold = 0.5
        self._auto_far_time_threshold = 1.0
        self._auto_far_speed_ratio = 1.0
        self._auto_medium_speed_ratio = 1.0
        self._auto_near_speed_ratio = 1.0
        self._invert_left_shooter_motor = 1.0
        self._invert_right_shooter_motor = 1.0
        self._shooter_up_direction = 1.0
        self._shooter_down_direction = -1.0
        self._normal_up_speed_ratio = 1.0
        self._normal_down_speed_ratio = 0.3
        self._alternate_up_speed_ratio = 1.0
        self._alternate_down_speed_ratio = 0.5

        # Close and delete old objects
        self._parameters = None
        self._encoder = None
        self._left_shooter_controller = None
        self._right_shooter_controller = None

        # Read the parameters file
        self._parameters = parameters.Parameters(self._parameters_file)
        section = __name__.lower()

        # Store parameters from the file to local variables
        if self._parameters:
            encoder_a_slot = self._parameters.get_value(section,
                                                "ENCODER_A_SLOT")
            encoder_a_channel = self._parameters.get_value(section,
                                                "ENCODER_A_CHANNEL")
            encoder_b_slot = self._parameters.get_value(section,
                                                "ENCODER_B_SLOT")
            encoder_b_channel = self._parameters.get_value(section,
                                                "ENCODER_B_CHANNEL")
            encoder_reverse = self._parameters.get_value(section,
                                                "ENCODER_REVERSE")
            encoder_type = self._parameters.get_value(section,
                                                "ENCODER_TYPE")
            left_shooter_channel = self._parameters.get_value(section,
                                                "LEFT_SHOOTER_CHANNEL")
            right_shooter_channel = self._parameters.get_value(section,
                                                "RIGHT_SHOOTER_CHANNEL")
            self._encoder_threshold = self._parameters.get_value(section,
                                                "ENCODER_THRESHOLD")
            self._auto_medium_encoder_threshold = self._parameters.get_value(
                                                section,
                                                "AUTO_MEDIUM_ENCODER_THRESHOLD")
            self._auto_far_encoder_threshold = self._parameters.get_value(
                                                section,
                                                "AUTO_FAR_ENCODER_THRESHOLD")
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
            self._auto_far_speed_ratio = self._parameters.get_value(section,
                                                "AUTO_FAR_SPEED_RATIO")
            self._auto_medium_speed_ratio = self._parameters.get_value(section,
                                                "AUTO_MEDIUM_SPEED_RATIO")
            self._auto_near_speed_ratio = self._parameters.get_value(section,
                                                "AUTO_NEAR_SPEED_RATIO")
            self._invert_left_shooter_motor = self._parameters.get_value(
                                                section,
                                                "INVERT_LEFT_SHOOTER_MOTOR")
            self._invert_right_shooter_motor = self._parameters.get_value(
                                                section,
                                                "INVERT_RIGHT_SHOOTER_MOTOR")
            self._shooter_up_direction = self._parameters.get_value(section,
                                                "SHOOTER_UP_DIRECTION")
            self._shooter_down_direction = self._parameters.get_value(section,
                                                "SHOOTER_DOWN_DIRECTION")
            self._normal_up_speed_ratio = self._parameters.get_value(section,
                                                "NORMAL_UP_SPEED_RATIO")
            self._normal_down_speed_ratio = self._parameters.get_value(section,
                                                "NORMAL_DOWN_SPEED_RATIO")
            self._alternate_up_speed_ratio = self._parameters.get_value(section,
                                                "ALTERNATE_UP_SPEED_RATIO")
            self._alternate_down_speed_ratio = self._parameters.get_value(
                                                section,
                                                "ALTERNATE_DOWN_SPEED_RATIO")

        # Create the encoder object if the channel is greater than 0
        self.encoder_enabled = False
        if (encoder_a_slot > 0 and encoder_a_channel > 0 and
            encoder_b_slot > 0 and encoder_b_channel > 0):
            self._encoder = wpilib.Encoder(encoder_a_channel,
                                           encoder_b_channel,
                                           encoder_reverse,
                                           encoder_type)
            if self._encoder:
                self.encoder_enabled = True
                self._encoder.Start()
                if self._log_enabled:
                    self._log.write_line("Encoder enabled")

        # Create the motor controller objects if the channels are greater than 0
        self._right_shooter_controller_enabled = False
        self._left_shooter_controller_enabled = False
        if right_shooter_channel > 0:
            self._right_shooter_controller = wpilib.Jaguar(
                                                        right_shooter_channel)
        if left_shooter_channel > 0:
            self._left_shooter_controller = wpilib.Jaguar(left_shooter_channel)
        if self._right_shooter_controller:
            self._right_shooter_controller_enabled = True
            if self._log_enabled:
                self._log.write_line("Right shooter motor enabled")
        if self._left_shooter_controller:
            self._left_shooter_controller_enabled = True
            if self._log_enabled:
                self._log.write_line("Left shooter motor enabled")

        # If at least one motor is working, the shooter is enabled
        self.shooter_enabled = False
        if (self._left_shooter_controller_enabled or
            self._right_shooter_controller_enabled):
            self.shooter_enabled = True
            if self._log_enabled:
                self._log.write_line("Shooter enabled")

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
        if self._timer:
            self._timer.stop()

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

    def reset_and_start_timer(self):
        """Resets and restarts the timer for time based movement."""
        if self._timer:
            self._timer.stop()
            self._timer.start()

    def read_sensors(self):
        """Read and store current sensor values."""
        if self.encoder_enabled:
            self._encoder_count = self._encoder.Get()

    def reset_sensors(self):
        """Reset sensor values."""
        if self.encoder_enabled:
            self._encoder.Reset()
            self._encoder_count = self._encoder.Get()

    def get_current_state(self):
        """Return a string containing sensor and status variables.

        Returns:
            A string with the encoder value

        """
        return 'Encoder: %(enc)3.0f' % {'enc':self._encoder_count}

    def log_current_state(self):
        """Log sensor and status variables."""
        if self._log:
            if self.encoder_enabled:
                self._log.write_value("Encoder count", self._encoder_count,
                                      True)

    def set_shooter_position(self, position, speed):
        """Sets the catapult to a specified position.

        Args:
            position: The desired position in encoder counts.
            speed: The motor speed ratio.

        Returns:
            True when the desired position is reached.

        """
        # Abort if we don't have the encoder or motors
        if not self.encoder_enabled or not self.shooter_enabled:
            return True

        movement_direction = 0.0

        # Check the encoder position against the boundaries (if enabled)
        # Check max boundary
        if (not self._ignore_encoder_limits and self._encoder_max_limit > 0 and
            position > self._encoder_count and
            self._encoder_count > self._encoder_max_limit):
            if self._left_shooter_controller_enabled:
                self._left_shooter_controller.Set(0, 0)
            if self._right_shooter_controller_enabled:
                self._right_shooter_controller.Set(0, 0)
            return True
        # Check min boundary
        if (not self._ignore_encoder_limits and position < self._encoder_count
            and self._encoder_count < self._encoder_min_limit):
            if self._left_shooter_controller_enabled:
                self._left_shooter_controller.Set(0, 0)
            if self._right_shooter_controller_enabled:
                self._right_shooter_controller.Set(0, 0)
            return True

        # Check to see if we've reached the correct position
        if math.fabs(position - self._encoder_count) <= self._encoder_threshold:
            if self._left_shooter_controller_enabled:
                self._left_shooter_controller.Set(0, 0)
            if self._right_shooter_controller_enabled:
                self._right_shooter_controller.Set(0, 0)
            return True

        # Continue moving
        if (position - self._encoder_count) < 0:
            direction = (self._shooter_down_direction *
                         self._alternate_down_speed_ratio)
        else:
            direction = (self._shooter_up_direction *
                         self._alternate_up_speed_ratio)

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

        if self._left_shooter_controller_enabled:
            self._left_shooter_controller.Set((movement_direction *
                                           self._invert_left_shooter_motor),
                                           0)
        if self._right_shooter_controller_enabled:
            self._right_shooter_controller.Set((movement_direction *
                                           self._invert_right_shooter_motor),
                                           0)
        return False

    def shoot_time(self, time, direction, speed):
        """Moves the shooter for a certain time and speed.

        Args:
            time: the time to move the shooter.
            direction: the common.Direction enum in which to move.
            speed: the motor speed ratio.

        Returns:
            True when finished.

        """
        # Abort if we don't have the timer or motors
        if not self._timer or not self.shooter_enabled:
            return True

        # Get the timer value since we started moving
        elapsed_time = self._timer.elapsed_time_in_secs()

        # Calculate time left to move
        time_left = time - elapsed_time

        # Check the encoder position against the boundaries (if enabled)
        if self.encoder_enabled:
            # Check max boundary
            if (not self._ignore_encoder_limits and self._encoder_max_limit > 0
                and direction == common.Direction.UP and
                self._encoder_count > self._encoder_max_limit):
                if self._left_shooter_controller_enabled:
                    self._left_shooter_controller.Set(0, 0)
                if self._right_shooter_controller_enabled:
                    self._right_shooter_controller.Set(0, 0)
                return True
            # Check min boundary
            if (not self._ignore_encoder_limits and self._encoder_min_limit > 0
                and direction == common.Direction.DOWN and
                self._encoder_count < self._encoder_min_limit):
                if self._left_shooter_controller_enabled:
                    self._left_shooter_controller.Set(0, 0)
                if self._right_shooter_controller_enabled:
                    self._right_shooter_controller.Set(0, 0)
                return True

        # Check if we've reached the time duration
        if time_left < self._time_threshold or time_left < 0:
            if self._left_shooter_controller_enabled:
                self._left_shooter_controller.Set(0, 0)
            if self._right_shooter_controller_enabled:
                self._right_shooter_controller.Set(0, 0)
            self._timer.stop()
            return True
        directional_speed = 0
        if direction == common.Direction.DOWN:
            directional_speed = (self._shooter_down_direction *
                                 self._alternate_down_speed_ratio)
        else:
            directional_speed = (self._shooter_up_direction *
                                 self._alternate_up_speed_ratio)

        if time_left > self._auto_far_time_threshold:
            directional_speed = (directional_speed * speed *
                    self._auto_far_speed_ratio)
        elif time_left > self._auto_medium_time_threshold:
            directional_speed = (directional_speed * speed *
                    self._auto_medium_speed_ratio)
        else:
            directional_speed = (directional_speed * speed *
                    self._auto_near_speed_ratio)

        if self._left_shooter_controller_enabled:
            self._left_shooter_controller.Set((directional_speed *
                                           self._invert_left_shooter_motor),
                                           0)
        if self._right_shooter_controller_enabled:
            self._right_shooter_controller.Set((directional_speed *
                                           self._invert_right_shooter_motor),
                                           0)
        return False

    def move_shooter(self, directional_speed):
        """Moves the shooter at a specified speed and direction.

        Args:
            directional_speed: the speed and direction for moving.

        """
        # Abort if the robot drive is not available
        if not self.shooter_enabled:
            return

        # Check the encoder position against the boundaries (if enabled)
        if self.encoder_enabled:
            # Check max boundary
            if (not self._ignore_encoder_limits and self._encoder_max_limit > 0
                and self._shooter_up_direction * directional_speed > 0 and
                self._encoder_count > self._encoder_max_limit):
                if self._left_shooter_controller_enabled:
                    self._left_shooter_controller.Set(0, 0)
                if self._right_shooter_controller_enabled:
                    self._right_shooter_controller.Set(0, 0)
                return True
            # Check min boundary
            if (not self._ignore_encoder_limits and self._encoder_min_limit > 0
                and self._shooter_down_direction * directional_speed > 0 and
                self._encoder_count < self._encoder_min_limit):
                if self._left_shooter_controller_enabled:
                    self._left_shooter_controller.Set(0, 0)
                if self._right_shooter_controller_enabled:
                    self._right_shooter_controller.Set(0, 0)
                return True

        if self._shooter_up_direction * directional_speed > 0:
            directional_speed = directional_speed * self._normal_up_speed_ratio
        else:
            directional_speed = (directional_speed *
                                 self._normal_down_speed_ratio)

        if self._left_shooter_controller_enabled:
            self._left_shooter_controller.Set((directional_speed *
                                           self._invert_left_shooter_motor),
                                           0)
        if self._right_shooter_controller_enabled:
            self._right_shooter_controller.Set((directional_speed *
                                           self._invert_right_shooter_motor),
                                           0)

    def auto_fire(self, power_as_percent):
        """Fire a shot automatically using sensors.

        Args:
            power_as_percent: the percentage of the maximum power for the shot.

        Returns:
            True when finished.

        """
        # Abort if we don't have the encoder or motors
        if not self.encoder_enabled or not self.shooter_enabled:
            return True

        shooting_power_as_speed = ((power_as_percent / 100.0) *
                                   self._shooter_up_direction *
                                   self._alternate_up_speed_ratio)

        # Check the encoder position against the boundaries (if enabled)
        # Check max boundary
        if (not self._ignore_encoder_limits and self._encoder_max_limit > 0 and
            self._encoder_count > self._encoder_max_limit):
            if self._left_shooter_controller_enabled:
                self._left_shooter_controller.Set(0, 0)
            if self._right_shooter_controller_enabled:
                self._right_shooter_controller.Set(0, 0)
            return True

        # Check to see if we've reached the correct position
        if (math.fabs(self._encoder_max_limit - self._encoder_count) <=
            self._encoder_threshold):
            if self._left_shooter_controller_enabled:
                self._left_shooter_controller.Set(0, 0)
            if self._right_shooter_controller_enabled:
                self._right_shooter_controller.Set(0, 0)
            return True

        if self._left_shooter_controller_enabled:
            self._left_shooter_controller.Set((shooting_power_as_speed *
                                           self._invert_left_shooter_motor),
                                           0)
        if self._right_shooter_controller_enabled:
            self._right_shooter_controller.Set((shooting_power_as_speed *
                                           self._invert_right_shooter_motor),
                                           0)
        return False

    def ignore_encoder_limits(self, state):
        """Notify shooter to ignore encoder limits.

        Args:
            state: True if limits should be ignored.

        """
        self._ignore_encoder_limits = state

