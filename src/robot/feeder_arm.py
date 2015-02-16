"""This module describes a feeder arm."""

# Imports
import wpilib
import common
import logging
import logging.config
import parameters
import stopwatch


class FeederArm(object):
    """An arm that pulls objects in to the robot.

    This class describes a feeder arm with a spinning wheel on the end to pull
    objects into the robot.

    Attributes:
        arm_enabled: True if the arm is fully functional (default False).

    """
    # Public member variables
    arm_enabled = False

    # Private member objects
    _log = None
    _parameters = None
    _wheel_controller = None
    _movement_timer = None

    # Private parameters
    _time_threshold = 0
    _clockwise_direction = None
    _counter_clockwise_direction = None
    _clockwise_speed_ratio = None
    _counter_clockwise_speed_ratio = None

    # Private member variables
    _log_enabled = False
    _parameters_file = None
    _robot_state = common.ProgramState.DISABLED

    def __init__(self, params="/home/lvuser/par/arm.par",
                 logging_enabled=False):
        """Create and initialize an arm.

        Instantiate a FeederArm and specify a parameters file and whether
        logging is enabled or disabled.

        Args:
            params: The parameters filename to use for configuration.
            logging_enabled: True if logging should be enabled.

        """
        self._initialize(params, logging_enabled)

    def dispose(self):
        """Dispose of an arm object.

        Dispose of an arm object when it is no longer required by removing
        references to any internal objects.

        """
        self._log = None
        self._parameters = None
        self._wheel_controller = None
        self._movement_timer = None

    def _initialize(self, params, logging_enabled):
        """Initialize and configure a FeederArm object.

        Initialize instance variables to defaults, read parameter values from
        the specified file, instantiate required objects and update status
        variables.

        Args:
            params: The parameters filename to use for configuration.
            logging_enabled: True if logging should be enabled.

        """
        # Initialize public member variables
        self.arm_enabled = False

        # Initialize private member objects
        self._log = None
        self._parameters = None
        self._wheel_controller = None
        self._movement_timer = None

        # Initialize private parameters
        self._time_threshold = 0.1
        self._clockwise_direction = 1.0
        self._counter_clockwise_direction = -1.0
        self._clockwise_speed_ratio = 1.0
        self._counter_clockwise_speed_ratio = 1.0

        # Initialize private member variables
        self._log_enabled = False
        self._robot_state = common.ProgramState.DISABLED
        self._parameters_file = None

        # Enable logging if specified
        if logging_enabled:
            # Create a new data log object
            self._log = logging.getLogger('feederarm')
            self._log.setLevel(logging.DEBUG)
            fh = logging.FileHandler('/home/lvuser/log/feederarm.log')
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
        self._wheel_controller = None

        # Read the parameters file
        self._parameters = parameters.Parameters(self._parameters_file)
        section = __name__.lower()

        # Store parameters from the file to local variables
        if self._parameters:
            motor_channel = self._parameters.get_value(section,
                                            "MOTOR_CHANNEL")
            self._time_threshold = self._parameters.get_value(section,
                                            "TIME_THRESHOLD")
            self._clockwise_direction = self._parameters.get_value(section,
                                            "CLOCKWISE_DIRECTION")
            self._counter_clockwise_direction = self._parameters.get_value(
                                            section,
                                            "COUNTER_CLOCKWISE_DIRECTION")
            self._clockwise_speed_ratio = self._parameters.get_value(section,
                                            "CLOCKWISE_SPEED_RATIO")
            self._counter_clockwise_speed_ratio = self._parameters.get_value(
                                            section,
                                            "COUNTER_CLOCKWISE_SPEED_RATIO")

        # Create motor controllers
        if motor_channel >= 0:
            self._wheel_controller = wpilib.Victor(motor_channel)
            self.arm_enabled = True

        if self._log_enabled:
            if self.arm_enabled:
                self._log.debug("Arm enabled")
            else:
                self._log.debug("Arm disabled")

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
        pass

    def reset_sensors(self):
        """Reset sensors."""
        pass

    def reset_and_start_timer(self):
        """Resets and restarts the timer for time based movement."""
        if self._movement_timer:
            self._movement_timer.stop()
            self._movement_timer.start()

    def get_current_state(self):
        """Return a string containing sensor and status variables.

        Returns:
            ""
        """
        return ''

    def log_current_state(self):
        """Log sensor and status variables."""
        pass

    def spin(self, direction, speed):
        """Conrol the wheel on the end of the arm.

        Args:
            direction: Direction enumeration of clockwise, counterclockwise,
                or stop.
            speed: The speed of the wheel.

        """
        if direction == common.Direction.CLOCKWISE:
            if self.arm_enabled:
                self._wheel_controller.set((self._clockwise_direction * speed *
                    self._clockwise_speed_ratio), 0)
        elif direction == common.Direction.COUNTERCLOCKWISE:
            if self.arm_enabled:
                self._wheel_controller.set((self._counter_clockwise_direction *
                    speed * self._counter_clockwise_speed_ratio), 0)
        elif direction == common.Direction.STOP:
            if self.arm_enabled:
                self._wheel_controller.set(0.0, 0)

    def spin_time(self, time, direction, speed):
        """Controls the wheel on the end of the arm for a time duration.

        Using a timer, spins the wheel for a certain time duration.

        Args:
            time: the amount of time to spin.
            direction: the direction to spin.
            speed: the motor speed ratio.

        Returns:
            True when the time duration has been reached.
        """
        # Abort if arm or timer is not available
        if not self.arm_enabled or not self._movement_timer:
            return True

        # Get the timer value since we started moving
        elapsed_time = self._movement_timer.elapsed_time_in_secs()

        # Calculate time left to turn
        time_left = time - elapsed_time

        # Check if we've spun long enough
        if time_left < self._time_threshold or time_left < 0:
            if self.arm_enabled:
                self._wheel_controller.set(0.0, 0)
            self._movement_timer.stop()
            return True
        else:
            if direction == common.Direction.CLOCKWISE:
                if self.arm_enabled:
                    self._wheel_controller.set((self._clockwise_direction *
                        speed * self._clockwise_speed_ratio), 0)
            elif direction == common.Direction.COUNTERCLOCKWISE:
                if self.arm_enabled:
                    self._wheel_controller.set(
                            (self._counter_clockwise_direction *
                             speed * self._counter_clockwise_speed_ratio), 0)

        return False
