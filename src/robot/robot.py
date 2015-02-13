"""This module contains the FRC robot class."""

# Imports
import wpilib
#import autoscript
import common
import drivetrain
import feeder
import lift
import logging
import logging.config
import math
import userinterface


class MyRobot(wpilib.IterativeRobot):
    """Controls the robot.

    This is the main robot class that controls the robot during all 3 modes.
    """
    # Public member variables

    # Private member objects
    _drive_train = None
    _feeder = None
    _lift = None
    _log = None
    _user_interface = None

    # Private member variables
    _log_enabled = False
    _driver_alternate = False


    # Iterative robot methods that we override.
    # These get called by the main control loop in the IterativeRobot class.
    def robotInit(self):
        """Performs robot-wide initialization.

        Called upon robot power-on.

        """
        self._initialize("/home/lvuser/par/robot.par", True)

    def disabledInit(self):
        """Prepares the robot for Disabled mode.

        Called only when first disabled.

        """
        self._set_robot_state(common.ProgramState.DISABLED)

        # Read sensors
        self._read_sensors()

    def autonomousInit(self):
        """Prepares the robot for Autonomous mode.

        Called each and every time autonomous is entered from another mode.

        """
        self._set_robot_state(common.ProgramState.AUTONOMOUS)

        # Read sensors
        self._read_sensors()

    def teleopInit(self):
        """Prepares the robot for Teleop mode.

        Called each and every time teleop is entered from another mode.

        """
        self._set_robot_state(common.ProgramState.TELEOP)

        # Read sensors
        self._read_sensors()

    def testInit(self):
        """Prepares the robot for Test mode.

        Called each and every time test mode is entered from another mode.

        """
        pass

    def disabledPeriodic(self):
        """Called iteratively during disabled mode.

        The period is synced to the driver station control packets,
        giving a periodic frequency of about 50Hz (50 times per second).

        """
        # Set all motors to be stopped (prevent motor safety errors)
        if self._drive_train:
            self._drive_train.drive(0.0, 0.0, False)

        # Read sensors
        self._read_sensors()

    def autonomousPeriodic(self):
        """Called iteratively during autonomous mode.

        The period is synced to the driver station control packets,
        giving a periodic frequency of about 50Hz (50 times per second).

        """
        # Read sensors
        self._read_sensors()

    def teleopPeriodic(self):
        """Called iteratively during teleop mode.

        The period is synced to the driver station control packets,
        giving a periodic frequency of about 50Hz (50 times per second).

        """
        # Read sensors
        self._read_sensors()

        # Perform user controlled actions
        if self._user_interface:
            # Check for alternate speed mode request
            self._check_alternate_speed_modes()

            # Manually control the robot
            self._control_drive_train()
            self._control_feeder()
            self._control_lift()

            # Update/store the UI button state
            self._user_interface.store_button_states(
                    userinterface.UserControllers.DRIVER)
            self._user_interface.store_button_states(
                    userinterface.UserControllers.SCORING)

    def testPeriodic(self):
        """Called iteratively during test mode.

        The period is synced to the driver station control packets,
        giving a periodic frequency of about 50Hz (50 times per second).

        """
        pass

    # Custom methods used by the methods above
    def _initialize(self, params, logging_enabled):
        """Initialize the robot.

        Initialize instance variables to defaults, read parameter values from
        the specified file, instantiate required objects and update status
        variables.

        Args:
            params: The parameters filename to use for configuration.
            logging_enabled: True if logging should be enabled.

        """
        # Initialize public member variables

        # Initialize private member objects
        self._drive_train = None
        self._feeder = None
        self._lift = None
        self._log = None
        self._user_interface = None

        # Initialize private member variables
        self._log_enabled = False
        self._driver_alternate = False

        # Enable logging if specified
        if logging_enabled:
            # Create a new data log object
            self._log = logging.getLogger('robot')
            self._log.setLevel(logging.DEBUG)
            fh = logging.FileHandler('/home/lvuser/log/robot.log')
            fh.setLevel(logging.DEBUG)
            formatter = logging.Formatter(
                    '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            fh.setFormatter(formatter)
            self._log.addHandler(fh)
            if self._log:
                self._log_enabled = True
            else:
                self._log = None

        # Create robot objects
        self._drive_train = drivetrain.DriveTrain(
                                    "/home/lvuser/par/drivetrain.par",
                                    self._log_enabled)
        self._feeder = feeder.Feeder("/home/lvuser/par/feeder.par",
                                     self._log_enabled)
        self._lift = lift.Lift("/home/lvuser/par/lift.par", self._log_enabled)
        self._user_interface = userinterface.UserInterface(
                                    "/home/lvuser/par/userinterface.par",
                                    self._log_enabled)

    def _read_sensors(self):
        """Have the objects read their sensors."""
        if self._drive_train:
            self._drive_train.read_sensors()
        if self._feeder:
            self._feeder.read_sensors()
        if self._lift:
            self._lift.read_sensors()

    def _set_robot_state(self, state):
        """Notify objects of the current mode."""
        if self._drive_train:
            self._drive_train.set_robot_state(state)
        if self._feeder:
            self._feeder.set_robot_state(state)
        if self._lift:
            self._lift.set_robot_state(state)
        if self._user_interface:
            self._user_interface.set_robot_state(state)

    def _check_alternate_speed_modes(self):
        """Check for alternate speed mode."""
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.DRIVER,
                        userinterface.JoystickButtons.LEFTBUMPER) == 1):
            self._driver_alternate = True
        else:
            self._driver_alternate = False

    def _control_drive_train(self):
        """Manually control the drive train."""
        if self._drive_train:
            driver_left_y = self._user_interface.get_axis_value(
                    userinterface.UserControllers.DRIVER,
                    userinterface.JoystickAxis.LEFTY)
            driver_right_x = self._user_interface.get_axis_value(
                    userinterface.UserControllers.DRIVER,
                    userinterface.JoystickAxis.RIGHTX)
            if driver_left_y != 0.0 or driver_right_x != 0.0:
                self._drive_train.drive(driver_left_y, driver_right_x, False)
            else:
                self._drive_train.drive(0.0, 0.0, False)

    def _control_feeder(self):
        """Manually control the feeder."""
        if self._feeder:
            scoring_left_y = self._user_interface.get_axis_value(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickAxis.LEFTY)
            scoring_dpad_y = self._user_interface.get_axis_value(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickAxis.DPADY)

            if scoring_left_y != 0.0:
                direction = common.Direction.STOP
                if scoring_left_y > 0:
                    direction = common.Direction.OPEN
                else:
                    direction = common.Direction.CLOSE
                self._feeder.move_arms(direction, math.fabs(scoring_left_y))
            else:
                self._feeder.move_arms(common.Direction.STOP, 0.0)

            if scoring_dpad_y != 0.0:
                direction = common.Direction.STOP
                if scoring_dpad_y > 0:
                    direction = common.Direction.IN
                else:
                    direction = common.Direction.OUT
                self._feeder.feed(direction, math.fabs(scoring_dpad_y))
            else:
                self._feeder.feed(common.Direction.STOP, 0.0)


    def _control_lift(self):
        """Manually control the lift."""
        if self._lift:
            scoring_right_y = self._user_interface.get_axis_value(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickAxis.RIGHTY)
            if scoring_right_y != 0.0:
                self._lift.move_lift(scoring_right_y)
            else:
                self._lift.move_lift(0.0)

if __name__ == "__main__":
    wpilib.run(MyRobot)
