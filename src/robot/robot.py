"""This module contains the FRC robot class."""

# Imports
import wpilib
import autoscript
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
    _autoscript = None
    _drive_train = None
    _feeder = None
    _lift = None
    _log = None
    _autonomous_chooser = None
    _user_interface = None

    # Private member variables
    _log_enabled = False
    _driver_alternate = False
    _autoscript_filename = None
    _autoscript_finished = False
    _current_command = None
    _current_command_complete = False
    _autoscript_owner = None
    _autoscript_method = None
    _robot_names = None
    _drive_train_names = None
    _feeder_names = None
    _lift_names = None
    _user_interface_names = None
    _auto_selection_setup = False
    _scoring_ignore_limits = False

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

        # Get the list of autoscript files/routines
        if self._autoscript and not self._auto_selection_setup:
            self._auto_selection_setup = True
            # Create autonomous selection chooser
            autoscript_files = self._autoscript.get_available_scripts(
                                                        "/home/lvuser/as/")
            if autoscript_files and len(autoscript_files) > 0:
                for script_path in autoscript_files:
                    script_name = script_path.replace("/home/lvuser/as/", "")
                    script_name = script_name.replace(".as", "")
                    self._autonomous_chooser.addObject(script_name, script_path)
                wpilib.SmartDashboard.putData("Auto Program",
                                              self._autonomous_chooser)

    def autonomousInit(self):
        """Prepares the robot for Autonomous mode.

        Called each and every time autonomous is entered from another mode.

        """
        self._set_robot_state(common.ProgramState.AUTONOMOUS)

        # Read sensors
        self._read_sensors()

        if self._autoscript:
            self._autoscript_filename = self._autonomous_chooser.getSelected()
            if self._autoscript_filename:
                self._autoscript.parse(self._autoscript_filename)

        self._autoscript_finished = False
        self._current_command = None
        self._autoscript_owner = None
        self._autoscript_method = None
        # Initially true to get 1st command
        self._current_command_complete = True
        if not self._autoscript or not self._autoscript_filename:
            self._autoscript_finished = True

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

        if not self._autoscript_finished:
            # Handle the command
            # If we found the method, unpack the parameters List
            # (using *) and call it
            if not self._current_command_complete:
                try:
                    self._current_command_complete = \
                            self._autoscript_method(
                                    *self._current_command.parameters)
                except TypeError:
                    #self._logger.warn("TypeError running autoscript"
                    #            " command: " + str(current_command.command))
                    self._current_command_complete = True

            # Move on to the next command when the current is finished
            if self._current_command_complete:
                # Get next command
                self._current_command_complete = False
                self._current_command = self._autoscript.get_next_command()

                # If it's None or invalid or end, we're finished
                if (not self._current_command or
                    self._current_command.command == "invalid" or
                    self._current_command.command == "end"):
                    self._autoscript_finished = True
                else:
                    # Try to get the method reference using its name
                    self._autoscript_owner, self._autoscript_method = \
                            self._get_method(self._current_command.command)
                    if not self._autoscript_method:
                        self._current_command_complete = True
                    # Check if method has '_time' in it
                    elif '_time' in self._current_command.command:
                        # We need to reset its internal timer first
                        reset = None
                        self._autoscript_owner, reset = self._get_method(
                                                'reset_and_start_timer',
                                                obj=self._autoscript_owner)
                        if reset:
                            try:
                                reset()
                            except TypeError:
                                #self._logger.warn("TypeError running "
                                #                  "autoscript timer reset")
                                self._current_command_complete = True
                        else:
                            self._current_command_complete = True

        # Autoscript is finished
        else:
            # Set all motors to inactive
            if self._drive_train:
                self._drive_train.drive(0.0, 0.0, False)
            if self._feeder:
                self._feeder.feed(common.Direction.STOP, 0.0)
            if self._lift:
                self._lift.move_lift(0.0)

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
        self._autoscript = None
        self._drive_train = None
        self._feeder = None
        self._lift = None
        self._log = None
        self._autonomous_chooser = None
        self._user_interface = None

        # Initialize private member variables
        self._log_enabled = False
        self._driver_alternate = False
        self._autoscript_filename = None
        self._autoscript_finished = False
        self._current_command = None
        self._current_command_complete = False
        self._autoscript_owner = None
        self._autoscript_method = None
        self._robot_names = None
        self._drive_train_names = None
        self._feeder_names = None
        self._lift_names = None
        self._user_interface_names = None
        self._auto_selection_setup = False
        self._scoring_ignore_limits = False

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
        self._autonomous_chooser = wpilib.SendableChooser()
        self._autoscript = autoscript.AutoScript()
        self._drive_train = drivetrain.DriveTrain(
                                    "/home/lvuser/par/drivetrain.par",
                                    self._log_enabled)
        self._feeder = feeder.Feeder("/home/lvuser/par/feeder.par",
                                     self._log_enabled)
        self._lift = lift.Lift("/home/lvuser/par/lift.par", self._log_enabled)
        self._user_interface = userinterface.UserInterface(
                                    "/home/lvuser/par/userinterface.par",
                                    self._log_enabled)

        # Store the attributes/names in each object
        self._robot_names = dir(self)
        self._drive_train_names = dir(self._drive_train)
        self._feeder_names = dir(self._feeder)
        self._lift_names = dir(self._lift)
        self._user_interface_names = dir(self._user_interface)

    def _get_method(self, name, obj=None):
        """Find the matching method in one of the objects.

        Args:
            name: the name of the method.

        Returns:
            Object, Method.

        """
        calling_object = None
        method = None

        # Determine which object contains the method
        if not obj:
            if name:
                if name in self._robot_names:
                    calling_object = self
                elif name in self._drive_train_names:
                    calling_object = self._drive_train
                elif name in self._feeder_names:
                    calling_object = self._feeder
                elif name in self._lift_names:
                    calling_object = self._lift
                elif name in self._user_interface_names:
                    calling_object = self._user_interface
        else:
            calling_object = obj

        # Get the method pointer (sort of) from the object
        if calling_object:
            try:
                method = getattr(calling_object, name)
                # Make sure method is callable
                if method and not callable(method):
                    method = None
            except AttributeError:
                #self._logger.warn("AttributeError getting method pointer")
                pass

        return calling_object, method

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
                self._drive_train.drive(driver_left_y, driver_right_x,
                                        self._driver_alternate)
            else:
                self._drive_train.drive(0.0, 0.0, False)

    def _control_feeder(self):
        """Manually control the feeder."""
        if self._feeder:
            scoring_right_x = self._user_interface.get_axis_value(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickAxis.RIGHTX)
            scoring_left_trigger = self._user_interface.get_button_state(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickButtons.LEFTTRIGGER)
            scoring_right_trigger = self._user_interface.get_button_state(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickButtons.RIGHTTRIGGER)
            scoring_left_bumper = self._user_interface.get_button_state(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickButtons.LEFTBUMPER)
            scoring_right_bumper = self._user_interface.get_button_state(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickButtons.RIGHTBUMPER)

            if scoring_right_x != 0.0:
                direction = common.Direction.STOP
                if scoring_right_x > 0:
                    direction = common.Direction.OPEN
                else:
                    direction = common.Direction.CLOSE
                self._feeder.move_arms(direction, math.fabs(scoring_right_x))
            else:
                self._feeder.move_arms(common.Direction.STOP, 0.0)

            if (scoring_left_trigger != 0.0 or scoring_right_trigger != 0.0 or
                scoring_left_bumper != 0.0 or scoring_right_bumper != 0.0):
                direction = common.Direction.STOP
                if scoring_right_trigger != 0.0:
                    direction = common.Direction.IN
                elif scoring_left_trigger != 0.0:
                    direction = common.Direction.OUT
                elif scoring_left_bumper != 0.0:
                    direction = common.Direction.CLOCKWISE
                elif scoring_right_bumper != 0.0:
                    direction = common.Direction.COUNTERCLOCKWISE
                self._feeder.feed(direction, 1.0)
            else:
                self._feeder.feed(common.Direction.STOP, 0.0)


    def _control_lift(self):
        """Manually control the lift."""
        if self._lift:
            scoring_left_y = self._user_interface.get_axis_value(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickAxis.LEFTY)
            scoring_y_button_value = self._user_interface.get_button_state(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickButtons.Y)
            scoring_y_button_chg = self._user_interface.button_state_changed(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickButtons.Y)

            if scoring_y_button_value != 0.0 and scoring_y_button_chg:
                self._scoring_ignore_limits = not self._scoring_ignore_limits
                self._lift.ignore_encoder_limits(self._scoring_ignore_limits)
                self._feeder.ignore_encoder_limits(self._scoring_ignore_limits)

            if scoring_left_y != 0.0:
                self._lift.move_lift(scoring_left_y)
            else:
                self._lift.move_lift(0.0)

if __name__ == "__main__":
    wpilib.run(MyRobot)
