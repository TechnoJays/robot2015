"""This module contains the FRC robot class."""

# Imports

# If wpilib not available use pyfrc
try:
    import wpilib
except ImportError:
    from pyfrc import wpilib

import autoscript
import common
#import datalog
import drivetrain
import feeder
import logging
import math
import parameters
import queue
import shooter
import stopwatch
import sys
import target
import target_server
import userinterface


class MyRobot(wpilib.SimpleRobot):
    """Controls the robot.

    This is the main robot class that controls the robot during all 3 modes.
    There is 1 method for each mode: Disabled, Autonomous, and OperatorControl
    (teleop).  The methods are called once from the main control loop each time
    the robot enters the states.

    """
    # Public member variables

    # Private member objects
    _autoscript = None
    _drive_train = None
    _feeder = None
    _log = None
    _logger = None
    _parameters = None
    _shooter = None
    _image_server = None
    _timer = None
    _range_print_timer = None
    _user_interface = None

    # Private parameters
    _max_hold_to_shoot_time = None
    _min_hold_to_shoot_power = None
    _catapult_feed_position = None
    _truss_pass_power = None
    _truss_pass_position = None
    _optimum_shooting_range = None
    _shooting_angle_offset = None

    # Private member variables
    _log_enabled = False
    _parameters_file = None
    _autoscript_file_counter = 0
    _autoscript_filename = None
    _autoscript_files = None
    _driver_alternate = False
    _scoring_alternate = False
    _current_feeder_position = None
    _hold_to_shoot_step = -1
    _hold_to_shoot_power = -1
    _prep_for_feed_step = -1
    _truss_pass_step = -1
    _driver_controls_swap_ratio = 1.0
    _hold_to_shoot_power_factor = 0.0
    _shooter_setup_step = -1
    _aim_at_target_step = -1
    _aim_at_target_target = None
    _disable_range_print = False
    _robot_names = None
    _drive_train_names = None
    _feeder_names = None
    _shooter_names = None
    _user_interface_names = None
    _target_queue = None
    _current_targets = None

    def _initialize(self, params, logging_enabled):
        """Initialize the robot.

        Initialize instance variables to defaults, read parameter values from
        the specified file, instantiate required objects and update status
        variables.

        Args:
            params: The parameters filename to use for configuration.
            logging_enabled: True if logging should be enabled.

        """
        self._logger = logging.getLogger(__name__)
        handler = None
        formatter = logging.Formatter('%(asctime)s - %(levelname)s:'
                                      '%(name)s:%(message)s')
        handler = logging.StreamHandler(stream=sys.stdout)
        handler.setLevel(logging.INFO)
        handler.setFormatter(formatter)
        self._logger.addHandler(handler)
        self._logger.setLevel(logging.INFO)

        # Initialize public member variables

        # Initialize private member objects
        self._autoscript = None
        self._drive_train = None
        self._feeder = None
        self._log = None
        self._parameters = None
        self._shooter = None
        self._image_server = None
        self._timer = None
        self._range_print_timer = None
        self._user_interface = None

        # Initialize private parameters
        self._max_hold_to_shoot_time = None
        self._min_hold_to_shoot_power = None
        self._catapult_feed_position = None
        self._truss_pass_power = None
        self._truss_pass_position = None
        self._optimum_shooting_range = None
        self._shooting_angle_offset = None

        # Initialize private member variables
        self._log_enabled = False
        self._parameters_file = None
        self._autoscript_file_counter = 0
        self._autoscript_filename = None
        self._autoscript_files = None
        self._driver_alternate = False
        self._scoring_alternate = False
        self._driver_controls_swap_ratio = 1.0
        self._current_feeder_position = None
        self._hold_to_shoot_step = -1
        self._hold_to_shoot_power = 0
        self._prep_for_feed_step = -1
        self._truss_pass_step = -1
        self._hold_to_shoot_power_factor = 0.0
        self._shooter_setup_step = -1
        self._aim_at_target_step = -1
        self._aim_at_target_target = None
        self._disable_range_print = False
        self._target_queue = None
        self._current_targets = []

        # Enable logging if specified
        #if logging_enabled:
            # Create a new data log object
            #self._log = datalog.DataLog("robot.log")

            #if self._log and self._log.file_opened:
                #self._log_enabled = True

        self._timer = stopwatch.Stopwatch()
        self._range_print_timer = stopwatch.Stopwatch()

        # Read parameters file
        self._parameters_file = params
        self.load_parameters()

        # Create robot objects
        self._autoscript = autoscript.AutoScript()
        self._drive_train = drivetrain.DriveTrain("/py/par/drivetrain.par",
                                                  self._log_enabled)
        self._feeder = feeder.Feeder("/py/par/feeder.par", self._log_enabled)
        self._shooter = shooter.Shooter("/py/par/shooter.par",
                                        self._log_enabled)
        self._user_interface = userinterface.UserInterface(
                                                    "/py/par/userinterface.par",
                                                    self._log_enabled)

        # Store the attributes/names in each object
        self._robot_names = dir(self)
        self._drive_train_names = dir(self._drive_train)
        self._feeder_names = dir(self._feeder)
        self._shooter_names = dir(self._shooter)
        self._user_interface_names = dir(self._user_interface)

        # Create a queue for transferring Targets from the image server to us
        # Since we pass a List of targets, the size will be 1
        self._target_queue = queue.Queue(1)

        # Create the TCP image server, and start it in a background thread
        self._image_server = target_server.ImageServer(self._target_queue)
        self._image_server.start()

    def load_parameters(self):
        """Load values from a parameter file and create and initialize objects.

        Read parameter values from the specified file, instantiate required
        objects, and update status variables.

        Returns:
            True if the parameter file was processed successfully.

        """
        # Close and delete old objects
        self._parameters = None

        # Read the parameters file
        self._parameters = parameters.Parameters(self._parameters_file)
        section = __name__.lower()

        # Read parameters from the file
        if self._parameters:
            self._max_hold_to_shoot_time = self._parameters.get_value(section,
                                                "MAX_HOLD_TO_SHOOT_TIME")
            self._min_hold_to_shoot_power = self._parameters.get_value(section,
                                                "MIN_HOLD_TO_SHOOT_POWER")
            self._catapult_feed_position = self._parameters.get_value(section,
                                                "CATAPULT_FEED_POSITION")
            self._truss_pass_power = self._parameters.get_value(section,
                                                "TRUSS_PASS_POWER")
            self._truss_pass_position = self._parameters.get_value(section,
                                                "TRUSS_PASS_POSITION")
            self._optimum_shooting_range = self._parameters.get_value(section,
                                                "OPTIMUM_SHOOTING_RANGE")
            self._shooting_angle_offset = self._parameters.get_value(section,
                                                "SHOOTING_ANGLE_OFFSET")

        self._hold_to_shoot_power_factor = ((100.0 -
                                             self._min_hold_to_shoot_power) /
                                            100.0)
        return True

    def RobotInit(self):
        """Performs robot-wide initialization.

        Called each time the robot enters the Disabled mode.

        """
        self._initialize("/py/par/robot.par", True)

    def _disabled_init(self):
        """Prepares the robot for Disabled mode."""
        self._set_robot_state(common.ProgramState.DISABLED)

        # Default starting mode is to have the arms UP
        if self._feeder:
            self._current_feeder_position = common.Direction.UP
            self._feeder.set_feeder_position(self._current_feeder_position)

        # Read sensors
        self._read_sensors()

        # Get the list of autoscript files/routines
        if self._autoscript:
            self._autoscript_files = self._autoscript.get_available_scripts(
                                                                    "/py/as/")
            if self._autoscript_files and len(self._autoscript_files) > 0:
                self._autoscript_file_counter = 0
                self._autoscript_filename = self._autoscript_files[
                                                self._autoscript_file_counter]
                if self._user_interface:
                    self._disable_range_print = True
                    self._user_interface.output_user_message(
                                                self._autoscript_filename,
                                                True)

        self.GetWatchdog().SetEnabled(False)

    def Disabled(self):
        """Control the robot during Disabled mode.

        Monitors the user input for a restart request.  This is
        useful during development to load new Python code and
        avoid rebooting the robot.

        Handle the changing of program settings from the driver
        before the start of a match (e.g., autonomous program).

        """
        self._disabled_init()

        # Repeat this loop as long as we're in Disabled
        while self.IsDisabled():
            # Set all motors to be stopped (prevent motor safety errors)
            if self._drive_train:
                self._drive_train.drive(0.0, 0.0, False)
            if self._feeder:
                self._feeder.feed(feeder.Direction.STOP, 0.0)
            if self._shooter:
                self._shooter.move_shooter(0.0)
            if (self._user_interface and self._autoscript and
                self._autoscript_files and len(self._autoscript_files) > 0):
                if (self._user_interface.get_button_state(
                                    userinterface.UserControllers.DRIVER,
                                    userinterface.JoystickButtons.START) == 1
                    and self._user_interface.button_state_changed(
                                    userinterface.UserControllers.DRIVER,
                                    userinterface.JoystickButtons.START)):
                    self._autoscript_file_counter += 1
                    if (self._autoscript_file_counter >
                        (len(self._autoscript_files) - 1)):
                        self._autoscript_file_counter = 0
                    self._autoscript_filename = self._autoscript_files[
                                                  self._autoscript_file_counter]
                    self._disable_range_print = True
                    self._user_interface.output_user_message(
                                                self._autoscript_filename,
                                                True)
                self._user_interface.store_button_states(
                                        userinterface.UserControllers.DRIVER)

            # Read sensors
            self._read_sensors()
            self._print_range(False)
            #self._print_targets(False)

            wpilib.Wait(0.01)

    def _autonomous_init(self):
        """Prepares the robot for Autonomous mode."""
        # Perform initialization before looping
        if self._autoscript and self._autoscript_filename:
            self._autoscript.parse(self._autoscript_filename)

        # Read sensors
        self._read_sensors()
        self._disable_range_print = False

        self._set_robot_state(common.ProgramState.AUTONOMOUS)
        self.GetWatchdog().SetEnabled(False)

        # Get targets in the queue if any exist
        if not self._target_queue.empty():
            try:
                self._current_targets = self._target_queue.get(block=False)
                if (not self._current_targets or
                    not isinstance(self._current_targets, list) or
                    (len(self._current_targets) == 1 and
                     self._current_targets[0].no_targets)):
                    self._current_targets = []
                    self._logger.debug("No targets flag, clearing targets")
            except queue.Empty:
                self._logger.warn("Target queue is empty")
                self._current_targets = []
            self._logger.debug("Targets: " + str(self._current_targets))
        else:
            self._logger.debug("Target queue is empty")


        # We set this to -2 to prepare for autonomous use
        self._aim_at_target_step = -2
        self._aim_at_target_target = None

        # Prepare to perform the shooter setup
        self._shooter_setup_step = 1

    def Autonomous(self):
        """Controls the robot during Autonomous mode.

        Instantiate a Class using default values.

        """
        # Autonomous initialization
        self._autonomous_init()

        # Method names must be unique for reflection to work,
        # and they should be descriptive
        autoscript_finished = False
        current_command = None
        current_command_complete = True # Initially true to get 1st command

        if not self._autoscript or not self._autoscript_filename:
            autoscript_finished = True

        # These store the owning object and method pointer for each
        # autoscript command (using reflection and introspection)
        owner = None
        method = None

        # Repeat this loop as long as we're in Autonomous
        while self.IsAutonomous() and self.IsEnabled():

            # Read sensors
            self._read_sensors()
            self._print_range(True)
            #self._print_targets(False)

            # Get targets in the queue if any exist
            if not self._target_queue.empty():
                try:
                    self._current_targets = self._target_queue.get(block=False)
                    if (not self._current_targets or
                        not isinstance(self._current_targets, list) or
                        (len(self._current_targets) == 1 and
                         self._current_targets[0].no_targets)):
                        self._current_targets = []
                        self._logger.debug("No targets flag, clearing targets")
                except queue.Empty:
                    self._logger.warn("Target queue is empty")
                    self._current_targets = []
                self._logger.debug("Targets: " + str(self._current_targets))
            else:
                self._logger.debug("Target queue is empty")

            # Execute autoscript commands
            if not autoscript_finished:

                # Handle the command
                # If we found the method, unpack the parameters List
                # (using *) and call it
                if not current_command_complete:
                    try:
                        current_command_complete = \
                                method(*current_command.parameters)
                    except TypeError:
                        self._logger.warn("TypeError running autoscript"
                                    " command: " + str(current_command.command))
                        current_command_complete = True

                # Move on to the next command when the current is finished
                if current_command_complete:
                    # Get next command
                    current_command_complete = False
                    current_command = self._autoscript.get_next_command()

                    # If it's None or invalid or end, we're finished
                    if (not current_command or
                        current_command.command == "invalid" or
                        current_command.command == "end"):
                        autoscript_finished = True
                    else:
                        # Try to get the method reference using its name
                        owner, method = self._get_method(
                                                    current_command.command)
                        if not method:
                            current_command_complete = True
                        # Check if method has '_time' in it
                        elif '_time' in current_command.command:
                            # We need to reset its internal timer first
                            reset = None
                            owner, reset = self._get_method(
                                                    'reset_and_start_timer',
                                                    obj=owner)
                            if reset:
                                try:
                                    reset()
                                except TypeError:
                                    self._logger.warn("TypeError running "
                                                      "autoscript timer reset")
                                    current_command_complete = True
                            else:
                                current_command_complete = True

            # Autoscript is finished
            else:
                # Set all motors to inactive
                if self._drive_train:
                    self._drive_train.drive(0.0, 0.0, False)
                if self._feeder:
                    self._feeder.feed(feeder.Direction.STOP, 0.0)
                if self._shooter:
                    self._shooter.move_shooter(0.0)

            wpilib.Wait(0.01)

    def _get_method(self, name, obj=None):
        """This tries to find the matching method in one of the objects.

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
                elif name in self._shooter_names:
                    calling_object = self._shooter
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
                self._logger.warn("AttributeError getting method pointer")

        return calling_object, method

    def _operator_control_init(self):
        """Prepares the robot for Teleop mode."""
        # Perform initialization before looping
        if self._timer:
            self._timer.stop()

        self._disable_range_print = False
        self._set_robot_state(common.ProgramState.TELEOP)

        # Enable the watchdog
        dog = self.GetWatchdog()
        dog.SetEnabled(True)
        dog.SetExpiration(1.0)

    def OperatorControl(self):
        """Controls the robot during Teleop/OperatorControl mode.

        Instantiate a Class using default values.

        """
        self._operator_control_init()
        dog = self.GetWatchdog()
        # Repeat this loop as long as we're in Teleop
        while self.IsOperatorControl() and self.IsEnabled():
            # Feed the watchdog timer
            dog.Feed()

            # Read sensors
            self._read_sensors()

            # Get targets in the queue if any exist
            if not self._target_queue.empty():
                try:
                    self._current_targets = self._target_queue.get(block=False)
                    if (not self._current_targets or
                        not isinstance(self._current_targets, list) or
                        (len(self._current_targets) == 1 and
                         self._current_targets[0].no_targets)):
                        self._current_targets = []
                        self._logger.debug("No targets flag, clearing targets")
                except queue.Empty:
                    self._logger.warn("Target queue is empty")
                    self._current_targets = []
                self._logger.debug("Targets: " + str(self._current_targets))
            else:
                self._logger.debug("Target queue is empty")

            # Perform tele-auto routines
            self._perform_tele_auto()

            # Perform user controlled actions
            if self._user_interface:
                # Check for alternate speed mode request
                self._check_alternate_speed_modes()

                # Check for ignore encoder limit request
                self._check_ignore_limits()

                # Print the range and check for other print timeouts
                self._print_range(True)
                #self._print_targets(False)
                self._check_ui_print_timeout()

                # Check swap drivetrain direction request
                self._check_swap_drivetrain_request()

                # Check for tele-auto requests
                self._check_tele_auto_requests()

                # Manually control the robot
                self._control_drive_train()
                self._control_shooter()
                self._control_feeder()

                # Check for tele-auto kill switch
                self._check_tele_auto_kill()

                # Check for debug to console request
                self._check_debug_request()

                # Update/store the UI button state
                self._user_interface.store_button_states(
                        userinterface.UserControllers.DRIVER)
                self._user_interface.store_button_states(
                        userinterface.UserControllers.SCORING)

            wpilib.Wait(0.01)

    def reset_and_start_timer(self):
        """Resets and restarts the timer."""
        if self._timer:
            self._timer.stop()
            self._timer.start()

    def wait_time(self, duration):
        """Autonomous method that does nothing for a specified duration."""
        elapsed_time = self._timer.elapsed_time_in_secs()
        time_left = duration - elapsed_time
        if time_left < 0:
            self._timer.stop()
            return True
        return False

    def _read_sensors(self):
        """Have the objects read their sensors."""
        if self._drive_train:
            self._drive_train.read_sensors()
        if self._shooter:
            self._shooter.read_sensors()

    def _set_robot_state(self, state):
        """Notify objects of the current mode."""
        if self._drive_train:
            self._drive_train.set_robot_state(state)
        if self._feeder:
            self._feeder.set_robot_state(state)
        if self._shooter:
            self._shooter.set_robot_state(state)
        if self._user_interface:
            self._user_interface.set_robot_state(state)

    def _check_tele_auto_requests(self):
        """Check if any teleop autonomous routines are requested."""
        # Hold the right trigger to shoot, longer duration = more power
        if (self._user_interface.button_state_changed(
                    userinterface.UserControllers.SCORING,
                    userinterface.JoystickButtons.RIGHTTRIGGER)):
            # If the trigger is just now held down, start counting the duration
            if (self._user_interface.get_button_state(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.RIGHTTRIGGER) == 1):
                self._timer.start()
                self._hold_to_shoot_step = 1
            # If the trigger has been let go, calcluate shot power and shoot
            else:
                # Calculate how long the trigger was held and convert to a %
                self._timer.stop()
                duration = self._timer.elapsed_time_in_secs()
                requested_power = (((duration * 1.0) /
                                    self._max_hold_to_shoot_time)
                                   * 100.0)
                self._hold_to_shoot_power = ((requested_power *
                                              self._hold_to_shoot_power_factor)
                                             + self._min_hold_to_shoot_power)
                if self._hold_to_shoot_power > 100.0:
                    self._hold_to_shoot_power = 100.0
                self._shooter.reset_and_start_timer()
                self._hold_to_shoot_step = 2
        # Press Y on scoring to prepare to pick up a ball
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.Y) == 1 and
            self._user_interface.button_state_changed(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.Y)):
            self._prep_for_feed_step = 1
        # Press Y on driver to auto-aim
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.DRIVER,
                        userinterface.JoystickButtons.Y) == 1 and
            self._user_interface.button_state_changed(
                        userinterface.UserControllers.DRIVER,
                        userinterface.JoystickButtons.Y)):
            self._drive_train.reset_sensors()
            self._aim_at_target_step = 1
            self._aim_at_target_target = None
        # Press left bumper to pass over the truss
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.LEFTBUMPER) == 1 and
            self._user_interface.button_state_changed(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.LEFTBUMPER)):
            self._truss_pass_step = 1

    def shooter_setup(self):
        """Get the shooter into a workable state.

        This is because the shooter starts with the arm UP, so the encoder
        value is not starting at 0.  To fix this, we disable encoder
        boundaries and move the arm down for enough time to ensure the arm is
        all the way down.  Then we reset the encoder to 0.

        Returns:
            True when setup is complete.

        """
        if self._shooter:
            # This requires the air tank to be pre-charged before a match
            # TODO: move time and speed to parameters file in step 3
            if self._shooter_setup_step == 1:
                if self._feeder:
                    self._current_feeder_position = common.Direction.DOWN
                    self._feeder.set_feeder_position(
                                            self._current_feeder_position)
                    wpilib.Wait(0.5)
                self._shooter_setup_step = 2
            elif self._shooter_setup_step == 2:
                self._shooter.ignore_encoder_limits(True)
                self._shooter.reset_and_start_timer()
                self._shooter_setup_step = 3
            elif self._shooter_setup_step == 3:
                if self._shooter.shoot_time(2.2, common.Direction.DOWN, 0.4):
                    self._shooter_setup_step = 4
            elif self._shooter_setup_step == 4:
                self._shooter.reset_sensors()
                self._shooter.ignore_encoder_limits(False)
                self._shooter_setup_step = -1
                return True
        else:
            return True

        return False

    def aim_at_target(self, side=None, desired_target=None):
        """Turn and drive until we are aiming at a target.

        This will turn the robot left or right, as well as drive forward or
        backward until the right distance is reached to shoot.  This can be
        called with either a particular target in mind or a
        side of the playing field/target wall.

        Args:
            side: the side to aim at.
            desired_target: The target.Target to aim at.

        Returns:
            True when complete.

        """
        # This is setup for using aim_at_target in autonomous mode
        # This is required because we can't set the aim_at_target_step variable
        # since we now use reflection/introspection.
        if self._aim_at_target_step == -2:
            self._aim_at_target_step = 1
            return False

        # Figure out which target to aim at
        current_target = desired_target
        if side:
            for trg in self._current_targets:
                if trg.side == side:
                    current_target = trg

        # Bail if we don't have a target
        if not current_target:
            self._aim_at_target_step = -1
            return True

        # Step 1 is to drive until we're at the optimum distance to shoot
        if self._aim_at_target_step == 1:
            # Use camera target distance instead of range finder
            distance_left = (current_target.distance -
                             self._optimum_shooting_range)
            # If we're within tolerance of the distance, stop the motors
            # and restart the drive train timer to drive backwards briefly
            if math.fabs(distance_left) < 0.5:
                self._drive_train.arcade_drive(0.0, 0.0, False)
                self._drive_train.reset_and_start_timer()
                self._aim_at_target_step = 2
            else:
                # TODO: add variable speed based on distance to target
                direction = -1.0 if distance_left > 0 else 1.0
                directional_speed = direction * 0.3
                self._drive_train.arcade_drive(directional_speed, 0.0, False)
        # Step 2 is to drive backwards briefly to stop the robot
        elif self._aim_at_target_step == 2:
            # TODO: this should not be hard-coded to backwards
            if self._drive_train.drive_time(0.1, common.Direction.BACKWARD,
                                            0.5):
                self._drive_train.reset_sensors()
                self._aim_at_target_step = 3
        # Step 3 is to turn to face the target
        elif self._aim_at_target_step == 3:
            # Include an offset. This is required since the image targets aren't
            # exactly where we want to aim (they're to the outside of the goals)
            # TODO: this was turning oddly: sometimes it would work, sometimes
            # it would turn way too far
            adjustment = current_target.angle
            if current_target.side == target.Side.LEFT:
                adjustment += self._shooting_angle_offset
            elif current_target.side == target.Side.RIGHT:
                adjustment -= self._shooting_angle_offset
            else:
                self._aim_at_target_step = -1
                return True
            if self._drive_train.adjust_heading(adjustment, 0.3):
                self._aim_at_target_step = -1
                return True

        return False

    def wait_for_hot_goal_with_time(self, side=None, desired_target=None,
                                    timeout=5.0):
        """Wait for the target goal to be 'hot'.

        This can be called with either a particular target in mind or a
        side of the playing field/target wall.

        Args:
            side: the side to become 'hot'
            desired_target: the target to become 'hot'

        Returns:
            True when complete.

        """
        # Figure out which target to check, and then check if it's 'hot'
        if side:
            for trg in self._current_targets:
                if ((trg.side == side or side == target.Side.EITHER) and
                    trg.is_hot):
                    return True
        elif desired_target:
            if desired_target.is_hot:
                return True
        else:
            return True

        elapsed_time = self._timer.elapsed_time_in_secs()
        time_left = timeout - elapsed_time
        if time_left < 0:
            self._timer.stop()
            return True

        return False

    def _perform_tele_auto(self):
        """Perform teleop autonomous actions."""
        # Hold to shoot
        # Print the shot power to the screen, then briefly move the shooter
        # down before shooting. The students said they got better performance
        # doing this..?
        if self._hold_to_shoot_step == 2:
            self._disable_range_print = True
            self._range_print_timer.start()
            self._user_interface.output_user_message('Power: %(pwr)3.0f' %
                                             {'pwr':self._hold_to_shoot_power},
                                             True)
            if self._shooter:
                if self._shooter.shoot_time(0.1, common.Direction.DOWN, 1.0):
                    self._hold_to_shoot_step = 3
        # Actually shoot the ball
        elif self._hold_to_shoot_step == 3:
            if self._shooter:
                if self._shooter.auto_fire(self._hold_to_shoot_power):
                    self._hold_to_shoot_step = -1

        # Prep for feed
        if self._prep_for_feed_step != -1:
            # Step 1 is to make sure the feeder arms are down
            if self._prep_for_feed_step == 1:
                if self._feeder:
                    self._current_feeder_position = common.Direction.DOWN
                    self._feeder.set_feeder_position(
                                                self._current_feeder_position)
                self._prep_for_feed_step = 2
            # Step 2 is to move the catapult arm all the way down
            elif self._prep_for_feed_step == 2:
                if self._shooter:
                    if self._shooter.set_shooter_position(
                                                self._catapult_feed_position,
                                                1.0):
                        self._prep_for_feed_step = -1

        # Truss pass
        if self._truss_pass_step != -1:
            if self._shooter:
                # This does a partial shot, moving the shooter at full speed,
                # but stopping before reaching the full range of motion
                if self._shooter.set_shooter_position(self._truss_pass_position,
                                                      1.0):
                    self._truss_pass_step = -1

        # Aim at target
        if self._aim_at_target_step > 0:
            self.aim_at_nearest()

    def _sort_targets(self):
        """Sort the targets based on which we're most closely facing."""
        if len(self._current_targets) > 1:
            # Sort targets based on which one we're most closely facing
            self._current_targets = sorted(self._current_targets,
                                           key=lambda x: math.fabs(x.angle),
                                           reverse=False)

    def aim_at_nearest(self):
        """Turn and drive until we are aiming at the nearest target."""
        if len(self._current_targets) > 0:
            self._sort_targets()
            self._aim_at_target_target = self._current_targets[0]

        # Aim at the nearest target
        return self.aim_at_target(desired_target=self._aim_at_target_target)

    def _check_debug_request(self):
        """Print debug info to driver station."""
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.DRIVER,
                        userinterface.JoystickButtons.BACK) == 1 and
            self._user_interface.button_state_changed(
                        userinterface.UserControllers.DRIVER,
                        userinterface.JoystickButtons.BACK)):
            # Disable the range to object prints briefly
            self._disable_range_print = True
            self._range_print_timer.start()
            if self._drive_train:
                self._drive_train.log_current_state()
                state = self._drive_train.get_current_state()
                self._user_interface.output_user_message(state, True)
            if self._shooter:
                self._shooter.log_current_state()
                state = self._shooter.get_current_state()
                self._user_interface.output_user_message(state, False)
            self._print_targets(False)

    def _print_range(self, clear=True):
        """Print the range to the nearest object."""
        # Only print the range if nothing else important is being shown
        if not self._disable_range_print:
            if self._drive_train:
                rng = self._drive_train.get_range()
                self._user_interface.output_user_message('Range: %(rng)4.1f' %
                                                         {'rng':rng},
                                                         clear)

    def _print_targets(self, clear=False):
        """Print information about vision targets found."""
        if len(self._current_targets) > 0:
            cleared = False
            clear_next = False
            if clear and not cleared:
                clear_next = True
                cleared = True
            for trg in self._current_targets:
                message = "Dis: %(dis)4.1f Ang: %(ang)4.1f" % {
                                                      'dis':trg.distance,
                                                      'ang':trg.angle}
                self._user_interface.output_user_message(message, clear_next)
                message = "Hot: %(hot)s Side: %(side)1.0f" % {
                                                    'hot':str(trg.is_hot),
                                                    'side':trg.side}
                self._user_interface.output_user_message(message, False)

    def _check_ui_print_timeout(self):
        """Show any non-range message on the screen for 2 seconds."""
        # If we're currently suppressing the range prints, check if the timeout
        # is over
        if self._disable_range_print:
            elapsed_time = self._range_print_timer.elapsed_time_in_secs()
            # TODO: this should be a parameter
            if elapsed_time > 3.0:
                self._range_print_timer.stop()
                self._disable_range_print = False

    def _check_tele_auto_kill(self):
        """Check kill switch for all tele-auto functionality."""
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.BACK) == 1):
            self._hold_to_shoot_step = -1
            self._prep_for_feed_step = -1
            self._truss_pass_step = -1
            self._aim_at_target_step = -1

    def _check_alternate_speed_modes(self):
        """Check for alternate speed mode."""
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.DRIVER,
                        userinterface.JoystickButtons.LEFTBUMPER) == 1):
            self._driver_alternate = True
        else:
            self._driver_alternate = False

    def _check_ignore_limits(self):
        """Check if encoder soft limits should be ignored."""
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.START) == 1):
            self._shooter.ignore_encoder_limits(True)
        else:
            self._shooter.ignore_encoder_limits(False)

    def _check_swap_drivetrain_request(self):
        """Check if the driver wants to swap forward and reverse."""
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.DRIVER,
                        userinterface.JoystickButtons.RIGHTTRIGGER) == 1 and
            self._user_interface.button_state_changed(
                        userinterface.UserControllers.DRIVER,
                        userinterface.JoystickButtons.RIGHTTRIGGER)):
            # Swap the driver controls to be the opposite of what they were
            self._driver_controls_swap_ratio = (self._driver_controls_swap_ratio
                                                * -1.0)
            # Print the notification for several seconds
            self._disable_range_print = True
            self._range_print_timer.start()
            if self._driver_controls_swap_ratio > 0:
                self._user_interface.output_user_message(("Controls "
                                                          "normal"),
                                                         True)
            else:
                self._user_interface.output_user_message(("Controls "
                                                          "swapped"),
                                                         True)

    def _control_drive_train(self):
        """Manually control the drive train."""
        driver_left_y = self._user_interface.get_axis_value(
                userinterface.UserControllers.DRIVER,
                userinterface.JoystickAxis.LEFTY)
        driver_right_x = self._user_interface.get_axis_value(
                userinterface.UserControllers.DRIVER,
                userinterface.JoystickAxis.RIGHTX)
        if driver_left_y != 0.0 or driver_right_x != 0.0:
            # Abort any relevent teleop auto routines
            if self._drive_train:
                driver_left_y = driver_left_y * self._driver_controls_swap_ratio
                self._drive_train.arcade_drive(driver_left_y, driver_right_x,
                                               False)
                self._aim_at_target_step = -1
        else:
            # Make sure we don't mess with any teleop auto routines
            # if they're running
            if self._aim_at_target_step < 0:
                self._drive_train.arcade_drive(0.0, 0.0, False)

    def _control_shooter(self):
        """Manually control the catapult."""
        scoring_left_y = self._user_interface.get_axis_value(
                userinterface.UserControllers.SCORING,
                userinterface.JoystickAxis.LEFTY)
        if scoring_left_y != 0.0:
            if self._shooter:
                self._shooter.move_shooter(scoring_left_y)
                # Abort any relevent teleop auto routines
                self._hold_to_shoot_step = -1
                self._prep_for_feed_step = -1
                self._truss_pass_step = -1
        else:
            # Make sure we don't mess with any teleop auto routines
            # if they're running
            if (self._hold_to_shoot_step == -1 and
                self._prep_for_feed_step == -1 and
                self._truss_pass_step == -1):
                self._shooter.move_shooter(0.0)

    def _control_feeder(self):
        """Manually control the feeder."""
        scoring_right_y = self._user_interface.get_axis_value(
                userinterface.UserControllers.SCORING,
                userinterface.JoystickAxis.RIGHTY)

        # Toggle feeder arms
        if (self._user_interface.get_button_state(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.RIGHTBUMPER) == 1
            and self._user_interface.button_state_changed(
                        userinterface.UserControllers.SCORING,
                        userinterface.JoystickButtons.RIGHTBUMPER)):
            if self._current_feeder_position == common.Direction.UP:
                self._current_feeder_position = common.Direction.DOWN
            else:
                self._current_feeder_position = common.Direction.UP
            self._feeder.set_feeder_position(self._current_feeder_position)
            # Abort any relevent teleop auto routines
            self._prep_for_feed_step = -1

        # Manually control feeder motors
        if scoring_right_y != 0.0:
            if self._feeder:
                direction = feeder.Direction.STOP
                if scoring_right_y > 0:
                    direction = feeder.Direction.IN
                else:
                    direction = feeder.Direction.OUT
                self._feeder.feed(direction, math.fabs(scoring_right_y))
                # Abort any relevent teleop auto routines
        else:
            # Make sure we don't mess with any teleop auto routines
            # if they're running
            self._feeder.feed(feeder.Direction.STOP, 0.0)


def run():
    """Create the robot and start it."""
    robot = MyRobot()
    robot.StartCompetition()
