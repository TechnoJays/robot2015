"""This module provides the connection to the user interface elements."""


# Imports
import wpilib
import os
import common
import logging
import logging.config
import parameters


class JoystickAxis(object):
    """Enumerates joystick axis."""
    LEFTX = 0
    LEFTY = 1
    RIGHTX = 2
    RIGHTY = 3
    DPADX = 5
    DPADY = 6


class JoystickButtons(object):
    """Enumerates joystick buttons."""
    #X = 0 ??
    #A = 1 ??
    #B = 2 ??
    #Y = 3 ??
    LEFTBUMPER = 4 #??
    #RIGHTBUMPER = 5 ??
    RIGHTTRIGGER = 5
    LEFTTRIGGER = 6
    #BACK = 8 ??
    #START = 9 ??

class UserControllers(object):
    """Enumerates the controllers."""
    DRIVER = 0
    SCORING = 1


class UserInterface(object):
    """Provides the user interface connections."""

    _log = None
    _parameters = None

    _controller_1 = None
    _controller_1_buttons = 0
    _controller_1_previous_button_state = 0
    _controller_1_dead_band = 0.0

    _controller_2 = None
    _controller_2_buttons = 0
    _controller_2_previous_button_state = 0
    _controller_2_dead_band = 0.0

    _parameters_file = None

    _robot_state = None
    _display_line = 0
    _log_enabled = False

    def __init__(self, params="/home/lvuser/par/userinterface.par",
                 logging_enabled=False):
        """Create and initialize a UserInterface.

        Args:
            params: The parameters filename to use for configuration
            logging_enabled: True if logging should be enabled

        """
        self._initialize(params, logging_enabled)

    def dispose(self):
        """Dispose of a UserInterface object.

        Dispose of the UserInterface object

        """
        self._log = None
        self._parameters = None
        self._controller_1 = None
        self._controller_2 = None

    def _initialize(self, params, logging_enabled):
        """Initialize and configure the UserInterface object.

        Initialize instance variables to defaults, read parameter values from
        the specified file, instantiate required objects and update status
        variables.

        Args:
            params: The parameters filename to use for configuration.
            logging_enabled: True if logging should be enabled.

        """
        # Initialize public member variables

        # Intialize private member objects
        self._log = None
        self._parameters = None
        self._controller_1 = None
        self._controller_2 = None
        self._controller_1_previous_button_state = []
        self._controller_2_previous_button_state = []

        # Initialize private parameters
        self._controller_1_buttons = 4
        self._controller_2_buttons = 4
        self._controller_1_dead_band = 0.05
        self._controller_2_dead_band = 0.05

        # Initialize private member variables
        self._display_line = 0
        self._log_enabled = False
        self._robot_state = common.ProgramState.DISABLED
        self._parameters_file = None

        if logging_enabled:
            #Create a new data log object
            self._log = logging.getLogger('userinterface')
            self._log.setLevel(logging.DEBUG)
            fh = logging.FileHandler('/home/lvuser/log/userinterface.log')
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
        self._parameters_file = os.path.realpath(params)
        self.load_parameters()

    def load_parameters(self):
        """Load values from a parameter file and create and initialize objects.

        Read parameter values from the specified file, instantiate required
        objects, and update status variables.

        Returns:
            True if the parameter file was processed successfully.

        """
        # Define and Initialize local variables
        controller_1_port = 1
        controller_2_port = 2
        controller_1_axis = 2
        controller_2_axis = 2
        param_reader = None
        section = __name__.lower()

        # Close and delete old objects
        self._parameters = None
        self._controller_1 = None
        self._controller_2 = None
        self._controller_1_previous_button_state = []
        self._controller_2_previous_button_state = []

        # Read the parameters file
        self._parameters = parameters.Parameters(self._parameters_file)

        if self._parameters is not None:
            controller_1_port = self._parameters.get_value(section,
                                                       "CONTROLLER_1_PORT")
            controller_2_port = self._parameters.get_value(section,
                                                       "CONTROLLER_2_PORT")
            controller_1_axis = self._parameters.get_value(section,
                                                       "CONTROLLER_1_AXIS")
            controller_2_axis = self._parameters.get_value(section,
                                                       "CONTROLLER_2_AXIS")

            self._controller_1_buttons = self._parameters.get_value(section,
                                                        "CONTROLLER1_BUTTONS")
            self._controller_2_buttons = self._parameters.get_value(section,
                                                        "CONTROLLER2_BUTTONS")

            self._controller_1_dead_band = self._parameters.get_value(section,
                                                      "CONTROLLER1_DEAD_BAND")
            self._controller_2_dead_band = self._parameters.get_value(section,
                                                      "CONTROLLER2_DEAD_BAND")
            # Initialize previous button state lists
            self._controller_1_previous_button_state = ([0] *
                                            (self._controller_1_buttons + 1))
            self._controller_2_previous_button_state = ([0] *
                                            (self._controller_2_buttons + 1))

            # Initialize  controller objects
            self._controller_1 = wpilib.Joystick(controller_1_port,
                                                 controller_1_axis,
                                                 self._controller_1_buttons)
            self._controller_2 = wpilib.Joystick(controller_2_port,
                                                 controller_2_axis,
                                                 self._controller_2_buttons)

        if self._log_enabled:
            if self._controller_1 is not None:
                self._log.debug("Controller 1 created")
            else:
                self._log.debug("Controller 1 not created")
            if self._controller_2 is not None:
                self._log.debug("Controller 2 created")
            else:
                self._log.debug("Controller 2 not created")



    def set_robot_state(self, state):
        """Set the current state of the robot and perform any actions
        necessary during mode changes.

        Args:
            state: current robot state

        """
        self._robot_state = state

    def set_log_state(self, state):
        """Enable or disable logging for this object.

            Args:
                state: true if logging should be enabled
        """
        if state:
            self._log_enabled = True
        else:
            self._log_enabled = False

    def button_state_changed(self, controller, button):
        """Check if the button state for the specified controller/button has
        changed since the last "Store".

        Args:
            controller: the controller to read the button state from
            button: the button ID to read the state from

        Return:
            true if the button state has changed

        """
        # Get the current button state
        current_state = self.get_button_state(controller, button)
        previous_state = 0

        if controller == UserControllers.DRIVER:
            previous_state = self._controller_1_previous_button_state[button]
        elif controller == UserControllers.SCORING:
            previous_state = self._controller_2_previous_button_state[button]

        if current_state != previous_state:
            return True
        else:
            return False

    def get_axis_value(self, controller, axis):
        """Read the current axis value for the specified controller/axis.

        Args:
            controller: the controller to read the axis value from
            axis: the axis ID to read

        Return:
            the current position fo the specified axis

        """
        # Get the current axis value from the controller
        value = 0.0

        if controller == UserControllers.DRIVER:
            if self._controller_1:
                if axis == JoystickAxis.DPADX:
                    value = self._controller_1.getPOV()
                    if value == 90:
                        value = 1.0
                    elif value == 270:
                        value = -1.0
                    else:
                        value = 0.0
                elif axis == JoystickAxis.DPADY:
                    value = self._controller_1.getPOV()
                    if value == 0:
                        value = -1.0
                    elif value == 180:
                        value = 1.0
                    else:
                        value = 0.0
                else:
                    value = self._controller_1.getRawAxis(axis)
                if abs(value) < self._controller_1_dead_band:
                    return 0.0
                return value
        elif controller == UserControllers.SCORING:
            if self._controller_2:
                if axis == JoystickAxis.DPADX:
                    value = self._controller_2.getPOV()
                    if value == 90:
                        value = 1.0
                    elif value == 270:
                        value = -1.0
                    else:
                        value = 0.0
                elif axis == JoystickAxis.DPADY:
                    value = self._controller_2.getPOV()
                    if value == 0:
                        value = -1.0
                    elif value == 180:
                        value = 1.0
                    else:
                        value = 0.0
                else:
                    value = self._controller_2.getRawAxis(axis)
                if abs(value) < self._controller_2_dead_band:
                    return 0.0
                return value
        return 0.0

    def get_button_state(self, controller, button):
        """Read the button state for the specified controller/button

        Args:
            controller: the controller to read the button state from
            button: the button ID to read the state from

        Return:
            1 if button is currently pressed

        """
        if controller == UserControllers.DRIVER:
            if self._controller_1:
                return self._controller_1.getRawButton(button)
            else:
                return 0
        elif controller == UserControllers.SCORING:
            if self._controller_2:
                return self._controller_2.getRawButton(button)
            else:
                return 0
        return 0

    def store_button_states(self, controller):
        """Store the current button states for the specified controller.

        Args:
            controller: the controller to read the button states from

        """
        button_state = 0
        button_count = 0

        # Get the total number of buttons for the controller
        button_count = {
                        0: self._controller_1_buttons,
                        1: self._controller_2_buttons
                        }.get(controller, 0)

        # Store the current state of each button for this controller
        # +1 is used in array indexing since #defines and GeRawButton start at
        # 1, and array starts at 0.
        for i in range(button_count):
            button_state = self.get_button_state(controller, i + 1)

            if controller == UserControllers.DRIVER:
                self._controller_1_previous_button_state[i + 1] = button_state

            if controller == UserControllers.SCORING:
                self._controller_2_previous_button_state[i + 1] = button_state
