"""This module provides a (fork)lift class."""

# Imports

# If wpilib not available use pyfrc
try:
    import wpilib
except ImportError:
    from pyfrc import wpilib
import common
import logging
import logging.config
import parameters
import stopwatch


class Lift(object):
    """A mechanism that lifts things off of the ground.

    This class desribes a lift mechanism that uses a metal bracket attached to
    a vertical rail to lift things using their handles.

    Attributes:
        lift_enabled: True if the Lift is fully functional (default False).

    """

    # Public member variables
    lift_enabled = False

    # Private member objects
    _log = None
    _parameters = None
    _lift_controller = None
    _movement_timer = None

    # Private parameters
    _forward_direction = -1
    _backward_direction = -1
    _time_threshold = 0

    # Private member variables
    _log_enabled = False
    _parameters_file = None
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
        self.lift_enabled = False

        # Initialize private member objects
        self._log = None
        self._parameters = None
        self._lift_controller = None
        self._movement_timer = None

        # Initialize private parameters
        self._forward_direction = 0.1
        self._backward_direction = 0.1
        self._time_threshold = 0.1

        # Initialize private member variables
        self._log_enabled = False
        self._robot_state = common.ProgramState.DISABLED

        # Enable logging if specified
        if logging_enabled:
            # Create a new data log object
            self._log = logging.getLogger('lift')
            self._log.setLevel(logging.DEBUG)
            fh = logging.FileHandler('/home/lvuser/log/lift.log')
            fh.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
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
