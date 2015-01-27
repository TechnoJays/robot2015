"""This module provides a class to read autoscript files.

Packages required:
    - os
    - csv
    - glob
    - text_utilities

"""

# Imports
import os
import csv
import glob
from text_utilities import convert_to_number


class AutoScriptCommand(object):
    """Object that stores the information for an autoscript command.

    Attributes:
        command: the autonomous command.
        parameters: the List of parameters for the command.

    """
    # Public member variables
    command = None
    parameters = None

    def __init__(self, c, p):
        """Create and initialize an AutoScriptCommand.

        Args:
            c: the autonomous command.
            p: the List of parameters for the command.

        """
        self.command = c
        self.parameters = p


class AutoScript(object):
    """Reads autonomous robot sequences from a file into memory."""

    # Public member variables

    # Private member objects

    # Private member variables
    _commands = None
    _command_iterator = None

    def __init__(self, path_and_file=None):
        """Create and initialize an AutoScript object with a specified file.

        Instantiate an AutoScript object and parse a specified script file.

        Args:
            path_and_file: the path and filename of the autoscript file.

        """
        if path_and_file:
            self.parse(path_and_file)

    def dispose(self):
        """Dispose of an AutoScript object."""
        self._commands = None

    def parse(self, path_and_file):
        """Read all autoscript commands from the file.

        Reads the entire autoscript file formatted as a comma separated value
        (CSV) file.  The commands are stored as objects in a List.

        Args:
            path_and_file: the path and filename of the autoscript file.

        Returns:
            The list of AutoScriptCommand objects.

        """
        # Clear out any old data
        self._commands = []

        try:
            with open(path_and_file, 'r') as asfile:
                csvreader = csv.reader(asfile, delimiter=',')
                for row in csvreader:
                    cmd = None
                    params = []
                    for column in row:
                        if not cmd:
                            cmd = column
                        else:
                            num = convert_to_number(column)
                            if num:
                                params.append(num)
                            else:
                                params.append(column)
                    if cmd:
                        command = AutoScriptCommand(cmd, params)
                        self._commands.append(command)
        except (OSError, IOError):
            self._commands = None
            return self._commands

        self._command_iterator = iter(self._commands)
        return self._commands

    def get_available_scripts(self, path=None):
        """Get a list of autoscript files in the current directory.

        Returns:
            A List of AutoScript filenames.

        """
        if path:
            return glob.glob(os.path.realpath(path + '/*.as'))
        else:
            return glob.glob('*.as')

    def get_next_command(self):
        """Get the next AutoScript command.

        Returns:
            The next AutoScriptCommand from the file.

        """
        cmd = None
        if self._command_iterator:
            try:
                cmd = next(self._command_iterator)
            except StopIteration:
                cmd = None
        return cmd

