"""This module provides a class to read from config files."""


import configparser
from text_utilities import convert_to_number


class Parameters(object):
    """ Reads in a parameters file.

    Initializes parameters for robot functionality based on values provided
    in parameters file.

    Attributes:
        file_opened:    True if the parameters file is open

    """

    # Public member variables
    file_opened = False

    # Private member objects

    # Private member variables
    _file = None
    _config = None

    def __init__(self, parameters_file="parameters.par"):
        """ Open a file to read program parameters

        Instantiate the parameters reader with the passed file

        Args:
            parameters_file: The path to the file to read parameters from

        """
        self._file = None
        self._config = None
        self.file_opened = False
        self._open(parameters_file)

    def _open(self, path):
        """ Open a file with the mode "r".

         Open a file for reading parameters

         Args:
            path: Path to the file

        """
        self.file_opened = False
        if path:
            self._config = configparser.SafeConfigParser()
            try:
                self._config.read(path)
                self.file_opened = True
            except configparser.NoSectionError:
                pass
            except configparser.DuplicateSectionError:
                pass
            except configparser.DuplicateOptionError:
                pass
            except configparser.NoOptionError:
                pass
            except configparser.InterpolationDepthError:
                pass
            except configparser.InterpolationMissingOptionError:
                pass
            except configparser.InterpolationSyntaxError:
                pass
            except configparser.MissingSectionHeaderError:
                pass
            except configparser.ParsingError:
                pass
        return self.file_opened

    def _close(self):
        """ Close the file

        Close the file and mark not opened

        """
        if self._file:
            self._file.close()
            self.file_opened = False
            self._config = None

    def read_values(self, section=None):
        """ Get the configuration dictionary
        ==DEPRECATED==

        Get the configuration dictionary for a section

        Return:
            If the section is passed, return the dictionary it contains
            else, return all sections of the configuration

        """
        if not self._config:
            return None

        if section:
            return self._config.items(section)

        return self._config._sections

    def get_value(self, section, parameter):
        """ Search the configuration dictionary for the parameter

        Search the configuration dictionary for the specified parameter
        and return the associated value

        Args:
            section: The section of the config file to read the parameter from
            parameter: The parameter to read from the file

        Return:
            the parameter value that is read from the file

        """

        if not self._config:
            return None

        if section and parameter:
            try:
                read_value = self._config.get(section, parameter.lower())
            except configparser.NoSectionError:
                return None
        else:
            return None

        param_value = convert_to_number(read_value)

        if param_value != None:
            return param_value
        else:
            return read_value
