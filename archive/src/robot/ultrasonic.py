"""This module provides a class to read a range finder."""


# Imports
try:
    import wpilib
except ImportError:
    from pyfrc import wpilib


class RangeFinder(object):
    """Get the distance to the nearest object."""
    # TODO: change hard coded values to come from parameters

    _volts_per_inch = None
    _channel = None
    _range_list = None
    _filtered_read_count = None

    def __init__(self, chan):
        self._channel = wpilib.AnalogChannel(chan)
        self._channel.SetOversampleBits(4)
        self._channel.SetAverageBits(4)
        self._volts_per_inch = 5 / 512.0
        self._range_list = []
        self._filtered_read_count = 0

    def get_voltage(self):
        """Get the voltage from the analog channel."""
        return self._channel.GetAverageVoltage()

    def get_range_in_inches(self):
        """Get the range to the nearest object in inches."""
        rng = self.get_voltage()
        rng = rng / self._volts_per_inch
        return rng

    def get_range_in_feet(self):
        """Get the range to the nearest object in feet."""
        rng = self.get_range_in_inches() / 12.0
        return rng

    def get_filtered_range_in_feet(self):
        """Get the median filtered range in feet."""
        self._filtered_read_count += 1
        current_range = self.get_range_in_feet()
        # Only store the range in the range list if it's greater than 1.0 and
        # every 5th reading.
        # The reason to only store every 5th range is to add a time delay in
        # between readings
        if current_range > 1.0 and self._filtered_read_count >= 5:
            if len(self._range_list) >= 21:
                self._range_list.pop(0)
            self._range_list.append(current_range)
            self._filtered_read_count = 0
        # If the range list is full of data, return the median range
        if len(self._range_list) > 20:
            # Copy and sort the range list
            sorted_ranges = list(self._range_list)
            sorted_ranges.sort()
            return sorted_ranges[18]
        # Otherwise just return the current reading
        else:
            return current_range

