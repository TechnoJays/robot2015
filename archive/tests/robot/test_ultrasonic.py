"""This module tests the ultrasonic module.

    Packages(s) required:
    - pytest

"""

# Imports
import pytest
import ultrasonic


class TestRangeFinder:
    """Test the RangeFinder class."""

    def setup_method(self, method):
        """Setup each test."""
        pass

    def test_constructor_no_arguments(self):
        r = None
        try:
            r = ultrasonic.RangeFinder()
        except Exception:
            pass
        assert not r

    def test_constructor_with_arguments(self):
        r = ultrasonic.RangeFinder(1)
        assert r._channel
        assert r._volts_per_inch == 5 / 512.0

    def test_get_voltage(self):
        r = ultrasonic.RangeFinder(2)
        assert r.get_voltage() == 0.0

    def test_get_range_in_inches(self):
        r = ultrasonic.RangeFinder(3)
        assert r.get_range_in_inches() == 0.0

    def test_get_range_in_feet(self):
        r = ultrasonic.RangeFinder(4)
        assert r.get_range_in_feet() == 0.0

