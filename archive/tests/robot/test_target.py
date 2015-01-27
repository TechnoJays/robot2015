"""This module tests the target module.

    Packages(s) required:
    - pytest

"""

# Imports
import pytest
import target


class TestTarget:
    """Test the Target class."""

    def setup_method(self, method):
        """Setup each test."""
        pass

    def test_full_constructor(self):
        values = {'side':1, 'distance':10.1, 'angle':-5.0, 'is_hot':True, 'confidence':80.0}
        t = target.Target(**values)
        assert t.side == values['side']
        assert t.distance == values['distance']
        assert t.angle == values['angle']
        assert t.is_hot == values['is_hot']
        assert t.confidence == values['confidence']
