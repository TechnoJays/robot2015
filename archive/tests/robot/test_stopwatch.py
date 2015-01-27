"""This module tests the stopwatch module.

    Packages(s) required:
    - pytest

"""

# Imports
import pytest
import stopwatch


class TestStopwatch:
    """Test the Stopwatch class."""

    def setup_method(self, method):
        """Setup each test."""
        self._sw = stopwatch.Stopwatch()

    def test_constructor(self):
        assert self._sw._running == False
        assert self._sw._start == None
        assert self._sw._end == None
        assert self._sw._secs == None
        assert self._sw._msecs == None

    def test_start_first_time(self):
        self._sw.start()
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._secs == None
        assert self._sw._msecs == None

    def test_two_starts_in_a_row(self):
        self._sw.start()
        start1 = self._sw._start
        self._sw.start()
        start2 = self._sw._start
        assert start1
        assert start2
        assert isinstance(start1, float)
        assert isinstance(start2, float)
        assert start2 > start1
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._secs == None
        assert self._sw._msecs == None

    def test_start_and_reset(self):
        self._sw.start()
        start1 = self._sw._start
        self._sw.reset()
        start2 = self._sw._start
        assert start1
        assert start2
        assert isinstance(start1, float)
        assert isinstance(start2, float)
        assert start2 > start1
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._secs == None
        assert self._sw._msecs == None

    def test_reset_no_start(self):
        self._sw.reset()
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        assert self._sw._running == False
        assert self._sw._end == None
        assert self._sw._secs == None
        assert self._sw._msecs == None

    def test_stop_no_start(self):
        self._sw.stop()
        assert self._sw._end == None
        assert self._sw._secs == None
        assert self._sw._msecs == None

    def test_start_and_stop(self):
        self._sw.start()
        assert self._sw._running == True
        assert self._sw._end == None
        self._sw.stop()
        assert self._sw._running == False
        assert self._sw._end
        assert isinstance(self._sw._end, float)

    def test_start_and_two_stops(self):
        self._sw.start()
        assert self._sw._running == True
        assert self._sw._end == None
        self._sw.stop()
        end1 = self._sw._end
        assert self._sw._running == False
        assert self._sw._end
        assert isinstance(self._sw._end, float)
        self._sw.stop()
        end2 = self._sw._end
        assert self._sw._running == False
        assert self._sw._end
        assert isinstance(self._sw._end, float)
        assert end1 == end2

    def test_start_stop_and_reset(self):
        self._sw.start()
        start1 = self._sw._start
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        self._sw.stop()
        assert self._sw._running == False
        assert self._sw._end
        assert isinstance(self._sw._end, float)
        self._sw.reset()
        start2 = self._sw._start
        assert self._sw._end == None
        assert start1 != start2
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        assert self._sw._running == False

    def test_elapsed_secs_while_not_running(self):
        assert self._sw.elapsed_time_in_secs() == None

    def test_elapsed_secs_after_start(self):
        self._sw.start()
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        val = self._sw.elapsed_time_in_secs()
        assert self._sw._running == True
        assert self._sw._end
        assert isinstance(self._sw._end, float)
        assert val
        assert isinstance(val, float)
        assert val > 0

    def test_elapsed_secs_after_stop(self):
        self._sw.start()
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        self._sw.stop()
        assert self._sw._running == False
        assert self._sw._end
        assert isinstance(self._sw._end, float)
        val = self._sw.elapsed_time_in_secs()
        assert self._sw._running == False
        assert self._sw._end
        assert self._sw._secs
        assert isinstance(self._sw._end, float)
        assert val
        assert isinstance(val, float)
        assert val > 0

    def test_elapsed_secs_after_reset(self):
        self._sw.start()
        start1 = self._sw._start
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        self._sw.reset()
        start2 = self._sw._start
        assert self._sw._end == None
        assert start1 != start2
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        assert self._sw._running == True
        val = self._sw.elapsed_time_in_secs()
        assert self._sw._running == True
        assert self._sw._end
        assert self._sw._secs
        assert isinstance(self._sw._end, float)
        assert val
        assert isinstance(val, float)
        assert val > 0

    def test_elapsed_msecs_while_not_running(self):
        assert self._sw.elapsed_time_in_msecs() == None

    def test_elapsed_msecs_after_start(self):
        self._sw.start()
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        val = self._sw.elapsed_time_in_msecs()
        assert self._sw._running == True
        assert self._sw._end
        assert isinstance(self._sw._end, float)
        assert val
        assert isinstance(val, float)
        assert val > 0

    def test_elapsed_msecs_after_stop(self):
        self._sw.start()
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        self._sw.stop()
        assert self._sw._running == False
        assert self._sw._end
        assert isinstance(self._sw._end, float)
        val = self._sw.elapsed_time_in_msecs()
        assert self._sw._running == False
        assert self._sw._end
        assert self._sw._secs
        assert self._sw._msecs
        assert isinstance(self._sw._end, float)
        assert val
        assert isinstance(val, float)
        assert val > 0

    def test_elapsed_msecs_after_reset(self):
        self._sw.start()
        start1 = self._sw._start
        assert self._sw._running == True
        assert self._sw._end == None
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        self._sw.reset()
        start2 = self._sw._start
        assert self._sw._end == None
        assert start1 != start2
        assert self._sw._start
        assert isinstance(self._sw._start, float)
        assert self._sw._running == True
        val = self._sw.elapsed_time_in_msecs()
        assert self._sw._running == True
        assert self._sw._end
        assert self._sw._secs
        assert self._sw._msecs
        assert isinstance(self._sw._end, float)
        assert val
        assert isinstance(val, float)
        assert val > 0

