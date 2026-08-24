import os
import sys
import unittest


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, REPO_ROOT)

from klippy.extras.filabuffer import (
    FilaBuffer, FilaFeeder, FilaMotor, ERROR_FEED_SLIP,
    FEEDER_ACTIVE, MODE_WORK)


class FilaMotorRampTest(unittest.TestCase):
    def test_ramp_segments_increase_by_target_fraction(self):
        segments = FilaMotor.build_speed_segments(
            10., 40., segment_time=0.1, accel_fraction=0.25)

        self.assertEqual(
            [(round(duration, 3), round(speed, 3))
             for duration, speed in segments],
            [(0.1, 5.), (0.1, 10.), (0.1, 20.), (0.1, 30.), (0.087, 40.)])

    def test_ramp_segments_shorten_final_segment(self):
        segments = FilaMotor.build_speed_segments(
            0.2, 40., segment_time=0.1, accel_fraction=0.25)

        self.assertEqual(len(segments), 1)
        self.assertAlmostEqual(segments[0][0], 0.04)
        self.assertAlmostEqual(segments[0][1], 5.)

    def test_calc_run_mm_follows_half_then_full_steps(self):
        self.assertAlmostEqual(
            FilaMotor.calc_run_mm(0., 0.1, 10., 40.), 0.5)
        self.assertAlmostEqual(
            FilaMotor.calc_run_mm(0., 0.2, 10., 40.), 1.5)
        self.assertAlmostEqual(
            FilaMotor.calc_run_mm(1., 1.35, 10., 40.), 5.0)


class FeedMatchTest(unittest.TestCase):
    def _make_feeder(self, total_mm=0., skip=False, printing=True):
        feeder = object.__new__(FilaFeeder)
        feeder.motor = FakeMotor(total_mm)
        feeder.feed_match_extruder_base = 0.
        feeder.feed_match_feeder_base = 0.
        feeder.feed_match_skip = skip
        feeder._buffer_present = False
        feeder.fb = FakeBuffer(printing=printing)
        return feeder

    def test_skips_first_unbuffered_feed_until_full_then_resets_base(self):
        feeder = self._make_feeder(0., skip=True)

        FilaFeeder.note_feed_match_select(feeder, 10.)
        feeder.motor.total_mm = 80.

        self.assertIsNone(FilaFeeder.check_feed_match(feeder, 40., 30.))

        FilaFeeder.note_feed_match_full(feeder, 40.)

        self.assertFalse(feeder.feed_match_skip)
        self.assertEqual(feeder.feed_match_feeder_base, 80.)
        self.assertEqual(feeder.feed_match_extruder_base, 40.)

    def test_detects_feed_slip_when_feeder_ahead(self):
        feeder = self._make_feeder(100., skip=False)
        feeder.feed_match_extruder_base = 50.
        feeder.feed_match_feeder_base = 100.
        feeder.motor.total_mm = 160.

        self.assertEqual(
            FilaFeeder.check_feed_match(feeder, 70., 30.), ERROR_FEED_SLIP)

    def test_skips_check_when_not_printing(self):
        feeder = self._make_feeder(160., skip=False, printing=False)
        feeder.feed_match_extruder_base = 50.
        feeder.feed_match_feeder_base = 100.

        self.assertIsNone(FilaFeeder.check_feed_match(feeder, 70., 30.))
        self.assertFalse(feeder.feed_match_skip)

    def test_watchdog_detects_slip_without_full_signal(self):
        feeder = self._make_feeder(100., skip=False)
        feeder.feeder_state = FEEDER_ACTIVE
        feeder.feed_match_extruder_base = 50.
        feeder.feed_match_feeder_base = 100.
        feeder.motor.total_mm = 160.
        fb = object.__new__(FilaBuffer)
        fb.mode = MODE_WORK
        fb.feed_match_tolerance = 30.
        fb.feed_idle_time = 10.
        fb._not_full_idle_since = 0.
        fb.sensors = FakeSensors(0)
        fb.reactor = FakeReactor()
        fb._active = lambda: feeder
        fb._get_extruded_mm = lambda eventtime=None: 70.
        fb._start_feeder_feed = lambda feeder, speed, length: None
        fb._enter_error = lambda msg: setattr(fb, 'error_msg', msg)

        FilaBuffer._check_not_full_idle_feed(fb, 1.)

        self.assertEqual(fb.error_msg, ERROR_FEED_SLIP)

    def test_watchdog_skips_initial_unbuffered_feed_until_full(self):
        feeder = self._make_feeder(100., skip=True)
        feeder.feeder_state = FEEDER_ACTIVE
        feeder.feed_match_extruder_base = 50.
        feeder.feed_match_feeder_base = 100.
        feeder.motor.total_mm = 200.
        fb = object.__new__(FilaBuffer)
        fb.mode = MODE_WORK
        fb.feed_match_tolerance = 30.
        fb.feed_idle_time = 10.
        fb._not_full_idle_since = 0.
        fb.sensors = FakeSensors(0)
        fb._active = lambda: feeder
        fb._get_extruded_mm = lambda eventtime=None: 50.
        fb._start_feeder_feed = lambda feeder, speed, length: None
        fb._enter_error = lambda msg: setattr(fb, 'error_msg', msg)

        FilaBuffer._check_not_full_idle_feed(fb, 1.)

        self.assertFalse(hasattr(fb, 'error_msg'))


class FakeBuffer:
    def __init__(self, printing=True):
        self._printing = printing

    def is_printing(self):
        return self._printing


class FakeMotor:
    def __init__(self, total_mm):
        self.total_mm = total_mm

    def get_total_mm(self):
        return self.total_mm

    def get_position_mm(self):
        return self.total_mm

    def get_move_mm(self):
        return 0.

    def is_moving(self):
        return False


class FakeSensors:
    def __init__(self, state):
        self.state = state


class FakeReactor:
    def monotonic(self):
        return 0.


if __name__ == '__main__':
    unittest.main()
