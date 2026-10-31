import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("camera_validity", Path(__file__).resolve().parents[1] / "camera_odometry_validity.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class TestCameraOdometryValidity(unittest.TestCase):
  def setUp(self):
    self.guard = module.CameraOdometryValidity()
    self.assertTrue(self.guard.update(True, 100, 10., 10.01, True))

  def test_isolated_skip_and_recovery(self):
    self.assertTrue(self.guard.update(False, 102, 10.10, 10.11, False))
    self.assertTrue(self.guard.update(True, 103, 10.15, 10.16, True))

  def test_consecutive_invalid(self):
    self.assertTrue(self.guard.update(False, 102, 10.1, 10.11, False))
    self.assertFalse(self.guard.update(False, 104, 10.2, 10.21, False))

  def test_no_history_or_no_skipped_frame(self):
    self.assertFalse(module.CameraOdometryValidity().update(False, 102, 10.1, 10.11, False))
    self.assertFalse(self.guard.update(False, 101, 10.05, 10.06, False))

  def test_stale_or_large_frame_gap(self):
    for frame, eof, stamp in [(102,10.2,10.21),(104,10.1,10.11),(102,10.1,10.3)]:
      self.setUp()
      self.assertFalse(self.guard.update(False, frame, eof, stamp, False))

  def test_estimator_rejection_cannot_enable_grace(self):
    self.guard.update(True, 101, 10.05, 10.06, False)
    self.assertFalse(self.guard.update(False, 103, 10.15, 10.16, False))

  def test_timestamp_reset_or_nonfinite(self):
    for eof, stamp in [(9.9,10.11),(10.1,9.9),(float('nan'),10.11)]:
      self.setUp()
      self.assertFalse(self.guard.update(False, 102, eof, stamp, False))

  def test_startup_validity_preserved(self):
    self.assertTrue(module.CameraOdometryValidity().update(True, 0, 0., 0., False))


class TestCalibrationOdometryValidity(unittest.TestCase):
  def setUp(self):
    self.guard = module.CalibrationOdometryValidity()
    self.guard.update(True, 100, 10., 10.01, trusted=True, now=10.02)

  def skip(self, **kw):
    args = dict(valid=False, frame_id=102, timestamp_eof=10.1, log_time=10.11,
                trusted=True, now=10.12)
    args.update(kw)
    return self.guard.update(**args)

  def test_isolated_skip_uses_recent_trusted_calibration(self):
    self.assertTrue(self.skip())
    self.assertTrue(self.guard.can_hold(10.14, trusted=True))

  def test_second_invalid_revokes_hold(self):
    self.assertTrue(self.skip())
    self.assertFalse(self.skip(frame_id=104, timestamp_eof=10.2, log_time=10.21, now=10.22))
    self.assertFalse(self.guard.can_hold(10.23, trusted=True))

  def test_timeout_and_future_or_nonfinite_clock(self):
    for now in (10.251, 10.09, float('nan')):
      self.setUp()
      self.assertTrue(self.skip())
      self.assertFalse(self.guard.can_hold(now, trusted=True))
      self.assertFalse(self.guard.can_hold(10.13, trusted=True))

  def test_actual_calibration_or_other_input_failure_revokes_hold(self):
    self.assertTrue(self.skip())
    self.assertFalse(self.guard.can_hold(10.13, trusted=False))
    self.assertFalse(self.guard.can_hold(10.14, trusted=True))

  def test_untrusted_previous_input_cannot_enable_hold(self):
    self.guard.update(True, 101, 10.05, 10.06, trusted=False, now=10.07)
    self.assertFalse(self.skip(frame_id=103, timestamp_eof=10.15, log_time=10.16, now=10.17))

  def test_untrusted_current_input_cannot_enable_hold(self):
    self.assertFalse(self.skip(trusted=False))

  def test_old_pose_or_large_skip_cannot_enable_hold(self):
    for changes in (dict(now=10.4), dict(frame_id=103), dict(log_time=10.3),
                    dict(timestamp_eof=float('nan'))):
      self.setUp()
      self.assertFalse(self.skip(**changes))

  def test_startup_and_recovery_do_not_hold(self):
    self.guard = module.CalibrationOdometryValidity()
    self.assertFalse(self.skip())
    self.setUp()
    self.assertTrue(self.skip())
    self.assertFalse(self.guard.update(True, 103, 10.15, 10.16, trusted=True, now=10.17))
