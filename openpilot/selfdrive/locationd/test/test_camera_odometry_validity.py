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
