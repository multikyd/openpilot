import math
import unittest

from openpilot.selfdrive.controls.lib.signal_stop_approach import SignalStopApproach


class TestSignalStopApproach(unittest.TestCase):
  def test_target_follows_travel_instead_of_noisy_outward_model(self):
    guard = SignalStopApproach()
    for i in range(80):
      distance = guard.update(60.+i, 8., .9, 2.16, 4.6, True, .05)
      if i >= 3:
        self.assertLessEqual(distance, 60.-7.75*i*.05+.013)
    self.assertLess(guard.accel_cap, 0.)

  def test_short_detection_does_not_add_braking(self):
    guard = SignalStopApproach()
    for _ in range(3):
      guard.update(60., 8., .9, 2.16, 4.6, True, .05)
      self.assertIsNone(guard.accel_cap)
    guard.update(60., 8., .9, 2.16, 4.6, False, .05)
    self.assertIsNone(guard.distance)

  def test_accel_ceiling_slews_into_braking(self):
    guard = SignalStopApproach()
    previous = .9
    for _ in range(100):
      guard.update(60., 8., .9, 2.16, 4.6, True, .05)
      if guard.accel_cap is not None:
        self.assertLessEqual(previous-guard.accel_cap, .03000001)
        previous = guard.accel_cap
    self.assertLess(guard.accel_cap, -.5)

  def test_nearer_target_is_accepted(self):
    guard = SignalStopApproach()
    for _ in range(10):
      guard.update(40., 8., 0., 2.16, 4.6, True, .05)
    self.assertEqual(guard.update(15., 8., 0., 2.16, 4.6, True, .05), 15.)

  def test_disable_resets_target_and_accel(self):
    guard = SignalStopApproach()
    for _ in range(10):
      guard.update(30., 8., 0., 2.16, 4.6, True, .05)
    self.assertLess(guard.accel_cap, 0.)
    self.assertEqual(guard.update(80., 8., 0., 2.16, 4.6, False, .05), 80.)
    self.assertIsNone(guard.accel_cap)
    self.assertIsNone(guard.distance)
    self.assertEqual(guard.update(80., 8., 0., 2.16, 4.6, True, .05), 80.)

  def test_invalid_input_resets(self):
    for field, value in [('distance', math.nan), ('speed', 41.), ('comfort_brake', 0.), ('buffer', 20.), ('dt', .5)]:
      with self.subTest(field=field):
        guard = SignalStopApproach()
        args = dict(distance=50., speed=8., acceleration=0., comfort_brake=2.16, buffer=4.6, enabled=True, dt=.05)
        for _ in range(10):
          guard.update(**args)
        args[field] = value
        guard.update(**args)
        self.assertIsNone(guard.accel_cap)
        self.assertIsNone(guard.distance)

  def test_standstill_does_not_add_hold_braking(self):
    guard = SignalStopApproach()
    for _ in range(10):
      guard.update(2., 0., 0., 2.16, 4.6, True, .05)
    self.assertIsNone(guard.accel_cap)


if __name__ == '__main__':
  unittest.main()
