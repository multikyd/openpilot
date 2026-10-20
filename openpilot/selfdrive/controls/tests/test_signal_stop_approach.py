import math
import unittest

from openpilot.selfdrive.controls.lib.signal_stop_approach import SignalStopApproach, SignalStopOverride


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


class TestStopCorrection(unittest.TestCase):
 def update(self,g,d,v=4):return g.update(d,v,0.,2.16,4.6,True,.05)
 def test_stable_outward_correction_is_adopted(self):
  g=SignalStopApproach();self.update(g,40)
  for i in range(100):self.update(g,42-4*(i+1)*.05)
  self.assertGreater(g.extension_used,1.);self.assertLessEqual(g.extension_used,2.)
 def test_single_frame_outlier_cannot_extend(self):
  g=SignalStopApproach();self.update(g,40)
  self.update(g,43.8);self.assertEqual(g.extension_used,0.)
  self.update(g,39.6);self.assertEqual(g.extension_used,0.)
 def test_large_step_is_rejected(self):
  g=SignalStopApproach();self.update(g,40)
  for i in range(50):self.update(g,65-4*(i+1)*.05)
  self.assertEqual(g.extension_used,0.)
 def test_total_extension_is_bounded(self):
  g=SignalStopApproach();self.update(g,100)
  for i in range(450):self.update(g,116-4*(i+1)*.05)
  self.assertGreater(g.extension_used,10.);self.assertLessEqual(g.extension_used,12.)
 def test_nearer_target_remains_immediate(self):
  g=SignalStopApproach();self.update(g,40);self.assertEqual(self.update(g,12),12)
 def test_standstill_does_not_move_target(self):
  g=SignalStopApproach();self.update(g,10,v=0)
  for i in range(100):self.update(g,12,v=0)
  self.assertEqual(g.extension_used,0.);self.assertEqual(g.distance,10)
class TestPedalRecheck(unittest.TestCase):
 def run_pulse(self,duration=1,speed=3):
  o=SignalStopOverride()
  for i in range(round(duration/.05)):self.assertFalse(o.update(True,i==0,speed,True,False,False,.05))
  return o
 def test_short_release_rechecks_once(self):
  o=self.run_pulse();self.assertTrue(o.update(False,False,3,True,False,False,.05));self.assertFalse(o.update(False,False,3,True,False,False,.05))
 def test_long_or_fast_pedal_preserves_explicit_override(self):
  for duration,speed in [(2.2,3),(1,6)]:
   o=self.run_pulse(duration,speed);self.assertFalse(o.update(False,False,speed,True,False,False,.05))
 def test_green_lead_and_disabled_cancel_recheck(self):
  for enabled,lead,green in [(True,False,True),(True,True,False),(False,False,False)]:
   o=self.run_pulse();self.assertFalse(o.update(False,False,3,enabled,lead,green,.05))
 def test_normal_gas_does_not_create_a_signal_stop(self):
  o=SignalStopOverride();o.update(True,False,3,True,False,False,.05);self.assertFalse(o.update(False,False,3,True,False,False,.05))

if __name__ == '__main__':
  unittest.main()
