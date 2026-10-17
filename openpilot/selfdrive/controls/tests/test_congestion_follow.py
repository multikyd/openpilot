import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.congestion_follow import CongestionFollow


class TestCongestionFollow(unittest.TestCase):
  def setUp(self):
    self.helper = CongestionFollow()
    self.lead = SimpleNamespace(present=True, radar=True, radarTrackId=4, modelProb=.99, dRel=12., dPath=0., vLead=0.)

  def tick(self, speed, seconds, enabled=True, normal=1.6):
    self.lead.vLead = speed
    result = None
    for _ in range(round(seconds/.05)):
      result = self.helper.update(self.lead, speed, normal, .9, enabled, .05)
    return result

  def arm(self):
    self.tick(0., 3.)
    self.tick(3., 4.)
    self.tick(0., 4.)

  def test_single_stop_keeps_normal_departure(self):
    self.tick(0., 8.)
    self.assertEqual(self.tick(3., 5.), (.9, 1.6))

  def test_repeated_waves_add_moving_reserve_and_limit_accel(self):
    self.arm()
    follow, accel = self.tick(3., 2.)
    self.assertAlmostEqual(follow, 1.15)
    self.assertGreaterEqual(accel, .75)
    self.assertLessEqual(accel, 1.1)

  def test_standstill_clearance_unchanged(self):
    self.arm()
    self.assertEqual(self.tick(0., 1.)[0], .9)

  def test_existing_lower_acceleration_limit_preserved(self):
    self.arm()
    self.assertEqual(self.tick(3., 2., normal=.3)[1], .3)
    self.assertEqual(self.tick(3., 2., normal=-.5)[1], -.5)

  def test_driver_override_clears_context(self):
    self.arm()
    self.assertEqual(self.tick(3., .05, enabled=False), (.9, 1.6))
    self.assertEqual(len(self.helper.stops), 0)

  def test_sustained_flow_clears_context(self):
    self.arm()
    self.assertEqual(self.tick(11., 15.), (.9, 1.6))
    self.assertEqual(self.tick(3., 2.), (.9, 1.6))

  def test_missing_lead_does_not_limit_acceleration(self):
    self.arm()
    self.lead.present = False
    self.assertEqual(self.tick(3., 4.), (.9, 1.6))
    self.assertFalse(self.helper.stops)

  def test_cut_in_requires_confirmation(self):
    self.arm()
    self.lead.radarTrackId = 8
    self.assertEqual(self.tick(3., .5), (.9, 1.6))

  def test_invalid_lead_rejected(self):
    self.arm()
    self.lead.dRel = float('nan')
    self.assertEqual(self.tick(3., 1.), (.9, 1.6))

  def test_continuous_stop_is_one_event(self):
    self.tick(0., 80.)
    self.assertEqual(len(self.helper.stops), 1)
    self.assertEqual(self.helper.strength, 0.)


if __name__ == '__main__':
  unittest.main()
