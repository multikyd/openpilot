import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.lead_gap_reserve import LeadGapReserve, apply_lead_gap_cap


class TestLeadGapReserve(unittest.TestCase):
  def setUp(self):
    self.helper = LeadGapReserve()
    self.lead = SimpleNamespace(present=True, radar=False, radarTrackId=-1, modelProb=.99,
                                dRel=5., vRel=0., vLead=10., aLeadK=0., dPath=0.)
    self.time = 0.

  def tick(self, count=1, enabled=True, speed=10.):
    result = None
    for _ in range(count):
      self.time += .05
      result = self.helper.update(self.lead, speed, .9, 3.6, self.time, self.time, enabled)
    return result

  def test_vision_equal_speed_still_restores_short_gap(self):
    self.assertLess(self.tick(80), -1.5)

  def test_confirmation_before_new_target_braking(self):
    self.assertIsNone(self.tick(10))

  def test_decelerating_lead_builds_small_moving_reserve(self):
    self.lead.radar = True
    self.lead.radarTrackId = 41
    self.lead.dRel = 14.4
    self.lead.vRel = -.8
    self.lead.vLead = 11.2
    self.lead.aLeadK = -.35
    # Follow a physically consistent range slope, instead of a frozen obstacle.
    for _ in range(40):
      self.lead.dRel -= .8*.05
      cap = self.tick(speed=12.)
    self.assertLess(cap, -.5)

  def test_stronger_existing_braking_preserved(self):
    cap = self.tick(80)
    self.assertEqual(apply_lead_gap_cap(-3., 4., -1., cap, 10., .2, -3., .05), (-3., 4., -1.))

  def test_velocity_and_jerk_follow_added_cap(self):
    a, v, j = apply_lead_gap_cap(-.4, 10., 1., -1., 10., .2, -.9, .05)
    self.assertEqual(a, -1.)
    self.assertAlmostEqual(v, 9.8)
    self.assertAlmostEqual(j, -2.)

  def test_no_action_at_generous_gap(self):
    self.lead.dRel = 40.
    self.assertIsNone(self.tick(80))

  def test_smooth_build_and_release(self):
    self.tick(80)
    previous = self.helper.cap
    self.lead.vRel = .6
    self.lead.vLead = 10.6
    for _ in range(70):
      self.tick()
      self.assertLessEqual(abs(self.helper.cap-previous), .35*.05+1e-9)
      previous = self.helper.cap

  def test_stale_frozen_sample_clears_cap(self):
    self.tick(80)
    self.assertIsNone(self.helper.update(self.lead, 10., .9, 3.6, self.time, self.time+.21, True))

  def test_repeated_sample_does_not_integrate(self):
    self.tick(80);before = self.helper.cap
    for _ in range(20):
      self.helper.update(self.lead, 10., .9, 3.6, self.time, self.time, True)
    self.assertEqual(self.helper.cap, before)

  def test_driver_override_reset(self):
    self.tick(80)
    self.assertIsNone(self.tick(enabled=False))
    self.assertEqual(self.helper.cap, 0.)

  def test_low_confidence_or_side_target_rejected(self):
    self.lead.modelProb = .94
    self.assertIsNone(self.tick(80))
    self.lead.modelProb = .99;self.lead.dPath = 2.
    self.assertIsNone(self.tick(80))

  def test_large_distance_jump_requires_new_confirmation(self):
    self.tick(80);self.lead.dRel = 2.
    self.assertIsNone(self.tick())
    self.assertEqual(self.helper.confirm, 0.)

  def test_radar_track_switch_requires_confirmation(self):
    self.lead.radar=True;self.lead.radarTrackId=1
    self.tick(80);self.lead.radarTrackId=2
    self.assertIsNone(self.tick())

  def test_standstill_and_invalid_inputs_are_not_modified(self):
    self.assertIsNone(self.tick(80,speed=.5))
    self.lead.dRel=float('nan')
    self.assertIsNone(self.tick(80))


if __name__ == '__main__':
  unittest.main()
