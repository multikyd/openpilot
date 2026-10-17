import math
import unittest
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc


def lead(present=True, **kwargs):
  values = dict(present=present, radar=present, radarTrackId=33, modelProb=.99,
                dRel=12., vLead=6., dPath=0.)
  values.update(kwargs)
  return NS(**values)


def guard():
  # No solver is needed to exercise this independent acceleration-cap helper.
  p = LongitudinalMpc.__new__(LongitudinalMpc)
  p.dt = .05
  p.turn_lead_id = p.turn_reacquire_id = p.turn_loss_limit = p.turn_exit_curvature = None
  p.turn_lead_age = p.turn_lead_confirm = 0.
  p.turn_loss_remaining = p.turn_loss_clear_time = p.turn_loss_reacquire = 0.
  p.turn_loss_elapsed = p.turn_exit_confirm = 0.
  p.turn_peak_angle = p.turn_peak_curvature = 0.
  p.turn_exit_side = 0
  return p


def tick(p, *, l=None, angle=-120., curve=.08, speed=4., normal=1.6, allow=True, enabled=True):
  return p.cruise_accel_in_turn_after_lead_loss(l or lead(False), lead(False), speed, 0., angle, normal,
                                             enabled, future_curvature=curve, allow_resume=allow)


def arm(side=1):
  p=guard()
  for _ in range(20):tick(p,l=lead(),angle=-120.*side,curve=.08*side)
  for _ in range(22):tick(p,angle=-120.*side,curve=.08*side)
  return p


class TestTurnLossResume(unittest.TestCase):
  def test_initial_blind_interval_remains_held(self):
    p=guard()
    for _ in range(20):tick(p,l=lead())
    for _ in range(17):self.assertEqual(tick(p,angle=-75.,curve=.035),0.)

  def test_sustained_opening_allows_small_positive_cap_both_directions(self):
    for side in (-1,1):
      p=arm(side)
      values=[tick(p,angle=-75.*side,curve=.035*side) for _ in range(20)]
      self.assertGreater(values[-1],.15)
      self.assertLessEqual(max(values),.3)
      self.assertGreater(p.turn_loss_remaining,0.)

  def test_brief_opening_does_not_resume(self):
    p=arm()
    self.assertTrue(all(tick(p,angle=-75.,curve=.035)==0 for _ in range(5)))

  def test_steering_or_curvature_alone_is_insufficient(self):
    for angle,curve in [(-75.,.08),(-120.,.035)]:
      p=arm()
      self.assertTrue(all(tick(p,angle=angle,curve=curve)==0 for _ in range(15)))

  def test_lateral_budget_blocks_resume(self):
    p=arm()
    self.assertTrue(all(tick(p,angle=-75.,curve=.035,speed=8.)==0 for _ in range(15)))

  def test_missing_or_invalid_preview_keeps_existing_hold(self):
    for curve in (None,math.nan):
      p=arm()
      self.assertTrue(all(tick(p,angle=-75.,curve=curve)==0 for _ in range(15)))

  def test_opposite_turn_preview_or_steering_cannot_resume(self):
    for angle,curve in [(-75.,-.035),(75.,.035),(75.,-.035)]:
      p=arm()
      self.assertTrue(all(tick(p,angle=angle,curve=curve)==0 for _ in range(15)))

  def test_present_lead_even_vision_only_prevents_partial_resume(self):
    p=arm();vision=lead(radar=False,radarTrackId=-1,modelProb=.5)
    self.assertTrue(all(tick(p,l=vision,angle=-75.,curve=.035)==0 for _ in range(15)))

  def test_stop_signal_or_insufficient_speed_headroom_gate(self):
    p=arm()
    self.assertTrue(all(tick(p,angle=-75.,curve=.035,allow=False)==0 for _ in range(15)))

  def test_normal_deceleration_and_limits_win(self):
    p=arm()
    for _ in range(15):value=tick(p,angle=-75.,curve=.035,normal=.1)
    self.assertLessEqual(value,.1)
    self.assertEqual(tick(p,angle=-75.,curve=.035,normal=-.4),-.4)

  def test_tightening_removes_partial_cap(self):
    p=arm()
    for _ in range(18):tick(p,angle=-75.,curve=.035)
    before=p.turn_loss_limit
    values=[tick(p,angle=-118.,curve=.078) for _ in range(8)]
    self.assertLess(values[-1],before)
    self.assertEqual(values[-1],0.)

  def test_raise_and_restrict_rates_are_bounded(self):
    p=arm();values=[0.]+[tick(p,angle=-75.,curve=.035) for _ in range(15)]
    self.assertLessEqual(max(b-a for a,b in zip(values,values[1:])),.5*.05+1e-12)
    tighter=[values[-1]]+[tick(p,angle=-120.,curve=.08) for _ in range(8)]
    self.assertGreaterEqual(min(b-a for a,b in zip(tighter,tighter[1:])), -.8*.05-1e-12)

  def test_disengagement_clears_exit_evidence(self):
    p=arm()
    for _ in range(15):tick(p,angle=-75.,curve=.035)
    self.assertEqual(tick(p,enabled=False),1.6)
    self.assertEqual(p.turn_peak_angle,0.)
    self.assertEqual(p.turn_exit_confirm,0.)
    self.assertIsNone(p.turn_exit_curvature)

  def test_timeout_still_releases_without_preview(self):
    p=arm()
    for _ in range(140):value=tick(p,curve=None)
    self.assertEqual(value,1.6)
    self.assertIsNone(p.turn_loss_limit)


if __name__ == '__main__':
  unittest.main()
