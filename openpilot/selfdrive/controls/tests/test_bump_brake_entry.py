import ast
from pathlib import Path
import unittest
from openpilot.selfdrive.controls.lib.bump_brake_entry import BumpBrakeEntry


class TestBumpBrakeEntry(unittest.TestCase):
  def setUp(self):
    self.h=BumpBrakeEntry()

  def tick(self,t=100.,**kwargs):
    args=dict(now=t,distance=100.,target=30./3.6,speed=60./3.6,
              desired=-2.,previous=0.,measured=0.,active=True,enabled=True)
    args.update(kwargs)
    return self.h.update(**args)

  def test_large_target_drop_starts_with_bounded_acceleration_change(self):
    a=self.tick()
    self.assertAlmostEqual(a,-.075)
    prev=a
    for i in range(1,40):
      a=self.tick(100.+i*.05,previous=prev,measured=prev)
      self.assertGreaterEqual(a-prev,-1.5*.05-1e-9)
      prev=a
    self.assertAlmostEqual(a,-2.)
    self.assertTrue(self.h.done)

  def test_late_bump_keeps_full_braking_immediately(self):
    self.assertEqual(self.tick(distance=20.),-2.)
    self.assertTrue(self.h.done)

  def test_existing_or_new_lead_priority_cannot_rearm_mid_bump(self):
    self.tick()
    self.assertEqual(self.tick(100.05,enabled=False),-2.)
    self.assertEqual(self.tick(100.10),-2.)

  def test_non_bump_has_no_effect(self):
    self.assertEqual(self.tick(active=False),-2.)
    self.assertFalse(self.h.seen)

  def test_bump_passage_releases_without_acceleration_delay(self):
    self.tick()
    self.assertEqual(self.tick(100.05,active=False,desired=1.),1.)

  def test_already_at_target_does_not_soften(self):
    self.assertEqual(self.tick(speed=8.),-2.)

  def test_waits_for_actual_brake_entry(self):
    self.assertEqual(self.tick(desired=0.),0.)
    self.assertFalse(self.h.running)
    self.assertAlmostEqual(self.tick(100.05),-.075)

  def test_new_bump_gets_separate_entry(self):
    self.tick(distance=20.)
    self.assertGreater(self.tick(100.05,distance=120.),-2.)

  def test_stale_or_invalid_inputs_never_soften(self):
    self.tick()
    self.assertEqual(self.tick(102.),-2.)
    self.setUp()
    self.assertEqual(self.tick(distance=float('nan')),-2.)

  def test_idealized_60_to_30_approach_keeps_distance_reserve(self):
    speed=60./3.6;target=30./3.6;distance=82.;accel=0.;dt=.05
    for i in range(160):
      before=speed
      accel=self.h.update(100.+i*dt,distance,target,speed,-2.,accel,accel,
                          active=True,enabled=True)
      speed=max(target,speed+accel*dt)
      distance-=(before+speed)*.5*dt
      if speed<=target+1e-6:break
    self.assertAlmostEqual(speed,target)
    self.assertGreater(distance,3.)

  def test_planner_lead_and_stop_caps_remain_after_entry(self):
    p=Path(__file__).resolve().parents[1]/'lib/longitudinal_planner.py'
    s=p.read_text(encoding='utf-8');ast.parse(s)
    self.assertLess(s.index('output_a_target = self.bump_brake_entry.update('),s.index('gap_cap = self.lead_gap_reserve.update('))
    self.assertLess(s.index('output_a_target = self.bump_brake_entry.update('),s.index('stop_cap = self.normal_stop_gap.update('))
    self.assertIn('self.mpc.source == log.LongitudinalPlan.LongitudinalPlanSource.cruise',s)


if __name__=='__main__':unittest.main()
