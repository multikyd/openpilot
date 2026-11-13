import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.congestion_follow import CongestionFollow


class TestCongestionFollow(unittest.TestCase):
  def setUp(self):
    self.helper = CongestionFollow()
    self.lead = SimpleNamespace(present=True, radar=True, radarTrackId=4, modelProb=.99, dRel=12., dPath=0., vLead=0.)

  def tick(self, speed, seconds, enabled=True, normal=1.6, highway=False):
    self.lead.vLead = speed
    result = None
    for _ in range(round(seconds/.05)):
      result = self.helper.update(self.lead, speed, normal, .9, enabled, .05, highway=highway)
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

  def test_highway_creeping_waves_use_gentler_acceleration(self):
    self.tick(1.6, 3., highway=True)
    self.tick(3.5, 4., highway=True)
    self.tick(1.6, 4., highway=True)
    follow, accel = self.tick(3., 2., highway=True)
    self.assertAlmostEqual(follow, 1.15)
    self.assertAlmostEqual(accel, .5+.25*3./8.)

  def test_highway_single_wave_does_not_limit_departure(self):
    self.tick(1.6, 8., highway=True)
    self.assertEqual(self.tick(3.5, 4., highway=True), (.9, 1.6))

  def test_highway_creep_threshold_noise_is_not_a_second_wave(self):
    self.tick(1.6, 3., highway=True)
    for _ in range(5):
      self.tick(2.5, 3., highway=True)
      self.tick(1.6, 3., highway=True)
    self.assertEqual(len(self.helper.stops), 1)
    self.assertEqual(self.helper.strength, 0.)

  def test_highway_keeps_stronger_deceleration_and_driver_override(self):
    self.tick(1.6, 3., highway=True)
    self.tick(3.5, 4., highway=True)
    self.tick(1.6, 4., highway=True)
    self.assertEqual(self.tick(3., 1., normal=-1.5, highway=True)[1], -1.5)
    self.assertEqual(self.tick(3., .05, enabled=False, highway=True), (.9, 1.6))

  def test_non_highway_creeping_does_not_arm_new_threshold(self):
    self.tick(1.6, 3.)
    self.tick(3.5, 4.)
    self.tick(1.6, 4.)
    self.assertEqual(self.tick(3., 2.), (.9, 1.6))

  def test_continuous_stop_is_one_event(self):
    self.tick(0., 80.)
    self.assertEqual(len(self.helper.stops), 1)
    self.assertEqual(self.helper.strength, 0.)


class TestHighwayQueueGate(unittest.TestCase):
  def test_only_current_valid_highway_navigation_enables_highway_queue(self):
    import ast
    from pathlib import Path
    tree = ast.parse((Path(__file__).parents[1]/'lib/longitudinal_planner.py').read_text(encoding='utf8'))
    expression = next(k.value for n in ast.walk(tree) if isinstance(n, ast.Call)
                      for k in n.keywords if k.arg == 'highway_queue')
    code = compile(ast.Expression(body=expression), 'actual_highway_queue_gate', 'eval')

    class SM(dict):
      checks = True
      logMonoTime = {'carrotMan': int(10e9)}
      def all_checks(self, services):
        return self.checks

    sm = SM(carrotMan=SimpleNamespace(naviHighway=True))
    self.assertTrue(eval(code, dict(sm=sm, now=10.5)))
    for now in (9.9, 11.01):
      self.assertFalse(eval(code, dict(sm=sm, now=now)))
    sm.checks = False
    self.assertFalse(eval(code, dict(sm=sm, now=10.5)))
    sm.checks = True
    sm['carrotMan'].naviHighway = False
    self.assertFalse(eval(code, dict(sm=sm, now=10.5)))


if __name__ == '__main__':
  unittest.main()
