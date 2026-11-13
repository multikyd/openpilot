import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

spec = importlib.util.spec_from_file_location('traffic_test_module', Path(__file__).parents[1]/'lib/overtake_traffic.py')
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m
spec.loader.exec_module(m)


def vehicle(track, distance, speed, path=0.):
  return NS(present=True, radar=True, radarTrackId=track, dRel=distance, vLead=speed/3.6, dPath=path)


class TestTrafficAssessment(unittest.TestCase):
  def setUp(self):
    self.x = dict(fresh=True, limit_valid=True, road_limit=110./3.6, set_speed=120./3.6,
                  lead=vehicle(1, 40., 100.), center=[], left=[], right=[])

  def assess(self, **kw):
    return m.assess_traffic(**dict(self.x, **kw))

  def test_limit_speed_respected_even_with_large_cruise_offset_and_fast_sides(self):
    result = self.assess(lead=vehicle(1, 40., 110.), set_speed=130./3.6,
                         left=[vehicle(2, 30., 125., 3.5)], right=[vehicle(3, 25., 125., -3.5)])
    self.assertEqual((result.eligible, result.reason), (False, 'respect_road_limit'))

  def test_no_detection_is_unknown_not_empty_road(self):
    result = self.assess()
    self.assertFalse(result.eligible)
    self.assertEqual(result.reason, 'insufficient_traffic_evidence')
    self.assertEqual(result.ahead_gap, 0.)

  def test_both_faster_sides_support_slow_lead_without_claiming_an_ahead_gap(self):
    result = self.assess(left=[vehicle(2, 30., 115., 3.5)], right=[vehicle(3, 20., 112., -3.5)])
    self.assertEqual((result.eligible, result.reason), (True, 'both_adjacent_lanes_faster'))
    self.assertEqual(result.ahead_gap, 0.)

  def test_one_fast_car_on_one_side_is_insufficient(self):
    self.assertFalse(self.assess(left=[vehicle(2, 30., 130., 3.5)]).eligible)

  def test_queue_vetoes_faster_sides(self):
    result = self.assess(center=[vehicle(2, 75., 98.)],
                         left=[vehicle(3, 30., 115., 3.5)], right=[vehicle(4, 20., 115., -3.5)])
    self.assertEqual((result.eligible, result.reason), (False, 'following_traffic'))
    self.assertEqual(result.ahead_gap, 35.)

  def test_observed_large_gap_supports_candidate(self):
    result = self.assess(center=[vehicle(2, 140., 110.)])
    self.assertEqual((result.eligible, result.reason), (True, 'observed_large_ahead_gap'))
    self.assertEqual(result.ahead_gap, 100.)

  def test_nearest_same_lane_vehicle_vetoes_distant_car(self):
    result = self.assess(center=[vehicle(2, 140., 115.), vehicle(3, 65., 100.)])
    self.assertEqual(result.reason, 'following_traffic')

  def test_adjacent_cut_in_and_original_id_are_not_a_second_lead(self):
    result = self.assess(center=[vehicle(1, 145., 110.), vehicle(2, 140., 110., 2.)])
    self.assertFalse(result.eligible)

  def test_stale_unknown_or_nonfinite_limit_never_allows_entry(self):
    for kw in (dict(fresh=False), dict(limit_valid=False), dict(road_limit=0.), dict(road_limit=float('nan'))):
      self.assertFalse(self.assess(center=[vehicle(2, 140., 110.)], **kw).eligible)

  def test_side_flow_rejects_invalid_opposing_and_out_of_lane_objects(self):
    for item in (vehicle(2, 30., -100., 3.5), vehicle(2, 30., 120., 7.), vehicle(2, 300., 120., 3.5)):
      result = self.assess(left=[item], right=[vehicle(3, 20., 115., -3.5)])
      self.assertFalse(result.left_faster)
      self.assertFalse(result.eligible)


class TestAdviceConfirmation(unittest.TestCase):
  def test_same_lead_requires_three_seconds(self):
    h = m.OvertakeAdviceConfirmation()
    self.assertFalse(h.update(10., True, 40))
    for i in range(1, 60):
      self.assertFalse(h.update(10.+i*.05, True, 40))
    self.assertTrue(h.update(13., True, 40))
    self.assertFalse(h.update(13.05, False, 40))

  def test_target_change_gap_and_duplicate_time_do_not_create_confirmation(self):
    for mode in ('target', 'gap', 'duplicate'):
      h = m.OvertakeAdviceConfirmation()
      for i in range(60):
        h.update(10.+i*.05, True, 40)
      if mode == 'target':
        self.assertFalse(h.update(13., True, 41))
      elif mode == 'gap':
        self.assertFalse(h.update(14., True, 40))
      else:
        for _ in range(100):
          self.assertFalse(h.update(12.95, True, 40))


if __name__ == '__main__':
  unittest.main()
