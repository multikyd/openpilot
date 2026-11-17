import unittest
from unittest.mock import patch

from openpilot.selfdrive.kisapilot.bump_learning import BumpMemory, offset


class TestBumpOverrideReview(unittest.TestCase):
  def setUp(self):
    self.memory = BumpMemory()
    self.record = self.memory._record(37., 127., 0., True)

  def passage(self, stamp, **kwargs):
    args = dict(record=self.record, peak=1., above=0, samples=100, speed=39.,
                manual=True, eligible=False, now=stamp,
                override_seconds=.8, override_eligible=True)
    args.update(kwargs)
    with patch('openpilot.selfdrive.kisapilot.bump_learning.time.time', return_value=stamp):
      self.memory.finish_pass(**args)

  def test_three_independent_passes_apply_to_this_site(self):
    self.passage(1000.)
    self.passage(1400.)
    self.assertNotIn('override_adjustment', self.record)
    self.passage(1800.)
    self.assertEqual(self.record['override_adjustment']['speed'], 35)
    self.assertEqual(self.record['target'], 35)
    self.assertIn('적용', self.memory.notice)
    self.assertEqual(self.memory.override_events[-1]['passes'], 3)

  def test_repeated_40_passes_propose_40(self):
    for stamp in (1000., 1400., 1800.):
      self.passage(stamp, speed=40.)
    self.assertEqual(self.record['override_adjustment']['speed'], 40)
    self.assertEqual(self.record['target'], 40)

  def test_same_encounter_and_brief_pedal_do_not_count(self):
    self.passage(1000.)
    self.passage(1010.)
    self.passage(1020., override_seconds=.1)
    self.assertEqual(len(self.record['override_passes']), 1)

  def test_excluded_context_and_missing_imu_do_not_count(self):
    self.passage(1000., override_eligible=False)
    self.passage(1400., samples=5)
    self.assertNotIn('override_passes', self.record)

  def test_rough_impact_clears_candidate_and_preserves_slowdown(self):
    for stamp in (1000., 1400., 1800.):
      self.passage(stamp)
    self.passage(2200., peak=4., above=3)
    self.assertNotIn('override_passes', self.record)
    self.assertLess(self.record['target'], 35)

  def test_inconsistent_speeds_do_not_propose(self):
    for stamp, speed in ((1000., 35.), (1400., 40.), (1800., 45.)):
      self.passage(stamp, speed=speed)
    self.assertNotIn('override_adjustment', self.record)

  def test_manual_cap_and_direction_remain_separate(self):
    other = self.memory._record(37., 127., 180., True)
    self.record['manual_cap'] = 30
    for stamp in (1000., 1400., 1800.):
      self.passage(stamp)
    self.assertNotIn('override_adjustment', self.record)
    self.assertNotIn('override_passes', other)

  def test_history_survives_restart_but_expires(self):
    self.passage(1000.)
    self.memory = BumpMemory(self.memory.export())
    self.record = self.memory.records[0]
    self.passage(1400.)
    self.assertEqual(len(self.record['override_passes']), 2)
    self.passage(1000.+91*86400)
    self.assertEqual(len(self.record['override_passes']), 1)

  def test_real_tick_tracks_pedal_until_crossing_once(self, brake=False):
    m = self.memory
    for index in range(115):
      now = 100.+index*.05
      distance = 30.-(now-100.)*40./3.6
      lat, lon = offset(37., 127., 0., -distance)
      m.gps(now, lat, lon, 0., 1.)
      m.sensor_last = now
      m.samples.append((now, 1.))
      with patch('openpilot.selfdrive.kisapilot.bump_learning.time.time', return_value=1000.):
        m.tick(now, 40., distance if distance > 0 else None, True, False,
               gas_pressed=True, override_eligible=True, brake_pressed=brake and index == 30)
    self.assertEqual(len(self.record.get('override_passes', [])), 0 if brake else 1)
    self.assertEqual(self.record['target'], 30)
    self.assertEqual(len(m.override_events), 1)

  def test_gas_then_brake_excludes_whole_pass(self):
    self.test_real_tick_tracks_pedal_until_crossing_once(brake=True)
    self.assertEqual(self.memory.override_events[-1]['reason'], 'context_excluded')

  def test_configured_raise_limit_is_respected(self):
    self.memory.raise_limit = 0
    for stamp in (1000., 1400., 1800.):
      self.passage(stamp, speed=40.)
    self.assertEqual(self.record['target'], 30)
    self.assertNotIn('override_adjustment', self.record)


if __name__ == '__main__':
  unittest.main()
