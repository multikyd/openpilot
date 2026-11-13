import math
import unittest

from openpilot.selfdrive.carrot.bump_approach import BumpApproach


class TestBumpApproach(unittest.TestCase):
  def test_final_target_steps_directly_at_braking_point(self):
    h = BumpApproach()
    self.assertEqual(h.update(100., 100., 30., 60., 66., 'main'), 250.)
    self.assertEqual(h.update(100.1, 82., 30., 60., 60., 'main'), 30.)

  def test_falling_speed_does_not_release_and_reapply_cap(self):
    h = BumpApproach()
    h.update(100., 82., 30., 60., 60., 'main')
    self.assertEqual(h.update(100.1, 80., 30., 55., 59., 'main'), 30.)
    self.assertEqual(h.update(100.2, 78., 30., 49., 58., 'main'), 30.)

  def test_learned_20_35_40_targets_are_preserved(self):
    for target in (20., 35., 40.):
      self.assertEqual(BumpApproach().update(100., 30., target, 60., 50., 'main'), target)

  def test_next_bump_and_long_observation_gap_reset_activation(self):
    h = BumpApproach()
    h.update(100., 10., 30., 40., 30., 'main')
    self.assertEqual(h.update(100.1, 150., 30., 40., 75., 'main'), 250.)
    h.update(100.2, 10., 30., 40., 30., 'main')
    self.assertEqual(h.update(102., 10., 30., 20., 40., 'main'), 250.)

  def test_channels_do_not_share_a_latch(self):
    h = BumpApproach()
    h.update(100., 10., 30., 60., 30., 'main')
    self.assertEqual(h.update(100., 100., 30., 60., 80., 'plus'), 250.)

  def test_invalid_data_clears_latch(self):
    h = BumpApproach()
    h.update(100., 10., 30., 60., 30., 'main')
    self.assertEqual(h.update(100.1, math.nan, 30., 60., 30., 'main'), 250.)
    self.assertNotIn('main', h.states)

  def test_target_held_at_location_until_upstream_passage_release(self):
    h = BumpApproach()
    self.assertEqual(h.update(100., 0., 30., 30., 30., 'main'), 30.)
