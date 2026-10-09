import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from openpilot.selfdrive.controls.lib import lane_planner_2 as lane_module
from openpilot.selfdrive.controls.lib.lateral_planner import get_lane_center_curvature


class TestLaneCenterOffset(unittest.TestCase):
  def setUp(self):
    self.values = {
      'LaneCenterOffset': -0.03, 'LeftEdgeOffset': 0.0, 'RightEdgeOffset': 0.0,
      'CameraOffsetAdj': 0.04, 'AdjustLaneOffset': 0.0, 'LatMpcInputOffset': 0,
    }
    params = SimpleNamespace(get=lambda key, **kwargs: self.values[key])
    with patch.object(lane_module, 'Params', lambda: params):
      self.lp = lane_module.LanePlanner()
    self.lp.d_prob = 1.0
    self.lp.lane_width = 3.5
    self.cs = SimpleNamespace(leftBlinker=False, rightBlinker=False)

  def settle(self, active=True, frames=250):
    return [self.lp._get_lane_center_offset(self.cs, active) for _ in range(frames)]

  def test_straight_left_preference(self):
    self.assertAlmostEqual(self.settle()[-1], -0.03, places=7)

  def test_curve_fades_symmetrically(self):
    for curvature, expected in [(0.002, -0.03), (0.0035, -0.015), (0.005, 0.0), (-0.0035, -0.015), (-0.01, 0.0)]:
      self.lp.lane_center_curvature = curvature
      self.assertAlmostEqual(self.settle()[-1], expected, places=7)

  def test_straight_curve_transition_is_smooth(self):
    straight = self.settle()
    self.lp.lane_center_curvature = 0.01
    curve = self.settle()
    self.assertLess(max(abs(x) for x in np.diff(straight + curve)), 0.002)
    self.assertTrue(np.all(np.diff(curve) >= -1e-12))
    self.assertAlmostEqual(curve[-1], 0.0, places=7)

  def test_laneless_has_no_added_offset_even_after_straight(self):
    self.settle()
    self.assertEqual(self.lp._get_lane_center_offset(self.cs, False), 0.0)

  def test_blinker_disables_added_offset(self):
    self.settle()
    for side in ['leftBlinker', 'rightBlinker']:
      setattr(self.cs, side, True)
      self.assertEqual(self.lp._get_lane_center_offset(self.cs, True), 0.0)
      setattr(self.cs, side, False)

  def test_confidence_and_width_limit_preference(self):
    self.settle()
    self.lp.d_prob = 0.3
    self.assertEqual(self.lp._get_lane_center_offset(self.cs, True), 0.0)
    self.lp.d_prob = 0.55
    self.assertAlmostEqual(self.lp._get_lane_center_offset(self.cs, True), -0.015, places=7)
    self.lp.d_prob = 1.0
    self.lp.lane_width = 2.8
    self.assertEqual(self.lp._get_lane_center_offset(self.cs, True), 0.0)
    self.lp.lane_width = 3.0
    self.assertAlmostEqual(self.lp._get_lane_center_offset(self.cs, True), -0.015, places=7)

  def test_parameter_disable_and_bound(self):
    self.lp.lane_center_offset = 0.0
    self.assertEqual(self.settle()[-1], 0.0)
    self.lp.lane_center_offset = -1.0
    self.assertAlmostEqual(self.settle()[-1], -0.05, places=7)

  def test_invalid_input_does_not_poison_filter(self):
    self.settle()
    for value, curvature in [(None, 0.0), (math.nan, 0.0), (-0.03, math.inf), (-0.03, math.nan)]:
      self.lp.lane_center_offset = value
      self.lp.lane_center_curvature = curvature
      self.assertEqual(self.lp._get_lane_center_offset(self.cs, True), 0.0)
      self.assertTrue(math.isfinite(self.lp.lane_center_offset_filtered.x))

  def test_preview_reduces_preference_before_curve(self):
    t = np.array([0.0, 0.75, 1.5, 3.0])
    speed = np.ones(4) * 20.0
    for sign in [-1.0, 1.0]:
      yaw_rate = np.array([0.0, 0.02, 0.1, 0.2]) * sign
      self.assertAlmostEqual(get_lane_center_curvature(0.0, t, yaw_rate, speed), 0.005)
    self.assertAlmostEqual(get_lane_center_curvature(-0.01, t, np.zeros(4), speed), 0.01)

  def test_preview_invalid_curvature_disables_preference(self):
    t = np.array([0.0, 0.75, 1.5])
    self.lp.lane_center_curvature = get_lane_center_curvature(math.nan, t, np.zeros(3), np.ones(3) * 20)
    self.assertEqual(self.settle()[-1], 0.0)

  def lane_path(self, frames=250):
    n = lane_module.TRAJECTORY_SIZE
    self.lp.ll_t = np.linspace(0.0, 3.0, n)
    self.lp.ll_x = np.linspace(0.0, 60.0, n)
    self.lp.lll_y = np.ones(n) * (-1.75 + self.lp.total_camera_offset)
    self.lp.rll_y = np.ones(n) * (1.75 + self.lp.total_camera_offset)
    self.lp.lll_prob = self.lp.rll_prob = 1.0
    self.lp.lll_std = self.lp.rll_std = 0.05
    path = None
    active = False
    for _ in range(frames):
      xyz = np.column_stack([self.lp.ll_x, np.zeros(n), np.zeros(n)])
      path, active = self.lp.get_d_path(self.cs, 20.0, self.lp.ll_t, xyz, 200.0)
    return path, active

  def test_full_lane_path_applies_preference_once(self):
    self.lp.lanefull_mode = True
    path, active = self.lane_path()
    self.assertTrue(active)
    np.testing.assert_allclose(path[:, 1], 0.04 - 0.03, atol=1e-7)

  def test_laneless_preserves_old_calibration_total(self):
    self.lp.lanefull_mode = False
    self.lp.path_offset = -0.01
    self.lp.path_offset2 = 0.02
    path, active = self.lane_path()
    self.assertFalse(active)
    np.testing.assert_allclose(path[:, 1], 2 * (-0.01 + 0.02), atol=1e-12)

  def test_disabled_preference_preserves_lane_calibration_total(self):
    self.lp.lanefull_mode = True
    self.lp.lane_center_offset = 0.0
    self.lp.total_camera_offset = 0.05
    self.lp.path_offset = -0.01
    self.lp.path_offset2 = 0.02
    path, active = self.lane_path()
    self.assertTrue(active)
    np.testing.assert_allclose(path[:, 1], 0.05 + 2 * (-0.01 + 0.02), atol=1e-12)


if __name__ == '__main__':
  unittest.main()
