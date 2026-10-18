import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

from openpilot.selfdrive.controls.lib import lane_planner_2 as lane_module


class TestLaneCameraProfile(unittest.TestCase):
  def setUp(self):
    self.values = dict(LaneCenterOffset=-.05, CameraOffsetAdj=.04, LeftEdgeOffset=0., RightEdgeOffset=0., AdjustLaneOffset=0.)
    with patch.object(lane_module, 'Params', lambda: NS(get=lambda k, **kw: self.values[k])):
      self.p = lane_module.LanePlanner()
    self.cs = NS(vEgo=20., leftBlinker=False, rightBlinker=False, steeringPressed=False, leftLaneLine=11)
    self.p.l_prob = self.p.r_prob = self.p.d_prob = 1.
    self.p.lane_width = 3.5
    self.p.lane_position_ready = True

  def geometry(self, kind):
    self.p.lane_width_left, self.p.lane_width_right = {
      'left': (.6, 3.2), 'right': (3.2, .6), 'middle': (3.2, 3.2),
      'single': (3.2, .6), 'bounded_single': (.6, .6)}[kind]
    self.p.lane_position_geometry = {
      'left': (.01,.8,.3,1.,-2.2,7.,-1.75,1.75),
      'right': (.8,.01,1.,.3,-7.,2.2,-1.75,1.75),
      'middle': (.8,.8,1.,1.,-7.,7.,-1.75,1.75),
      'single': (.8,.01,1.,.3,-7.,2.2,-1.75,1.75),
      'bounded_single': (.01,.01,.3,.3,-2.2,2.2,-1.75,1.75)}[kind]
    self.cs.leftLaneLine = 21 if kind == 'single' else 11

  def settle(self, count=500, active=True):
    return [self.p._get_lane_position_offset(self.cs, active) for _ in range(count)]

  def test_lane_counts_map_to_position_profiles(self):
    # One direction: 1 lane, 2 lanes, 3 lanes, 4 lanes. Middle slots share a profile.
    for positions in [('single',), ('left','right'), ('left','middle','right'), ('left','middle','middle','right')]:
      for kind in positions:
        with self.subTest(positions=positions,kind=kind):
          self.setUp()
          self.geometry(kind)
          self.settle()
          expected = {'left':0.,'middle':.01,'right':.04,'single':.03}[kind]
          self.assertEqual(self.p.lane_position_class, kind)
          self.assertAlmostEqual(self.p.effective_camera_offset, expected)

  def test_oncoming_lane_is_not_counted_as_same_direction_neighbor(self):
    self.geometry('single')
    self.settle()
    self.assertEqual(self.p.lane_position_class, 'single')
    self.assertAlmostEqual(self.p.effective_camera_offset, .03)

  def test_yellow_left_with_same_direction_lane_on_right_is_leftmost(self):
    self.geometry('middle')
    self.cs.leftLaneLine = 21
    self.settle()
    self.assertEqual(self.p.lane_position_class, 'left_open')
    self.assertAlmostEqual(self.p.effective_camera_offset, .01)

  def test_yellow_with_reliable_left_edge_keeps_bounded_profile(self):
    self.geometry('left')
    self.cs.leftLaneLine = 21
    self.settle()
    self.assertEqual(self.p.lane_position_class, 'left')
    self.assertAlmostEqual(self.p.effective_camera_offset, .00)

  def test_uncertain_near_left_edge_uses_open_profile(self):
    self.geometry('left')
    self.cs.leftLaneLine = 21
    geometry = list(self.p.lane_position_geometry)
    geometry[2] = .8
    self.p.lane_position_geometry = tuple(geometry)
    self.settle()
    self.assertEqual(self.p.lane_position_class, 'left_open')
    self.assertAlmostEqual(self.p.effective_camera_offset, .01)

  def test_missing_left_edge_does_not_immediately_change_profile(self):
    self.geometry('left')
    self.cs.leftLaneLine = 21
    self.settle()
    self.geometry('middle')
    self.cs.leftLaneLine = 21
    self.settle(20)
    self.assertEqual(self.p.lane_position_class, 'left')
    self.assertAlmostEqual(self.p.effective_camera_offset, .00)
    values = self.settle()
    self.assertEqual(self.p.lane_position_class, 'left_open')
    self.assertAlmostEqual(self.p.effective_camera_offset, .01)
    self.assertLessEqual(max(abs(np.diff([-.12]+values))), .015*.05+1e-10)

  def test_returning_left_edge_requires_confirmation(self):
    self.geometry('middle')
    self.cs.leftLaneLine = 21
    self.settle()
    self.geometry('left')
    self.cs.leftLaneLine = 21
    self.settle(20)
    self.assertEqual(self.p.lane_position_class, 'left_open')
    self.assertAlmostEqual(self.p.effective_camera_offset, .01)
    values = self.settle()
    self.assertEqual(self.p.lane_position_class, 'left')
    self.assertAlmostEqual(self.p.effective_camera_offset, .00)
    self.assertLessEqual(max(abs(np.diff([-.09]+values))), .015*.05+1e-10)

  def test_open_profile_requires_a_lane_to_the_right(self):
    self.geometry('middle')
    self.cs.leftLaneLine = 21
    self.p.lane_width_right = .5
    self.settle()
    self.assertEqual(self.p.lane_position_class, 'unknown')

  def test_brief_alternating_edge_detection_does_not_flicker(self):
    self.geometry('left')
    self.cs.leftLaneLine = 21
    self.settle()
    for kind in ['middle','left']*10:
      self.geometry(kind)
      self.cs.leftLaneLine = 21
      self.settle(10)
      self.assertEqual(self.p.lane_position_class, 'left')
      self.assertAlmostEqual(self.p.effective_camera_offset, .00)

  def test_unknown_colour_does_not_assert_rightmost(self):
    self.geometry('right')
    self.cs.leftLaneLine = 2  # Unknown colour, undecided type.
    self.settle()
    self.assertEqual(self.p.lane_position_class, 'unknown')
    self.assertAlmostEqual(self.p.effective_camera_offset, .01)

  def test_no_near_right_edge_does_not_assert_single(self):
    self.geometry('single')
    g = list(self.p.lane_position_geometry)
    g[3] = 1.
    self.p.lane_position_geometry = tuple(g)
    self.settle()
    self.assertEqual(self.p.lane_position_class, 'unknown')

  def test_physical_single_lane_with_two_edges(self):
    self.geometry('bounded_single')
    self.settle()
    self.assertEqual(self.p.lane_position_class, 'single')
    self.assertAlmostEqual(self.p.effective_camera_offset,.03)

  def test_confirmation_and_unknown_hold(self):
    self.geometry('left')
    self.settle(28)
    self.assertEqual(self.p.lane_position_class,'unknown')
    self.settle(300)
    previous = self.p.lane_position_offset
    self.p.lane_width_left = self.p.lane_width_right = 0.
    self.settle(20)
    self.assertEqual(self.p.lane_position_class,'left')
    self.assertAlmostEqual(self.p.lane_position_offset,previous)
    self.settle(20)
    self.assertEqual(self.p.lane_position_class,'unknown')

  def test_path_shift_has_bounded_slew(self):
    self.geometry('left')
    values = [0.]+self.settle()
    self.geometry('right')
    values += self.settle()
    self.assertLessEqual(max(abs(np.diff(values))), .015*.05+1e-10)

  def test_model_and_manual_transitions_release_gradually(self):
    self.geometry('left')
    self.settle()
    previous = self.p.lane_position_offset
    value = self.settle(1,active=False)[0]
    self.assertLessEqual(abs(value-previous),.015*.05+1e-10)
    self.assertEqual(self.settle(active=False)[-1],0.)

  def test_legacy_edge_adjustment_does_not_stack(self):
    self.geometry('right')
    self.p.total_camera_offset = .09
    self.settle()
    self.assertAlmostEqual(self.p.effective_camera_offset,.04)

  def test_invalid_geometry_releases_without_nan(self):
    self.geometry('left')
    self.settle()
    self.p.lane_position_geometry = (float('nan'),)*8
    values = self.settle()
    self.assertTrue(np.isfinite(values).all())
    self.assertEqual(values[-1],0.)

  def test_invalid_messaging_does_not_start_profile(self):
    self.geometry('left')
    self.p.lane_position_ready = False
    self.assertEqual(self.settle()[-1],0.)

  def test_profile_does_not_rewrite_global_camera_parameter(self):
    self.geometry('left')
    self.settle()
    self.assertEqual(self.values['CameraOffsetAdj'],.04)
    self.assertAlmostEqual(self.p.effective_camera_offset,0.)


if __name__ == '__main__':
  unittest.main()
