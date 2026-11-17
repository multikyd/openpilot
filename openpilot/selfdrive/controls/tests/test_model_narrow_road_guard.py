import copy
import math
from types import SimpleNamespace as NS
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.model_narrow_road_guard import ModelNarrowRoadGuard


def fixture():
  x = np.linspace(0., 40., 33)
  model = NS(position=NS(x=x, y=np.zeros(33)),
             laneLines=[NS(x=x, y=np.full(33, y)) for y in (-4., -1.25, 1.65, 4.)],
             laneLineProbs=[.5, .9, .9, .01], laneLineStds=[.1, .08, .08, .1],
             roadEdges=[NS(x=x, y=np.full(33, -1.4)), NS(x=x, y=np.full(33, 1.8))],
             roadEdgeStds=[.2, .2],
             meta=NS(laneWidthRight=.7, laneChangeState='off', desire='none'))
  cs = NS(vEgo=8., leftBlinker=False, rightBlinker=False, steeringPressed=False)
  return model, cs


class TestModelNarrowRoadGuard(unittest.TestCase):
  def setUp(self):
    self.model, self.cs = fixture()
    self.guard = ModelNarrowRoadGuard()
    self.now = 10.

  def tick(self, count=1, **kwargs):
    result = []
    for _ in range(count):
      self.now += .05
      args = dict(model=self.model, cs=self.cs, curvature=0., guarded=0.,
                  stamp=self.now, now=self.now, enabled=True)
      args.update(kwargs)
      result.append(self.guard.update(**args))
    return result

  def test_confirm_then_build_bounded_right_command(self):
    values = self.tick(140)
    self.assertTrue(all(v == 0. for v in values[:19]))
    self.assertGreater(values[-1], 0.)
    self.assertLessEqual(max(values)*self.cs.vEgo**2, .15)
    self.assertLessEqual(self.guard.bias.shift, .12)
    self.assertLessEqual(max(np.diff(values))*self.cs.vEgo**2, .15*.05+1e-8)

  def test_capnp_builder_and_reader_follow_the_same_path(self):
    from openpilot.cereal import log
    source, _ = fixture()
    actual = log.ModelDataV2.new_message()
    actual.position = dict(x=source.position.x.tolist(), y=source.position.y.tolist())
    for key in ('laneLines', 'roadEdges'):
      setattr(actual, key, [dict(x=line.x.tolist(), y=line.y.tolist()) for line in getattr(source, key)])
    for key in ('laneLineProbs', 'laneLineStds', 'roadEdgeStds'):
      setattr(actual, key, getattr(source, key))
    actual.meta.laneWidthRight = source.meta.laneWidthRight
    for model in (actual, actual.as_reader()):
      self.guard = ModelNarrowRoadGuard()
      self.model = model
      self.assertGreater(self.tick(140)[-1], 0.)

  def test_center_preference_not_additive_when_already_right(self):
    for line in self.model.laneLines:
      line.y -= .4
    self.assertEqual(self.tick(140), [0.]*140)

  def test_wide_lane_and_right_neighbor_are_excluded(self):
    for case in ('wide', 'neighbor'):
      with self.subTest(case=case):
        self.setUp()
        self.tick(80)
        if case == 'wide':
          self.model.laneLines[1].y[:] = -1.8
          self.model.laneLines[2].y[:] = 1.8
        else:
          self.model.meta.laneWidthRight = 3.
          self.model.laneLineProbs[3] = .8
        self.assertEqual(self.tick(), [0.])
        self.assertFalse(self.guard.active)

  def test_one_way_and_yellow_centerline_use_same_rightmost_geometry(self):
    for left_width in (.5, 3.2):
      self.setUp()
      self.model.meta.laneWidthLeft = left_width
      self.cs.leftLaneLine = 21 if left_width < 1. else 11
      self.assertGreater(self.tick(120)[-1], 0.)

  def test_existing_boundary_correction_vetoes_or_dominates_preference(self):
    self.tick(120)
    self.assertEqual(self.tick(guarded=-.0002), [-.0002])
    self.assertFalse(self.guard.active)
    values = self.tick(120, guarded=.003)
    self.assertEqual(values, [.003]*120)

  def test_new_right_boundary_has_priority_over_release_smoothing(self):
    self.tick(120)
    self.model.roadEdges[1].y[:] = 1.3
    self.assertEqual(self.tick(), [0.])
    self.assertEqual(self.guard.accel, 0.)

  def test_brief_probability_dip_cannot_increase_preference(self):
    self.tick(120)
    shift, accel = self.guard.bias.shift, self.guard.accel
    self.model.laneLineProbs[2] = .2
    values = self.tick(5)
    self.assertEqual(self.guard.bias.shift, shift)
    # Do not let the command converge further toward its former target either.
    self.assertLessEqual(max(values)*self.cs.vEgo**2, accel+1e-10)
    self.tick(180)
    self.assertAlmostEqual(self.guard.accel, 0., places=6)
    self.assertEqual(self.guard.bias.shift, 0.)

  def test_geometry_bump_does_not_build_new_preference(self):
    self.tick(90)
    old = self.guard.bias.shift
    for line in self.model.laneLines:
      line.y += .25
    self.model.roadEdges[1].y += .25
    self.tick(4)
    self.assertLessEqual(self.guard.bias.shift, old)

  def test_missing_road_edge_cannot_create_or_retain_preference(self):
    self.tick(90)
    self.model.roadEdgeStds[1] = 1.
    self.assertEqual(self.tick(), [0.])
    self.assertFalse(self.guard.active)

  def test_driver_maneuver_stale_high_speed_and_disabled_reset(self):
    for case in ('driver', 'blinker', 'blindspot', 'desire', 'stale', 'fast', 'disabled', 'curve'):
      with self.subTest(case=case):
        self.setUp()
        self.tick(90)
        args = {}
        if case == 'driver': self.cs.steeringPressed = True
        if case == 'blinker': self.cs.rightBlinker = True
        if case == 'blindspot': self.cs.rightBlindspot = True
        if case == 'desire': self.model.meta.desire = 'laneChangeRight'
        if case == 'stale': args['now'] = self.now+1.
        if case == 'fast': self.cs.vEgo = 20.
        if case == 'disabled': args['enabled'] = False
        if case == 'curve': args.update(curvature=.02, guarded=.02)
        self.assertEqual(self.tick(**args), [args.get('guarded', 0.)])
        self.assertFalse(self.guard.active)

  def test_duplicate_frames_cannot_integrate_control_frequency(self):
    self.tick(90)
    state = (self.guard.accel, self.guard.bias.shift, self.guard.bias.confirm)
    for _ in range(100):
      self.guard.update(self.model, self.cs, 0., 0., self.now, self.now, enabled=True)
    self.assertEqual(state, (self.guard.accel, self.guard.bias.shift, self.guard.bias.confirm))

  def test_nonfinite_and_incomplete_geometry_fail_closed(self):
    original = copy.deepcopy(self.model)
    for case in ('nan', 'short', 'time'):
      self.model = copy.deepcopy(original)
      if case == 'nan': self.model.laneLines[2].y[4] = math.nan
      if case == 'short': self.model.position.x = self.model.position.x[:4]
      self.assertEqual(self.tick(stamp=math.nan) if case == 'time' else self.tick(), [0.])


if __name__ == '__main__':
  unittest.main()
