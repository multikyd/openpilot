"""Exercise the production radar association functions without native messaging."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np


def load_matching():
  source = Path(__file__).resolve().parents[1] / 'radard.py'
  tree = ast.parse(source.read_text(encoding='utf-8'))
  names = {'laplacian_pdf', 'is_stationary_side_track', 'match_vision_to_track'}
  functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
  radar = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'RadarD')
  functions += [node for node in radar.body if isinstance(node, ast.FunctionDef) and node.name == 'get_sticky_track']
  prefix = ast.parse('from __future__ import annotations').body
  module = ast.Module(body=prefix+functions, type_ignores=[])
  env = dict(math=math, np=np, RADAR_TO_CAMERA=1.52, DT_MDL=.05, STICKY_SELECTED_COUNT_MAX=40)
  exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), env)
  return env


class TestRadarSideMatch(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.functions = load_matching()

  def track(self, side=-1., path=2.8, speed=.3, inward=0., future=None):
    return NS(dRel=7., yRel=side*path, dPath=side*path,
              dPath_future=side*(path if future is None else future), yvLead=-side*inward,
              vLead=speed, in_lane_prob=max(0., 1.-path/1.8), selected_count=10,
              is_stopped_car_count=25, cnt=30, score=0., measured=True,
              sticky_dPath=side*path, sticky_dpath_limit=lambda: .8,
              vlead_for_matching=lambda: speed)

  def test_roadside_stopped_returns_cannot_become_lead_with_uncertain_vision(self):
    vision = NS(x=[8.], y=[.9], v=[4.], xStd=[10.], yStd=[1.], vStd=[2.])
    for side in (-1., 1.):
      track = self.track(side=side)
      self.assertIsNone(self.functions['match_vision_to_track'](8., vision, .98, {50: track}))
      self.assertEqual(track.selected_count, 0)

  def test_real_in_path_stop_remains_eligible(self):
    track = self.track(path=.2, speed=0.)
    vision = NS(x=[8.52], y=[.2], v=[0.], xStd=[1.], yStd=[.3], vStd=[1.])
    self.assertIs(self.functions['match_vision_to_track'](8., vision, .98, {50: track}), track)

  def test_slow_inward_vehicle_and_moving_adjacent_track_not_blanket_filtered(self):
    self.assertFalse(self.functions['is_stationary_side_track'](self.track(inward=.5, future=1.2)))
    self.assertFalse(self.functions['is_stationary_side_track'](self.track(speed=14.)))
    self.assertTrue(self.functions['is_stationary_side_track'](self.track(inward=-.5, future=3.)))

  def test_sticky_fallback_cannot_restore_rejected_side_return(self):
    track = self.track()
    self.assertIsNone(self.functions['get_sticky_track'](None, {50: track}))
    self.assertEqual(track.selected_count, 0)


if __name__ == '__main__':
  unittest.main()
