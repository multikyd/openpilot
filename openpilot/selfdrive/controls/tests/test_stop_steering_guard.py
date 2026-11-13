import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location('stop_steering_guard',
  Path(__file__).resolve().parents[1]/'lib'/'stop_steering_guard.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fixture(curvature=0.):
  x = np.linspace(0., 50., 33)
  model = NS(laneLines=[NS(x=x,y=.5*curvature*x*x+v) for v in (-5.25,-1.75,1.75,5.25)],
             laneLineProbs=[.9]*4,laneLineStds=[.1]*4,
             meta=NS(desire='none',laneChangeState='off'))
  cs = NS(vEgo=3.,aEgo=0.,gasPressed=False,steeringPressed=False,leftBlinker=False,rightBlinker=False)
  return model, cs


class TestStopSteeringGuard(unittest.TestCase):
  def setUp(self):
    self.guard=module.StopSteeringGuard()
    self.model,self.cs=fixture()

  def tick(self, curvature, t, *, stopping=True, enabled=True, measured=0.):
    return self.guard.update(self.model,self.cs,curvature,measured,t,t+.02,
                             stopping=stopping,enabled=enabled)

  def test_test13_straight_stop_rejects_large_new_left_or_right_action(self):
    for side in (-1,1):
      self.setUp();self.tick(side*.0003,10.)
      self.cs.vEgo=.4
      for i in range(1,101):
        value=self.tick(side*.04,10.+i*.01)
        self.assertLessEqual(abs(value),.00181)
        self.assertEqual(np.sign(value),side)

  def test_existing_left_and_right_bends_are_retained(self):
    for side in (-1,1):
      self.setUp();self.model,self.cs=fixture(side*.015)
      self.tick(side*.015,10.)
      self.cs.vEgo=.4
      for i in range(1,51):
        t=10.+i*.01
        self.assertAlmostEqual(self.tick(side*.015,t),side*.015)
      self.assertLessEqual(abs(self.tick(side*.06,10.51)),.016501)

  def test_real_reader_style_lists_require_integer_indices(self):
    class IntegerIndexList(list):
      def __getitem__(self, index):
        if not isinstance(index, int):
          raise TypeError('an integer is required')
        return super().__getitem__(index)
    self.model,self.cs=fixture(.015)
    expected=self.guard.lane_curvature(self.model)
    for field in ('laneLines','laneLineProbs','laneLineStds'):
      setattr(self.model,field,IntegerIndexList(getattr(self.model,field)))
    self.assertIsNotNone(expected)
    self.assertAlmostEqual(self.guard.lane_curvature(self.model),expected)

  def test_lane_geometry_can_support_tightening_bend_without_step(self):
    self.tick(.005,10.);self.cs.vEgo=1.
    self.tick(.005,10.01)
    self.model,_=fixture(.015)
    prev=.0065
    for i in range(2,402):
      value=self.tick(.015,10.+i*.01)
      self.assertLessEqual(value-prev,.003*.01+1e-9)
      prev=value
    self.assertAlmostEqual(value,.015)

  def test_unreliable_geometry_cannot_open_envelope(self):
    self.tick(0.,10.);self.cs.vEgo=.5
    self.model,_=fixture(-.04);self.model.laneLineProbs[1]=.1
    for i in range(1,50):
      self.assertAlmostEqual(self.tick(-.04,10.+i*.01),-.0015)

  def test_no_stop_request_leaves_creeping_unchanged(self):
    self.cs.vEgo=1.
    self.assertEqual(self.tick(.03,10.,stopping=False),.03)

  def test_stock_scc_sustained_deceleration_arms_without_planner_stop_flag(self):
    self.cs.aEgo=-.6
    for i in range(50):
      self.tick(.001,10.+i*.01,stopping=False)
    self.cs.vEgo=1.
    self.assertLess(self.tick(.04,10.5,stopping=False),.003)

  def test_brief_slowdown_or_accelerator_does_not_arm(self):
    for gas in (False,True):
      self.setUp();self.cs.vEgo=1.;self.cs.aEgo=-.6;self.cs.gasPressed=gas
      for i in range(20 if not gas else 100):
        self.assertEqual(self.tick(.04,10.+i*.01,stopping=False),.04)

  def test_missing_approach_history_keeps_physical_bend(self):
    self.cs.vEgo=.5
    self.assertAlmostEqual(self.tick(.06,10.,measured=.02),.0215)

  def test_driver_maneuver_disengage_stale_and_reversed_clock_reset(self):
    for case in ('driver','left','right','desire','lane_change','disabled','stale','backward'):
      self.setUp();self.tick(0.,10.);self.cs.vEgo=.5;self.tick(-.04,10.01)
      if case=='driver':self.cs.steeringPressed=True
      if case=='left':self.cs.leftBlinker=True
      if case=='right':self.cs.rightBlinker=True
      if case=='desire':self.model.meta.desire='turnLeft'
      if case=='lane_change':self.model.meta.laneChangeState='laneChangeStarting'
      t=9. if case=='backward' else 10.02
      if case=='stale':
        value=self.guard.update(self.model,self.cs,-.04,0.,t,t+.3,stopping=True,enabled=True)
      else:value=self.tick(-.04,t,stopping=False,enabled=case!='disabled')
      self.assertEqual(value,-.04,case)
      self.assertFalse(self.guard.active,case)

  def test_stop_flag_flicker_does_not_restore_large_action(self):
    self.tick(0.,10.);self.cs.vEgo=.5
    self.tick(-.04,10.01)
    self.assertAlmostEqual(self.tick(-.04,10.02,stopping=False),-.0015)
    self.cs.vEgo=3.
    self.assertEqual(self.tick(-.02,10.03,stopping=False),-.02)


if __name__=='__main__':
  unittest.main()
