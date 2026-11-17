from types import SimpleNamespace as NS
import unittest
import numpy as np

from openpilot.selfdrive.controls.lib.single_lane_reference import SingleLaneReference
from openpilot.selfdrive.controls.lib.model_narrow_road_guard import ModelNarrowRoadGuard


def fixture():
  x = np.linspace(0., 60., 61)
  m = NS(position=NS(x=x,y=x*0),
         laneLines=[NS(x=x,y=x*0+v) for v in (-3.8,-1.05,1.65,2.2)],
         laneLineProbs=[.8,.95,.9,.01], laneLineStds=[.3,.12,.15,.8],
         roadEdges=[NS(x=x,y=x*0-5),NS(x=x,y=x*0+1.9)],roadEdgeStds=[1.,.3],
         meta=NS(laneWidthLeft=2.8,laneWidthRight=.5,desire='none',laneChangeState='off'))
  c=NS(vEgo=8.,leftLaneLine=21,rightLaneLine=24,steeringPressed=False,
       leftBlinker=False,rightBlinker=False,rightBlindspot=False)
  return m,c


class TestSingleLaneReference(unittest.TestCase):
  def setUp(self):
    self.m,self.c=fixture();self.ref=SingleLaneReference();self.t=10.

  def tick(self,n=1,**kwargs):
    values=[]
    for _ in range(n):
      self.t+=.05
      values.append(self.ref.update(self.m,self.c,0.,self.t,self.t,enabled=True,**kwargs))
    return values

  def test_opposing_lane_is_not_same_direction_neighbor(self):
    values=self.tick(160)
    self.assertTrue(self.ref.supported)
    self.assertGreater(values[-1],.1)
    self.assertLessEqual(max(values),.25)
    self.assertLessEqual(max(np.diff(values)),.04*.05+1e-10)

  def test_no_yellow_evidence_no_initial_preference(self):
    self.c.leftLaneLine=11
    self.assertEqual(max(self.tick(160)),0.)

  def test_weak_right_line_uses_recent_width_and_edge(self):
    self.tick(50)
    self.m.laneLineProbs[2]=.1;self.m.laneLineStds[2]=.7
    self.assertGreater(self.tick(25)[-1],0.)
    self.assertEqual(self.ref.reason,'single width hold')
    self.assertEqual(self.tick(110)[-1],0.)

  def test_unknown_colour_bridge_expires(self):
    self.tick(50);self.c.leftLaneLine=2
    self.assertGreater(self.tick(10)[-1],0.)
    self.assertEqual(self.tick(40)[-1],0.)

  def test_right_neighbor_or_missing_edge_cancels(self):
    for case in ('neighbor','edge','left','nan'):
      self.setUp();self.tick(50)
      if case=='neighbor':self.m.meta.laneWidthRight=3.;self.m.laneLineProbs[3]=.9
      if case=='edge':self.m.roadEdgeStds[1]=1.
      if case=='left':self.m.laneLineProbs[1]=.1
      if case=='nan':self.m.laneLines[1].y[3]=float('nan')
      self.assertEqual(self.tick()[-1],0.,case)

  def test_close_right_edge_wins_over_preference(self):
    self.tick(60);self.m.roadEdges[1].y[:]=1.2
    self.assertEqual(self.tick()[-1],0.)

  def test_already_right_of_centre_not_pushed(self):
    for line in self.m.laneLines:line.y-=.5
    self.m.roadEdges[1].y-=.5
    self.assertEqual(max(self.tick(120)),0.)

  def test_disabled_stale_driver_and_maneuver_reset(self):
    for case in ('disabled','stale','driver','blinker','blindspot','turn','fast'):
      self.setUp();self.tick(60)
      if case=='driver':self.c.steeringPressed=True
      if case=='blinker':self.c.rightBlinker=True
      if case=='blindspot':self.c.rightBlindspot=True
      if case=='turn':self.m.meta.desire='turnRight'
      if case=='fast':self.c.vEgo=20.
      self.assertEqual(self.ref.update(self.m,self.c,0.,self.t,self.t+(1. if case=='stale' else 0.),enabled=case!='disabled'),0.,case)

  def test_model_mode_acquires_reference_with_untrusted_right_line(self):
    guard=ModelNarrowRoadGuard()
    for _ in range(40):
      self.t+=.05;guard.update(self.m,self.c,0.,0.,self.t,self.t,enabled=True)
    self.m.laneLineProbs[2]=.1;self.m.laneLineStds[2]=.7
    for _ in range(20):
      self.t+=.05;out=guard.update(self.m,self.c,0.,0.,self.t,self.t,enabled=True)
    self.assertGreater(out,0.)
    self.assertEqual(guard.reason,'single width hold')
    self.assertLessEqual(guard.accel,.15)
    self.assertEqual(guard.update(self.m,self.c,0.,-.001,self.t+.05,self.t+.05,enabled=True),-.001)

  def test_moderate_painted_right_line_maintains_confirmed_width(self):
    self.tick(40)
    self.m.laneLineProbs[2]=.28;self.m.laneLineStds[2]=.37
    self.assertGreater(self.tick(140)[-1],0.)
    self.assertTrue(self.ref.supported)
    self.m.laneLines[2].y += .3
    self.tick(110)
    self.assertFalse(self.ref.supported)

  def test_calibrated_path_does_not_stack_right_bias_or_cross_margin(self):
    self.tick(160)
    path=np.column_stack((self.m.position.x,self.m.position.y.copy()))
    self.assertGreater(self.ref.for_path(path),0.)
    path[:,1]=.6
    self.assertEqual(self.ref.for_path(path),0.)
    path[:,1]=self.ref.shift*.5
    self.assertLess(self.ref.for_path(path),self.ref.shift)

  def test_lane_to_model_transition_retains_observed_width(self):
    guard=ModelNarrowRoadGuard()
    for _ in range(40):
      self.t+=.05
      guard.single_lane.update(self.m,self.c,0.,self.t,self.t,enabled=True)
      guard.reset(reset_reference=False)
    self.m.laneLineProbs[2]=.1;self.m.laneLineStds[2]=.7
    self.t+=.05
    out=guard.update(self.m,self.c,0.,0.,self.t,self.t,enabled=True)
    self.assertGreater(out,0.)
    self.assertEqual(guard.reason,'single width hold')

  def test_brief_left_uncertainty_uses_recent_strong_reference_only(self):
    self.tick(40)
    self.m.laneLineProbs[1]=.7;self.m.laneLineStds[1]=.38
    self.assertGreater(self.tick(10)[-1],0.)
    self.assertEqual(self.tick(15)[-1],0.)
    self.ref=SingleLaneReference()
    self.assertEqual(max(self.tick(30)),0.)

  def test_duplicate_frame_does_not_increase_shift(self):
    self.tick(60);old=self.ref.shift
    for _ in range(100):self.ref.update(self.m,self.c,0.,self.t,self.t,enabled=True)
    self.assertEqual(old,self.ref.shift)


if __name__=='__main__':unittest.main()
