"""Recorded test7 failure modes: target relative to corridor and bump provenance."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT=Path(__file__).parents[1]
def module(name,path):
  spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
narrow=module('test7_narrow',ROOT/'lib/narrow_road_bias.py')
bump=module('test7_bump',ROOT.parent/'carrot/bump_learning.py')

def model():
  x=np.linspace(0,40,33)
  return NS(laneLines=[NS(x=x,y=np.full(33,y)) for y in (-4.,-1.45,1.45,4.)],
            laneLineProbs=[.7,.9,.9,.01],laneLineStds=[.1,.1,.1,.5],
            roadEdges=[],roadEdgeStds=[],meta=NS(laneWidthRight=.5,desire=0,laneChangeState=0))

class TestFinalCorridorTarget(unittest.TestCase):
  def setUp(self):
    self.m=model();self.cs=NS(vEgo=8.,leftBlinker=False,rightBlinker=False,steeringPressed=False)
    self.path=np.column_stack((np.linspace(0,40,33),np.full(33,-.10),np.zeros(33)))
  def settle(self,h,path=None):
    for _ in range(300):value=h.update(self.m,self.cs,self.path if path is None else path,0.,.09,enabled=True)
    return value
  def test_left_preference_cannot_cancel_right_correction_completely(self):
    shift=self.settle(narrow.NarrowRoadBias())
    self.assertAlmostEqual(shift,.12)
    self.assertGreater(self.path[0,1]+shift,0.)
  def test_path_already_at_desired_position_is_not_shifted_again(self):
    self.path[:,1]=.04
    self.assertEqual(self.settle(narrow.NarrowRoadBias()),0.)
  def test_already_right_vehicle_clears_preference(self):
    h=narrow.NarrowRoadBias();self.settle(h)
    for line in self.m.laneLines:line.y-=.2
    self.assertEqual(h.update(self.m,self.cs,self.path,0.,.09,enabled=True),0.)
  def test_moderate_left_uncertainty_with_good_right_boundary(self):
    self.m.laneLineStds[1]=.454
    self.assertGreater(self.settle(narrow.NarrowRoadBias()),0.)
    self.m.laneLineStds[1]=.8
    self.assertEqual(self.settle(narrow.NarrowRoadBias()),0.)
  def test_right_boundary_is_not_relaxed(self):
    self.m.laneLineProbs[2]=.23
    self.assertEqual(self.settle(narrow.NarrowRoadBias()),0.)
  def test_guard_reduces_model_path_boundary_violation(self):
    guard=narrow.LaneBoundaryGuard();self.path[:,1]=.40
    for _ in range(40):shift=guard.update(self.m,self.cs,self.path,enabled=True)
    self.assertAlmostEqual(shift,-.08)
    self.assertLess(self.path[0,1]+shift,self.path[0,1])
  def test_guard_only_corrects_boundary_violation_not_normal_lane_position(self):
    self.path[:,1]=0.
    self.assertEqual(narrow.LaneBoundaryGuard().update(self.m,self.cs,self.path,enabled=True),0.)
  def test_guard_does_not_follow_weak_lines_or_override_driver(self):
    for mode in ('weak','driver','blinker','desire'):
      self.setUp();self.path[:,1]=.4
      if mode=='weak':self.m.laneLineStds[2]=.7
      if mode=='driver':self.cs.steeringPressed=True
      if mode=='blinker':self.cs.rightBlinker=True
      if mode=='desire':self.m.meta.desire=1
      self.assertEqual(narrow.LaneBoundaryGuard().update(self.m,self.cs,self.path,enabled=True),0.)

class TestBumpPhysicalEvidence(unittest.TestCase):
  def memory(self,**kw):
    h=bump.BumpMemory();lat,lon=bump.offset(37.,127.,0.,40.)
    r=dict(lat=lat,lon=lon,heading=0.,target=26.,confirmed=True,passes=10,
           manual_cap=26.,impact_learned=True,physical_passes=0,clear_passes=0)
    r.update(kw);h.records=[r];h.gps(100.,37.,127.,0.,1.);return h,r
  def test_legacy_aggregate_and_manual_counts_are_not_physical_evidence(self):
    h,r=self.memory();loaded=bump.BumpMemory(h.export());loaded.gps(100.,37.,127.,0.,1.)
    self.assertIsNone(loaded.tick(100.,35.))
    self.assertIsNotNone(loaded.tick(100.,35.,nav_distance=40.))
  def test_two_distinct_rough_passes_are_required(self):
    h,r=self.memory()
    h.finish_pass(r,4.,3,50,30.,False,True,90.)
    self.assertIsNone(h.tick(100.,30.))
    h.finish_pass(r,4.,3,50,30.,False,True,95.)
    self.assertIsNotNone(h.tick(100.,30.))
  def test_manual_clear_passes_can_retire_false_location(self):
    h,r=self.memory(physical_passes=2)
    for i in range(2):h.finish_pass(r,1.286,0,100,38.6,True,False,80.+i)
    self.assertEqual(r['clear_passes'],2)
    self.assertIsNone(h.tick(100.,35.))
    live=h.tick(100.,35.,nav_distance=40.)
    self.assertIsNotNone(live);self.assertEqual(live['target'],30.)
  def test_crawling_is_not_evidence_of_absent_bump(self):
    h,r=self.memory(physical_passes=2)
    for i in range(2):h.finish_pass(r,1.,0,100,12.,True,False,80.+i)
    self.assertEqual(r['clear_passes'],0)
    self.assertIsNotNone(h.tick(100.,30.))
  def test_new_rough_pass_restores_physical_evidence(self):
    h,r=self.memory(physical_passes=2,clear_passes=2)
    h.finish_pass(r,4.,3,100,30.,False,True,90.)
    self.assertEqual(r['clear_passes'],0)
    self.assertIsNotNone(h.tick(100.,30.))
  def test_unknown_single_impact_is_only_a_candidate(self):
    h,r=self.memory(confirmed=False,passes=0)
    h.finish_pass(r,4.,3,100,30.,False,True,90.)
    self.assertFalse(r['confirmed']);self.assertIsNone(h.tick(100.,30.))

class TestLaneReleaseAndDiagnostics(unittest.TestCase):
  def test_lane_loss_eases_then_fully_releases_to_model(self):
    import sys
    from unittest.mock import patch
    params=dict(CameraOffsetAdj=.01,LaneCenterOffset=0.,LeftEdgeOffset=0.,RightEdgeOffset=0.,AdjustLaneOffset=0.)
    dependencies={'openpilot.cereal':NS(log=NS(Desire=NS(laneChangeLeft=3,laneChangeRight=4))),
                  'openpilot.common.realtime':NS(DT_MDL=.05),
                  'openpilot.common.params':NS(Params=lambda:NS(get=lambda k,**kw:params[k]))}
    with patch.dict(sys.modules,dependencies):planner=module('test7_release_planner',ROOT/'lib/lane_planner_2.py')
    p=planner.LanePlanner();p.camera_offset=p.total_camera_offset=.01;p.path_offset=-.03
    md=model();md.meta.desireState=[];md.meta.laneWidthRight=3.;md.laneLineProbs[3]=.8
    t=np.linspace(0,3,33)
    for line in md.laneLines:line.t=t;line.y+=.2
    md.roadEdges=[NS(x=md.laneLines[0].x,y=np.full(33,-5.),t=t),NS(x=md.laneLines[0].x,y=np.full(33,5.),t=t)];md.roadEdgeStds=[1.,1.]
    cs=NS(vEgo=8.,leftBlinker=False,rightBlinker=False,steeringPressed=False)
    def step():
      p.parse_model(md);p.lane_position_ready=p.narrow_right_ready=True;p.lane_width_left=p.lane_width_right=3.
      return p.get_d_path(cs,8.,t,np.column_stack((md.laneLines[0].x,np.zeros(33),np.zeros(33))),200.)[0][0,1]
    p.lanefull_mode=True
    for _ in range(50):before=step()
    p.lanefull_mode=False;md.laneLineProbs[1]=md.laneLineProbs[2]=.1
    first=step();self.assertGreater(first,-.06);self.assertLess(first,before)
    for _ in range(10):last=step()
    self.assertAlmostEqual(last,-.06)
    p.lane_release_delta[:]=.1;p.lane_release_remaining=.35;cs.steeringPressed=True
    self.assertAlmostEqual(step(),-.06);self.assertEqual(p.lane_release_remaining,0.)
  def test_memory_source_is_logged_on_change_and_release_only(self):
    import textwrap
    source=(ROOT.parent/'carrot/carrot_serv.py').read_text(encoding='utf8')
    start=source.index('    # Record source changes once per encounter')
    end=source.index('    # A secondary bump can drive passage',start)
    block=compile(textwrap.dedent(source[start:end]),'real_memory_diagnostics','exec')
    events=[];h=NS(bump_display=dict(source='physical_memory',record_lat=37.,record_lon=127.,target=26.,distance=100.,physical_passes=2,clear_passes=0))
    env=dict(self=h,cloudlog=NS(event=lambda event,**kw:events.append((event,kw))))
    for _ in range(10):exec(block,env)
    self.assertEqual(len(events),1);self.assertEqual(events[0][1]['source'],'physical_memory')
    h.bump_display['distance']=50.;exec(block,env);self.assertEqual(len(events),1)
    h.bump_display=None
    for _ in range(5):exec(block,env)
    self.assertEqual(len(events),2);self.assertEqual(events[-1][0],'bump_memory_release')

if __name__=='__main__':unittest.main()
