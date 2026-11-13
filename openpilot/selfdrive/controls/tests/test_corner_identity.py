"""Production corner identity and slot/filter/cut-in integration without native IPC."""
import ast
from collections import deque
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest
import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.cutin_anticipation import CutInAnticipation

ROOT = Path(__file__).resolve().parents[4]
IDENTITY = ROOT/'opendbc_repo/opendbc/car/hyundai/corner_tracks.py'
spec = importlib.util.spec_from_file_location('corner_identity_under_test', IDENTITY)
identity = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = identity
spec.loader.exec_module(identity)


def load_methods(path, class_name, names, env):
  tree = ast.parse(path.read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==class_name)
  cls.bases=[];cls.decorator_list=[]
  cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
  module=ast.Module(body=ast.parse('from __future__ import annotations').body+[cls],type_ignores=[])
  exec(compile(ast.fix_missing_locations(module),str(path),'exec'),env)
  return env[class_name]


def candidate(slot=200, oid=7, age=20, x=20., y=-2.8, v=-3., vy=.6, quality=80):
  return (slot,oid,age,quality,x,y,v,vy,0.)


def road_model():
  lane=lambda y:N(x=[0.,20.,40.,80.],y=[y]*4)
  return N(laneLines=[lane(-5.4),lane(-1.8),lane(1.8),lane(5.4)],
           laneLineProbs=[.5,.99,.99,.5],laneLineStds=[.2]*4)


class TestCornerIdentity(unittest.TestCase):
  def test_slot_handoff_keeps_id_but_new_object_does_not(self):
    m=identity.CornerObjectTrackIdManager()
    key=m.assign([candidate()],1.)[200]
    self.assertEqual(m.assign([candidate(slot=205,age=21,x=19.85,y=-2.77)],1.05)[205],key)
    self.assertNotEqual(m.assign([candidate(slot=205,oid=8,age=22)],1.1)[205],key)

  def test_age_reset_and_impossible_motion_start_fresh_history(self):
    for change in (dict(age=1),dict(x=40.),dict(y=2.),dict(v=7.)):
      m=identity.CornerObjectTrackIdManager();key=m.assign([candidate()],1.)[200]
      self.assertNotEqual(m.assign([candidate(**change)],1.05)[200],key)

  def test_saturated_age_is_not_a_reacquisition(self):
    m=identity.CornerObjectTrackIdManager();key=m.assign([candidate(age=255)],1.)[200]
    self.assertEqual(m.assign([candidate(slot=201,age=255)],1.05)[201],key)

  def test_same_id_on_far_objects_is_not_merged(self):
    a,b=candidate(),candidate(slot=201,x=70.)
    m=identity.CornerObjectTrackIdManager();out=m.assign(identity.deduplicate_corner_candidates([a,b]),1.)
    self.assertEqual(len(set(out.values())),2)

  def test_duplicate_slot_handoff_prefers_older_quality(self):
    a,b=candidate(),candidate(slot=201,age=21,quality=90,x=20.1)
    self.assertEqual(identity.deduplicate_corner_candidates([a,b]),[b])
    self.assertEqual(identity.deduplicate_corner_candidates([b,a]),[b])

  def test_timeout_clock_reset_and_clear_never_reuse_ids(self):
    m=identity.CornerObjectTrackIdManager();key=m.assign([candidate()],1.)[200]
    later=m.assign([candidate()],1.2)[200];self.assertGreater(later,key)
    m.clear();self.assertGreater(m.assign([candidate()],1.3)[200],later)
    m.assign([],float('nan'));self.assertFalse(m.states)

  def test_slot_order_does_not_steal_existing_identity(self):
    m=identity.CornerObjectTrackIdManager();initial=m.assign([candidate(x=20.),candidate(slot=201,x=25.)],1.)
    updated=m.assign([candidate(slot=201,x=24.85,age=21),candidate(x=19.85,age=21)],1.05)
    self.assertEqual(initial,updated)

  def test_new_ids_retain_corner_classification(self):
    env=dict(is_corner_track_id=identity.is_corner_track_id)
    cls=load_methods(ROOT/'openpilot/selfdrive/controls/radard.py','RadarD',{'_is_corner_track','_lead_is_corner_track'},env)
    radar=cls();radar.stable_corner_ids=True
    for key in (200,219,240,249,1000,50000):
      self.assertTrue(radar._is_corner_track(N(identifier=key)))
      self.assertTrue(radar._lead_is_corner_track(dict(radarTrackId=key)))
    for key in (-1,0,32,63,220):self.assertFalse(radar._is_corner_track(N(identifier=key)))
    radar.stable_corner_ids=False
    self.assertFalse(radar._is_corner_track(N(identifier=1000)))
    self.assertFalse(radar._lead_is_corner_track(dict(radarTrackId=1000)))

  def test_stable_slot_handoff_reaches_existing_smooth_cutin_assistance(self):
    stable,legacy=CutInAnticipation(),CutInAnticipation()
    manager=identity.CornerObjectTrackIdManager();model=road_model();s_trace=[];l_trace=[]
    for i in range(30):
      t=1.+i*.05;elapsed=i*.05;y=-2.8+.6*elapsed;x=20.-3.*elapsed;slot=200+(i//5)%2
      c=candidate(slot=slot,age=20+i,x=x,y=y)
      key=manager.assign([c],t)[slot]
      for helper,track,trace in ((stable,key,s_trace),(legacy,slot,l_trace)):
        lead=N(present=True,radar=True,radarTrackId=track,dRel=x,yRel=y,dPath=y,vRel=-3.,vLead=17.,vLat=.6)
        trace.append(helper.update(N(leadsLeft=[],leadsRight=[lead]),model,20.,1.6,0.,t,t,True))
    self.assertTrue(any(v is not None for v in s_trace))
    self.assertLess(next(i for i,v in enumerate(s_trace) if v is not None),
                    next(i for i,v in enumerate(l_trace) if v is not None))
    self.assertEqual(stable.stage, "coast")
    self.assertNotEqual(legacy.stage, "coast")
    caps=[1.6]+[v for v in s_trace if v is not None]
    self.assertTrue(all(b-a >= -.7*.05-1e-8 for a,b in zip(caps,caps[1:])))
    # A stronger existing braking goal is never weakened by anticipation.
    lead=N(present=True,radar=True,radarTrackId=key,dRel=x,yRel=y,dPath=y,vRel=-3.,vLead=17.,vLat=.6)
    self.assertLessEqual(stable.update(N(leadsLeft=[],leadsRight=[lead]),model,20.,-3.,0.,2.5,2.5,True),-3.)


class TestCornerInterface(unittest.TestCase):
  def setUp(self):
    env=dict(math=math,CornerObjectTrackIdManager=identity.CornerObjectTrackIdManager,
             deduplicate_corner_candidates=identity.deduplicate_corner_candidates,
             CORNER_OBJECT_235_START_ADDR=0x235,CORNER_OBJECT_235_MSG_COUNT=20,CORNER_OBJECT_235_TRACK_ID_OFFSET=200)
    cls=load_methods(ROOT/'opendbc_repo/opendbc/car/hyundai/radar_interface.py','RadarInterface',
                     {'_update_corner_objects','_clear_point','_clear_corner_objects'},env)
    self.r=cls();self.r.v_ego=20.;self.r.pts={i:N(trackId=i,measured=False) for i in range(200,220)}
    self.r.corner_object_ids=identity.CornerObjectTrackIdManager()
    self.r.rcp_corner_objects=N(_last_update_nanos=1_000_000_000,ts_nanos={},vl={})

  def feed(self,slot=0,age=20,oid=7):
    parser=self.r.rcp_corner_objects;name=f'CORNER_RADAR_235_OBJECTS_{0x235+slot:x}'
    parser.ts_nanos[name]={'OBJ_AGE':parser._last_update_nanos}
    parser.vl[name]=dict(OBJ_AGE=age,OBJ_OBJECT_ID=oid,OBJ_QUAL_LEVEL=80,OBJ_REL_POS_X=20.,OBJ_REL_POS_Y=-2.8,
                         OBJ_REL_VEL_X=-3.,OBJ_REL_VEL_Y=.6,OBJ_REL_ACCEL_X=0.)
    self.r._update_corner_objects({0x235+slot})

  def test_inactive_slot_does_not_duplicate_live_track_id(self):
    self.feed();key=self.r.pts[200].trackId
    p=self.r.rcp_corner_objects;p.ts_nanos.clear();p._last_update_nanos+=50_000_000
    self.feed(slot=1,age=21)
    self.assertEqual(self.r.pts[201].trackId,key)
    self.assertEqual(self.r.pts[200].trackId,200)
    self.assertEqual(len({v.trackId for v in self.r.pts.values()}),20)

  def test_stale_individual_slot_is_not_refreshed_by_other_can_traffic(self):
    self.feed();self.r.rcp_corner_objects._last_update_nanos+=110_000_000
    self.r._update_corner_objects({0x248})
    self.assertFalse(self.r.pts[200].measured)

  def test_missing_data_clears_identity_but_allocator_is_monotonic(self):
    self.feed();key=self.r.pts[200].trackId
    self.r._clear_corner_objects();self.feed()
    self.assertGreater(self.r.pts[200].trackId,key)

  def test_real_acceleration_filter_survives_slot_handoff(self):
    env=dict(np=np,FirstOrderFilter=FirstOrderFilter)
    cls=load_methods(ROOT/'opendbc_repo/opendbc/car/interfaces.py','MyTrack',{'__init__','init_point','update'},env)
    filters={}
    for i in range(12):
      p=self.r.rcp_corner_objects;p.ts_nanos.clear();p._last_update_nanos=1_000_000_000+i*50_000_000
      self.feed(slot=i%2,age=20+i)
      new={}
      for point in self.r.pts.values():
        # Fill optional fields on the fake IPC point as real cereal does.
        for name in ('dRel','yRel','vRel','yvRel','vLead'):setattr(point,name,getattr(point,name,0.))
        key=point.trackId;f=filters.get(key) or cls(key,point,.05);f.update(point,0.);new[key]=f
      filters=new
    active=next(v for v in self.r.pts.values() if v.measured)
    self.assertEqual(filters[active.trackId].cnt,12)
    self.assertAlmostEqual(filters[active.trackId].aLead,0.)



class TestCornerDbcGenerator(unittest.TestCase):
  def test_runtime_generator_contains_identity_fields_for_all_slots(self):
    path=ROOT/'opendbc_repo/opendbc/dbc/generator/hyundai/hyundai_canfd_corner_radar_235.py'
    spec=importlib.util.spec_from_file_location('corner235_dbc_regression',path)
    generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    source=generator.generate()['hyundai_canfd_corner_radar_235.dbc']
    messages=source.split('BO_ ')[1:]
    self.assertEqual(len(messages),20)
    for message in messages:
      self.assertIn('SG_ OBJ_AGE : 32|8@1+',message)
      self.assertIn('SG_ OBJ_OBJECT_ID : 44|7@1+',message)

  def test_startup_rejects_missing_identity_in_any_slot(self):
    path=ROOT/'opendbc_repo/opendbc/car/hyundai/radar_interface.py'
    tree=ast.parse(path.read_text(encoding='utf8'))
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='get_corner_object_can_parser')
    function=ast.Module(body=[node],type_ignores=[])
    names=[f'CORNER_RADAR_235_OBJECTS_{addr:x}' for addr in range(0x235,0x249)]
    required={'OBJ_OBJECT_ID','OBJ_AGE','OBJ_QUAL_LEVEL','OBJ_REL_POS_X','OBJ_REL_POS_Y','OBJ_REL_VEL_X','OBJ_REL_VEL_Y','OBJ_REL_ACCEL_X'}
    for missing in (None,'OBJ_OBJECT_ID','OBJ_AGE'):
      parser=N(vl={name:dict.fromkeys(required,0.) for name in names})
      if missing:del parser.vl[names[-1]][missing]
      env=dict(CORNER_OBJECT_235_DBC='test',CORNER_OBJECT_235_START_ADDR=0x235,CORNER_OBJECT_235_MSG_COUNT=20,
               HyundaiFlags=N(CANFD=1),DBC_PATH='.',os=N(path=N(join=lambda *args:'test',exists=lambda p:True)),
               CanBus=lambda cp:N(ACAN=1),CANParser=lambda *args:parser,print=lambda *args:None)
      exec(compile(function,str(path),'exec'),env)
      result=env['get_corner_object_can_parser'](N(flags=1),True)
      self.assertIs(result,parser if missing is None else None)

if __name__=='__main__':unittest.main()
