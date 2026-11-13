import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

spec=importlib.util.spec_from_file_location('corridor',Path(__file__).resolve().parents[1]/'lib'/'model_corridor_guard.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fixture(displacement=-.25, curvature=0., width=3., rightmost=False):
  x=np.linspace(0.,60.,61)
  path=.5*curvature*x*x
  center=path-displacement
  model=NS(position=NS(x=x,y=path),
    laneLines=[NS(x=x,y=center+v) for v in (-width*1.5,-width*.5,width*.5,width*1.5)],
    laneLineProbs=[.9,.95,.95,.01 if rightmost else .9],laneLineStds=[.1]*4,
    roadEdges=[NS(x=x,y=center-5.),NS(x=x,y=center+width*.5+.6)],roadEdgeStds=[.2,.2],
    meta=NS(desire='none',laneChangeState='off',laneWidthLeft=3.,laneWidthRight=.7 if rightmost else 3.))
  cs=NS(vEgo=8.,steeringPressed=False,leftBlinker=False,rightBlinker=False,
        leftBlindspot=False,rightBlindspot=False)
  return model,cs


class TestModelCorridorGuard(unittest.TestCase):
  def setUp(self):
    self.model,self.cs=fixture()
    self.guard=module.ModelCorridorGuard()
    self.t=10.

  def tick(self,count=1,curvature=0.,guarded=None,**kwargs):
    out=[]
    for _ in range(count):
      self.t+=.05
      args=dict(preference=0.,camera_offset=0.,actuator_delay=.4,enabled=True)
      args.update(kwargs)
      out.append(self.guard.update(self.model,self.cs,curvature,
        curvature if guarded is None else guarded,self.t,self.t+.02,**args))
    return out

  def test_neutral_corridor_recovers_from_left_and_right(self):
    for side in (-1,1):
      self.setUp();self.model,self.cs=fixture(displacement=side*.25)
      v=self.tick(80)[-1]
      self.assertLess(v*side,0.)
      self.assertLessEqual(abs(v*self.cs.vEgo**2),.2+1e-9)

  def test_centered_neutral_corridor_does_not_add_preference(self):
    self.model,self.cs=fixture(displacement=0.)
    self.assertTrue(all(v==0. for v in self.tick(80)))

  def test_rightmost_narrow_road_adds_small_right_preference(self):
    self.model,self.cs=fixture(displacement=0.,width=2.7,rightmost=True)
    self.assertGreater(self.tick(80)[-1],0.)
    self.assertTrue(self.guard.rightmost)
    self.assertAlmostEqual(self.guard.target_offset,.04)

  def test_249m_lane_retains_room_bounded_right_preference(self):
    self.model,self.cs=fixture(displacement=0.,width=2.49,rightmost=True)
    self.assertGreater(self.tick(80)[-1],0.)

  def test_wide_rightmost_lane_does_not_invent_edge_preference(self):
    self.model,self.cs=fixture(displacement=0.,width=3.7,rightmost=True)
    self.assertTrue(all(v==0. for v in self.tick(80)))

  def test_isolated_ramp_with_two_road_edges_keeps_center_even_with_yellow_left(self):
    self.model,self.cs=fixture(displacement=0.,width=2.8,rightmost=True)
    self.model.roadEdges[0].y=self.model.laneLines[1].y-.4
    self.cs.leftLaneLine=21
    self.assertTrue(all(v==0. for v in self.tick(80,preference=-.07)))
    self.assertEqual(self.guard.corridor_type,'isolated')
    self.assertFalse(self.guard.rightmost)
    self.assertEqual(self.guard.target_offset,0.)

  def test_one_lane_each_way_yellow_center_with_road_beyond_it_can_prefer_right(self):
    self.model,self.cs=fixture(displacement=0.,width=2.8,rightmost=True)
    self.model.meta.laneWidthLeft=.8
    self.model.laneLineProbs[0]=.05
    self.cs.leftLaneLine=21
    self.assertGreater(self.tick(80)[-1],0.)
    self.assertTrue(self.guard.rightmost)

  def test_isolated_left_and_right_ramps_keep_required_curve(self):
    for side in (-1,1):
      self.setUp();k=side*.012
      self.model,self.cs=fixture(displacement=0.,width=2.8,rightmost=True,curvature=k)
      self.model.roadEdges[0].y=self.model.laneLines[1].y-.4
      self.assertTrue(all(abs(v-k)<1e-9 for v in self.tick(80,curvature=k)))
      self.assertEqual(self.guard.corridor_type,'isolated')

  def test_yellow_line_alone_does_not_classify_opposing_road(self):
    self.model,self.cs=fixture(displacement=0.,width=2.8,rightmost=True)
    self.model.meta.laneWidthLeft=.8
    self.model.laneLineProbs[0]=.05
    self.model.roadEdgeStds[0]=1.2
    self.cs.leftLaneLine=21
    self.assertTrue(all(v==0. for v in self.tick(80)))
    self.assertFalse(self.guard.rightmost)

  def test_already_right_of_target_does_not_push_further_right(self):
    self.model,self.cs=fixture(displacement=.2,width=2.8,rightmost=True)
    self.assertLess(self.tick(80)[-1],0.)

  def test_persistent_curve_boundary_protects_in_both_directions(self):
    for side in (-1,1):
      self.setUp();k=side*.012
      self.model,self.cs=fixture(displacement=side*.65,curvature=k)
      self.model.laneLineProbs[2 if side>0 else 1]=.72
      self.model.laneLineStds[2 if side>0 else 1]=.32
      v=self.tick(80,curvature=k)[-1]
      self.assertLess(abs(v),abs(k))
      self.assertGreaterEqual(v*k,.5*k*k-1e-10)
      self.assertLessEqual(abs((v-k)*self.cs.vEgo**2),.600001)

  def test_short_confidence_dip_uses_current_line_with_recent_identity(self):
    k=-.012;self.model,self.cs=fixture(displacement=-.65,curvature=k)
    self.tick(40,curvature=k)
    before=self.guard.boundary_accel
    self.model.laneLineProbs[1]=.5;self.model.laneLineStds[1]=.42
    self.tick(5,curvature=k)
    self.assertGreater(self.guard.boundary_accel,0.)
    self.tick(40,curvature=k)
    self.assertLess(self.guard.boundary_accel,before)

  def test_weak_line_without_history_cannot_start_boundary_protection(self):
    self.model,self.cs=fixture(displacement=-.65,curvature=-.012)
    self.model.laneLineProbs[1]=.5;self.model.laneLineStds[1]=.42
    self.tick(15,curvature=-.012)
    self.assertEqual(self.guard.boundary_accel,0.)

  def test_driver_blinkers_maneuver_disabled_and_stale_data_reset(self):
    for flag in ('steeringPressed','leftBlinker','rightBlinker','desire','laneChangeState','disabled','stale'):
      self.setUp();self.tick(40)
      if flag in ('steeringPressed','leftBlinker','rightBlinker'):setattr(self.cs,flag,True)
      if flag=='desire':self.model.meta.desire='turnLeft'
      if flag=='laneChangeState':self.model.meta.laneChangeState='laneChangeStarting'
      if flag=='stale':v=self.guard.update(self.model,self.cs,0.,0.,self.t,self.t+.3)
      else:v=self.tick(enabled=flag!='disabled')[0]
      self.assertEqual(v,0.,flag)
      self.assertEqual(self.guard.accel,0.,flag)

  def test_blindspot_vetoes_recovery_toward_that_side(self):
    self.cs.rightBlindspot=True
    self.assertTrue(all(v==0. for v in self.tick(80)))

  def test_existing_boundary_correction_is_not_added_twice(self):
    self.tick(80,guarded=.003)
    self.assertAlmostEqual(self.tick(1,guarded=.003)[-1],.003)

  def test_short_or_nan_geometry_never_starts_correction(self):
    for mode in ('short','nan'):
      self.setUp()
      if mode=='short':self.model.position.x=np.linspace(0,2,61)
      else:self.model.position.y[2]=np.nan
      self.assertTrue(all(v==0. for v in self.tick(40)))

  def test_unique_frames_and_bounded_jerk(self):
    out=self.tick(80)
    a=np.asarray(out)*self.cs.vEgo**2
    self.assertLessEqual(max(abs(np.diff(a))),.5*.05+1e-9)
    before=self.guard.position_accel
    for i in range(10):
      self.guard.update(self.model,self.cs,0.,0.,self.t,self.t+.02+i*.01)
    self.assertEqual(self.guard.position_accel,before)

  def test_current_and_preview_must_agree_before_position_recovery(self):
    self.model.position.y+=.02*self.model.position.x**2
    self.tick(50)
    self.assertEqual(self.guard.position_accel,0.)

  def test_ordinary_road_respects_explicit_left_preference(self):
    self.model,self.cs=fixture(displacement=0.)
    self.assertLess(self.tick(80,preference=-.07)[-1],0.)

  def test_native_reader_style_integer_only_lists(self):
    class Indexed(list):
      def __getitem__(self,i):
        if not isinstance(i,int):raise TypeError('integer required')
        return super().__getitem__(i)
    for field in ('laneLines','roadEdges'):setattr(self.model,field,Indexed(getattr(self.model,field)))
    self.assertGreater(self.tick(80)[-1],0.)

  def test_production_model_mode_wiring_and_stale_car_gate(self):
    import ast
    from unittest.mock import MagicMock
    source=Path(__file__).resolve().parents[1]/'controlsd.py'
    tree=ast.parse(source.read_text(encoding='utf8'))
    branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and
                ast.unparse(n.test)=="self.CP.brand == 'hyundai'" and
                'self.model_corridor_guard.update(' in ast.unparse(n))
    class SM(dict):
      logMonoTime={'modelV2':int(9.95e9),'carState':int(9.98e9)}
      def all_checks(self,names):return True
    obj=NS(CP=NS(brand='hyundai'),sm=SM(),lane_center_preference=0.,camera_offset=.01,
           model_corridor_guard=MagicMock())
    obj.model_corridor_guard.update.return_value=.003
    model=NS(action=NS(desiredCurvature=.002))
    env=dict(self=obj,model_v2=model,CS=self.cs,guard_now=10.,steer_actuator_delay=.3,
             guarded_curvature=.0018,position_curvature=MagicMock(return_value=-.001))
    code=compile(ast.Module(body=[branch],type_ignores=[]),'production_corridor_wiring','exec')
    exec(code,env)
    call=obj.model_corridor_guard.update.call_args
    self.assertEqual(call.args[2:4],(.002,.0018))
    self.assertTrue(call.kwargs['enabled'])
    self.assertEqual(env['guarded_curvature'],.003)
    env['position_curvature'].assert_not_called()
    obj.sm.logMonoTime['carState']=int(9.8e9)
    exec(code,env)
    self.assertFalse(obj.model_corridor_guard.update.call_args.kwargs['enabled'])
    obj.CP.brand='toyota'
    obj.model_corridor_guard.update.reset_mock()
    exec(code,env)
    obj.model_corridor_guard.update.assert_not_called()
    env['position_curvature'].assert_called_once()

  def test_production_corridor_guard_precedes_smoothing_and_actuator_limits(self):
    import ast
    path=Path(__file__).resolve().parents[1]/'controlsd.py'
    tree=ast.parse(path.read_text(encoding='utf8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Controls')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='state_control')
    s=ast.unparse(method)
    self.assertLess(s.index('self.model_corridor_guard.update('),s.index('smooth_value(guarded_curvature'))
    self.assertLess(s.index('smooth_value(guarded_curvature'),s.index('clip_curvature(CS.vEgo'))
    self.assertLess(s.index('clip_curvature(CS.vEgo'),s.index('self.LaC.update('))
    reset=next(n for n in ast.walk(method) if isinstance(n,ast.If) and
               ast.unparse(n.test)=='not model_corridor_guard_used')
    self.assertEqual(ast.unparse(reset.body[0]),'self.model_corridor_guard.reset()')


if __name__=='__main__':unittest.main()
