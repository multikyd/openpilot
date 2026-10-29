"""Conditional narrow rightmost preference, without changing camera Params."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import math
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('narrow_bias_test',Path(__file__).parents[1]/'lib/narrow_road_bias.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def model(width=2.9,right_space=.7,outer=.01):
  x=np.linspace(0.,40.,33)
  return NS(laneLines=[NS(x=x,y=np.ones(33)*y) for y in (-4.,-width/2,width/2,4.)],
            laneLineProbs=[.6,.9,.9,outer],laneLineStds=[.1,.08,.08,.1],
            roadEdges=[NS(x=x,y=np.full(33,-5.)),NS(x=x,y=np.full(33,2.))],roadEdgeStds=[1.,.3],
            meta=NS(laneWidthRight=right_space,laneChangeState=0,desire=0))


class TestNarrowRoadBias(unittest.TestCase):
  def setUp(self):
    self.h=m.NarrowRoadBias();self.md=model()
    self.cs=NS(vEgo=8.,leftBlinker=False,rightBlinker=False,steeringPressed=False,leftLaneLine=21)
    self.path=np.column_stack([np.linspace(0.,40.,33),np.full(33,-.06),np.zeros(33)])
  def tick(self,count=1,**kw):
    args=dict(model=self.md,cs=self.cs,base_path=self.path,curvature=0.,requested_shift=.06,enabled=True)
    args.update(kw)
    return [self.h.update(**args) for _ in range(count)]
  def test_real_capnp_builder_and_reader_support_narrow_road_preference(self):
    from openpilot.cereal import log
    source=model()
    actual=log.ModelDataV2.new_message()
    for key in ('laneLines','roadEdges'):
      setattr(actual,key,[dict(x=line.x.tolist(),y=line.y.tolist()) for line in getattr(source,key)])
    for key in ('laneLineProbs','laneLineStds','roadEdgeStds'):
      setattr(actual,key,getattr(source,key))
    actual.meta.laneWidthRight=source.meta.laneWidthRight
    for message in (actual,actual.as_reader()):
      self.h=m.NarrowRoadBias()
      result=self.tick(160,model=message)
      self.assertTrue(all(math.isfinite(v) for v in result))
      self.assertGreater(result[-1],0.)
      self.assertLessEqual(result[-1],.120001)

  def test_single_lane_and_rightmost_both_confirm_and_shift_right(self):
    for left_color in (21,11,2):
      self.setUp();self.cs.leftLaneLine=left_color
      a=self.tick(160)
      self.assertEqual(a[:19],[0.]*19)
      self.assertAlmostEqual(a[-1],.10)
      self.assertLessEqual(max(np.diff(a)),.025*.05+1e-9)
  def test_no_shift_in_wide_middle_or_left_lane(self):
    for md in (model(width=3.6),model(right_space=3.2,outer=.8),model(right_space=3.2,outer=.2)):
      self.setUp();self.assertEqual(self.tick(200,model=md),[0.]*200)
  def test_transition_to_neighbor_lane_removes_shift(self):
    self.tick(160);self.assertGreater(self.h.shift,0.)
    self.assertEqual(self.tick(1,model=model(right_space=3.2,outer=.8)),[0.])
  def test_boundary_space_limits_complete_base_path(self):
    self.path[:,1]=.18
    a=self.tick(160)
    # Right line1.45 minus centre margin1.25 minus uncertainty.024 minus base.18.
    self.assertLessEqual(max(a),0.)
  def test_trusted_road_edge_has_priority(self):
    self.md.roadEdges[1].y[:]=1.3
    self.assertEqual(self.tick(160),[0.]*160)
  def test_weak_boundary_does_not_build_preference(self):
    self.md.laneLineProbs[2]=.25
    self.assertEqual(self.tick(160),[0.]*160)
  def test_brief_zigzag_probability_dip_does_not_erase_context(self):
    self.tick(80)
    before=self.h.confirm
    self.md.laneLineProbs[2]=.15;self.tick(6)
    self.assertGreater(self.h.confirm,before-.5)
    self.md.laneLineProbs[2]=.9
    self.assertGreater(self.tick(20)[-1],0.)
  def test_long_missing_context_clears_preference(self):
    self.tick(160);self.md.laneLineProbs[2]=.1
    self.assertEqual(self.tick(25)[-1],0.)
  def test_blinker_driver_override_and_disable_clear(self):
    for key in ('leftBlinker','rightBlinker','steeringPressed'):
      self.setUp();self.tick(160);setattr(self.cs,key,True)
      self.assertEqual(self.tick(),[0.])
    self.setUp();self.tick(160);self.assertEqual(self.tick(enabled=False),[0.])
  def test_lane_change_and_turn_desire_clear(self):
    for key in ('laneChangeState','desire'):
      self.setUp();self.tick(160);setattr(self.md.meta,key,1)
      self.assertEqual(self.tick(),[0.])
  def test_high_speed_stop_or_invalid_data_clear(self):
    for speed in (0.,20.,math.nan):
      self.setUp();self.tick(160);self.cs.vEgo=speed
      self.assertEqual(self.tick(),[0.])
  def test_strong_curve_does_not_build_preference(self):
    self.assertEqual(self.tick(160,curvature=.01),[0.]*160)
  def test_request_is_bounded_and_never_left(self):
    self.assertAlmostEqual(self.tick(220,requested_shift=1.)[-1],.10)
    self.setUp();self.assertEqual(self.tick(160,requested_shift=-.1),[0.]*160)
  def test_bad_lane_geometry_cannot_poison_state(self):
    self.tick(160);self.md.laneLines[2].y[0]=math.nan
    self.assertEqual(self.tick(),[0.])
    self.assertTrue(math.isfinite(self.h.shift))
  def test_future_narrowing_caps_shift_before_it_arrives(self):
    self.md.laneLines[2].y[:8]=np.linspace(1.45,1.15,8)
    self.assertEqual(self.tick(160),[0.]*160)


class TestNarrowPlannerWiring(unittest.TestCase):
  def test_global_profiles_and_manual_parameters_remain_original(self):
    source=(Path(__file__).parents[1]/'lib/lane_planner_2.py').read_text(encoding='utf8')
    import ast
    tree=ast.parse(source)
    profiles=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and
                  any(isinstance(t,ast.Name) and t.id=='LANE_CAMERA_PROFILES' for t in n.targets))
    self.assertEqual(profiles['single'],.03)
    self.assertEqual(profiles['unknown'],.01)
    self.assertNotIn('.put(',source)
  def test_new_preview_horizon_keeps_original_default(self):
    import ast
    path=Path(__file__).parents[1]/'lib/lateral_planner.py'
    tree=ast.parse(path.read_text(encoding='utf8'));f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='get_lane_center_curvature')
    env=dict(np=np,MIN_SPEED=1.);exec(compile(ast.Module(body=[f],type_ignores=[]),str(path),'exec'),env)
    t=np.array([0.,.75,1.5]);yaw=np.array([0.,.01,.1]);v=np.ones(3)*10.
    self.assertAlmostEqual(env['get_lane_center_curvature'](0.,t,yaw,v),.01)
    self.assertAlmostEqual(env['get_lane_center_curvature'](0.,t,yaw,v,preview_horizon=.75),.001)


class TestNarrowPathIntegration(unittest.TestCase):
  def test_same_narrow_condition_works_in_model_and_lane_modes(self):
    import sys
    from unittest.mock import patch
    sys.path.insert(0,str(Path(__file__).parents[4]))
    params=dict(CameraOffsetAdj=.01,LaneCenterOffset=-.05,LeftEdgeOffset=0.,RightEdgeOffset=0.,AdjustLaneOffset=0.)
    dependencies={
      'openpilot.cereal':NS(log=NS(Desire=NS(laneChangeLeft=3,laneChangeRight=4))),
      'openpilot.common.realtime':NS(DT_MDL=.05),
      'openpilot.common.params':NS(Params=lambda:NS(get=lambda k,**kw:params[k]))}
    spec=importlib.util.spec_from_file_location('narrow_path_actual',Path(__file__).parents[1]/'lib/lane_planner_2.py')
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,dependencies):spec.loader.exec_module(module)
    cs=NS(vEgo=8.,leftBlinker=False,rightBlinker=False,steeringPressed=False,leftLaneLine=21)
    def md(width=2.9,neighbor=.7,outer=.01):
      obj=model(width,neighbor,outer);obj.meta.desireState=[]
      for line in obj.laneLines+obj.roadEdges:line.t=np.linspace(0.,3.,33)
      return obj
    for lane_mode in (False,True):
      reference=module.LanePlanner();modified=module.LanePlanner()
      for obj in (reference,modified):
        obj.camera_offset=obj.total_camera_offset=.01;obj.path_offset=-.03
      def step(observation):
        outputs=[]
        for obj,enable in ((reference,False),(modified,True)):
          obj.lanefull_mode=lane_mode;obj.parse_model(observation)
          obj.lane_width_left=3.2;obj.lane_width_right=observation.meta.laneWidthRight
          obj.lane_position_ready=True;obj.narrow_right_ready=enable
          obj.narrow_road_curvature=0.;obj.lane_center_curvature=0.
          path=np.column_stack([np.linspace(0.,40.,33),np.zeros(33),np.zeros(33)])
          out,active=obj.get_d_path(cs,8.,np.linspace(0.,3.,33),path,200.)
          outputs.append(out[:,1].copy())
        return outputs
      for _ in range(300):before,after=step(md())
      delta=after-before
      self.assertGreater(delta[0],.01)
      self.assertLessEqual(np.max(delta),.120001)
      np.testing.assert_allclose(delta,np.full(33,delta[0]))
      before,after=step(md(3.6,3.2,.8))
      np.testing.assert_allclose(before,after)
    self.assertEqual(params['CameraOffsetAdj'],.01)
    self.assertEqual(params['LaneCenterOffset'],-.05)


if __name__=='__main__':unittest.main()
