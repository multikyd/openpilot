from pathlib import Path
import importlib.util
from types import SimpleNamespace as NS
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('stop_approach_test',Path(__file__).parents[1]/'lib/stop_approach.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def lead(**kw):
  d=dict(present=True,radar=True,radarTrackId=59,modelProb=.99,dPath=0.,dRel=20.,vRel=-1.,vLead=5.,aLeadK=-1.2)
  d.update(kw)
  return NS(**d)


class TestStopApproach(unittest.TestCase):
  def setUp(self):
    self.h=m.StopApproach();self.t=0.;self.distance=20.
  def tick(self,count=1,**kw):
    out=[]
    for _ in range(count):
      self.t+=.05;self.distance-=.05
      values=dict(lead=lead(dRel=self.distance),other=lead(present=False),speed=6.,accel=-.7,
                  desired_distance=10.,stop_distance=3.6,stamp=self.t,now=self.t,enabled=True)
      values.update(kw);out.append(self.h.update(**values))
    return out
  def arm(self):self.tick(50)
  def ease(self,count=20,**kw):
    values=dict(lead=lead(dRel=self.distance-.05,aLeadK=-.5))
    # The same stable stationary distance is valid within the continuity tolerance.
    values.update(kw)
    return self.tick(count,**values)
  def test_stable_braking_then_easing_confirms_and_ramps(self):
    self.arm();a=self.ease(40)
    self.assertEqual(a[:5],[0.]*5);self.assertGreater(a[-1],.4)
    self.assertLessEqual(max(a),.8)
    self.assertLessEqual(max(b-a for a,b in zip(a,a[1:])),.35*.05+1e-9)
  def test_new_target_requires_own_history(self):
    self.arm();self.assertEqual(self.ease(20,lead=lead(radarTrackId=60,aLeadK=-.5)),[0.]*20)
  def test_no_braking_history_means_no_change(self):
    self.assertEqual(self.ease(80),[0.]*80)
  def test_strong_braking_and_fast_closing_have_no_bias(self):
    for values in (dict(aLeadK=-1.2),dict(aLeadK=-.5,vRel=-2.2,vLead=3.8)):
      self.setUp();self.arm();self.ease()
      self.assertEqual(self.ease(1,lead=lead(**values)),[0.])
  def test_short_gap_rejects_reference_shift(self):
    self.arm();self.assertEqual(self.ease(30,desired_distance=30.),[0.]*30)
  def test_high_speed_standstill_and_stationary_lead_keep_original_goal(self):
    for kwargs in (dict(speed=12.,lead=lead(aLeadK=-.5,vLead=11.)),dict(speed=.2,lead=lead(aLeadK=-.5,vLead=.1,vRel=-.1)),
                   dict(lead=lead(aLeadK=-.5,vLead=0.,vRel=-6.))):
      self.setUp();self.arm();self.assertEqual(self.ease(20,**kwargs),[0.]*20)
  def test_disabled_stale_and_invalid_data_reset(self):
    for kwargs in (dict(enabled=False),dict(now=100.),dict(speed=float('nan'))):
      self.setUp();self.arm();self.ease()
      self.assertEqual(self.ease(1,**kwargs),[0.]);self.assertEqual(self.h.bias,0.)
  def test_vision_low_probability_and_off_path_are_untouched(self):
    for kwargs in (dict(radar=False),dict(modelProb=.5),dict(dPath=2.)):
      self.setUp();self.arm();self.assertEqual(self.ease(20,lead=lead(aLeadK=-.5,**kwargs)),[0.]*20)
  def test_different_near_second_lead_vetoes(self):
    self.arm();self.assertEqual(self.ease(20,other=lead(radarTrackId=70,dRel=10.)),[0.]*20)
  def test_duplicate_lead_one_and_two_are_not_a_false_veto(self):
    self.arm();l=lead(dRel=self.distance,aLeadK=-.5)
    self.assertGreater(self.ease(30,lead=l,other=l)[-1],0.)
    self.assertFalse(self.h.same_lead(l,lead(radarTrackId=60)))
    self.assertFalse(self.h.same_lead(l,lead(dRel=l.dRel+2.)))
  def test_bias_fades_when_easing_condition_ends(self):
    self.arm();self.ease(40)
    self.assertGreater(self.h.bias,0.)
    self.assertEqual(self.ease(30,accel=.1)[-1],0.)
  def test_duplicate_radar_stamp_cannot_build_confirmation(self):
    self.arm();l=lead(dRel=self.distance,aLeadK=-.5)
    args=dict(lead=l,other=lead(present=False),speed=6.,accel=-.7,desired_distance=10.,stop_distance=3.6,
              stamp=self.t,now=self.t,enabled=True)
    self.assertEqual([self.h.update(**args) for _ in range(100)],[0.]*100)


class TestDistanceReference(unittest.TestCase):
  def ref(self,**kw):
    v=dict(bias=.8,speed_plan=np.array([5.,3.,1.,0.]),lead_speed_plan=np.array([4.,2.,.2,0.]),
           lead_obstacle=np.array([20.,25.,30.,31.]),other_obstacle=np.full(4,100.),
           cruise_obstacle=np.full(4,50.),stop_obstacle=np.full(4,1000.))
    v.update(kw);return m.stop_approach_reference(**v)
  def test_cost_shift_fades_at_predicted_standstill(self):
    r=self.ref();self.assertLess(r[0],0.);self.assertEqual(list(r[2:]),[0.,0.])
    self.assertAlmostEqual(-r[0]*15.,.8)
  def test_other_lead_and_stop_obstacle_keep_original_cost(self):
    for kw in (dict(other_obstacle=np.ones(4)),dict(stop_obstacle=np.ones(4))):
      np.testing.assert_array_equal(self.ref(**kw),np.zeros(4))
  def test_disabled_and_nonfinite_values_are_zero(self):
    for kw in (dict(bias=0.),dict(bias=float('nan')),dict(speed_plan=np.full(4,float('nan')))):
      np.testing.assert_array_equal(self.ref(**kw),np.zeros(4))
  def test_reference_does_not_mutate_input_obstacles(self):
    x=np.array([20.,25.,30.,31.]);self.ref(lead_obstacle=x)
    np.testing.assert_array_equal(x,[20.,25.,30.,31.])


class TestMpcIntegration(unittest.TestCase):
  def test_cost_reference_is_reset_and_only_distance_cost_changes(self):
    import ast
    from unittest.mock import MagicMock
    path=Path(__file__).parents[1]/'lib/longitudinal_mpc_lib/long_mpc.py'
    tree=ast.parse(path.read_text(encoding='utf8'));cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LongitudinalMpc')
    update=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update')
    start=next(i for i,n in enumerate(update.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='approach_bias')
    end=next(i for i,n in enumerate(update.body[start:],start) if isinstance(n,ast.Assign) and
             isinstance(n.targets[0],ast.Subscript) and ast.unparse(n.targets[0])=='self.yref[:, 1]')
    block=compile(ast.Module(body=update.body[start:end],type_ignores=[]),'actual_stop_approach_integration','exec')
    helper=MagicMock();helper.update.return_value=.5;helper.same_lead.return_value=False
    planner=NS(stop_approach=helper,yref=np.zeros((4,6)),v_solution=np.full(4,5.),desired_distance=10.,turn_loss_limit=None,
               max_a=1.,_stop_approach_active=False,params=np.ones((4,8)))
    env=dict(np=np,self=planner,radarstate=NS(leadOne=lead(aLeadK=-.5),leadTwo=lead(present=False)),v_ego=5.,a_ego=-.5,
             stop_distance=3.6,radar_stamp=10.,observation_time=10.,allow_stop_approach=True,mode='acc',reset_state=False,
             carrot=NS(xState=0,signal_follow=NS(strength=0.),signal_stop_approach=NS(accel_cap=None)),
             XState=NS(e2eStop=1,e2eStopped=2,e2ePrepare=3),cutin_accel_limit=None,jam_accel=1.,
             lead_xv_0=np.column_stack([np.arange(4),np.full(4,5.)]),lead_0_obstacle=np.full(4,20.),
             lead_1_obstacle=np.full(4,100.),cruise_obstacle=np.full(4,50.),x2=np.full(4,1000.),N=3,cloudlog=MagicMock(),
             stop_approach_reference=m.stop_approach_reference)
    exec(block,env)
    self.assertTrue(helper.update.call_args.kwargs['enabled'])
    self.assertTrue(np.all(planner.yref[:,0]<0.))
    np.testing.assert_array_equal(planner.params,np.ones((4,8)))
    np.testing.assert_array_equal(planner.yref[:,1:],np.zeros((4,5)))
    helper.update.return_value=0.;exec(block,env)
    np.testing.assert_array_equal(planner.yref[:,0],np.zeros(4))
    for key,value in (('allow_stop_approach',False),('reset_state',True),('mode','blended'),('cutin_accel_limit',.2),('jam_accel',.5)):
      old=env[key];env[key]=value;exec(block,env)
      self.assertFalse(helper.update.call_args.kwargs['enabled'],key);env[key]=old
    env['carrot'].signal_follow.strength=.2;exec(block,env)
    self.assertFalse(helper.update.call_args.kwargs['enabled'])


class TestPlannerApproachGate(unittest.TestCase):
  def test_pedals_mode_signals_freshness_and_lateral_override(self):
    import ast
    tree=ast.parse((Path(__file__).parents[1]/'lib/longitudinal_planner.py').read_text(encoding='utf8'))
    expression=next(k.value for n in ast.walk(tree) if isinstance(n,ast.Call) for k in n.keywords if k.arg=='allow_stop_approach')
    code=compile(ast.Expression(body=expression),'actual_planner_stop_gate','eval')
    class SM(dict):
      def all_checks(self,*args):return self.checks
    sm=SM(selfdriveState=NS(enabled=True));sm.checks=True
    cs=NS(gasPressed=False,brakePressed=False,leftBlinker=False,rightBlinker=False,steeringPressed=False,steeringAngleDeg=0.)
    planner=NS(CP=NS(openpilotLongitudinalControl=True),output_should_stop=False)
    carrot=NS(desireState=0.,myDrivingMode=NS(value=3))
    env=dict(sm=sm,cs=cs,self=planner,carrot=carrot,reset_state=False,force_slow_decel=False,fresh=True)
    self.assertTrue(eval(code,env))
    for obj,key,value in [(cs,'gasPressed',True),(cs,'brakePressed',True),(cs,'leftBlinker',True),
                          (cs,'rightBlinker',True),(cs,'steeringPressed',True),(cs,'steeringAngleDeg',40.),
                          (carrot,'desireState',1.),(carrot.myDrivingMode,'value',1),
                          (carrot.myDrivingMode,'value',2),(carrot.myDrivingMode,'value',4),
                          (planner,'output_should_stop',True),(sm['selfdriveState'],'enabled',False)]:
      old=getattr(obj,key);setattr(obj,key,value);self.assertFalse(eval(code,env),key);setattr(obj,key,old)
    for key in ('reset_state','force_slow_decel'):
      env[key]=True;self.assertFalse(eval(code,env));env[key]=False
    env['fresh']=False;self.assertFalse(eval(code,env));env['fresh']=True
    sm.checks=False;self.assertFalse(eval(code,env))


if __name__=='__main__':unittest.main()
