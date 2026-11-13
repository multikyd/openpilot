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
  def test_close_moving_queue_can_ease_before_desired_gap_is_fully_restored(self):
    self.arm()
    self.distance = 5.8
    l = lead(dRel=5.8, vLead=2.6, vRel=-.4, aLeadK=-.6)
    self.h.distance, self.h.relative_speed = 5.82, -.4
    values = self.ease(20, lead=l, speed=3., accel=-1.5, desired_distance=7.4)
    self.assertGreater(values[-1], .3)
    self.assertLessEqual(values[-1], .45)
    # Renewed braking immediately returns to the original soft distance goal.
    l.aLeadK = -1.1
    self.assertEqual(self.ease(1, lead=l, speed=3., accel=-1.5, desired_distance=7.4), [0.])

  def test_close_queue_with_insufficient_room_has_no_easing(self):
    self.arm()
    self.h.distance, self.h.relative_speed = 3.52, -.4
    l = lead(dRel=3.5, vLead=2.6, vRel=-.4, aLeadK=-.6)
    self.assertEqual(self.ease(20, lead=l, speed=3., accel=-1.5, desired_distance=7.4), [0.]*20)

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
    for key,value in (('allow_stop_approach',False),('reset_state',True),('mode','blended'),('cutin_accel_limit',.2)):
      old=env[key];env[key]=value;exec(block,env)
      self.assertFalse(helper.update.call_args.kwargs['enabled'],key);env[key]=old
    env['jam_accel']=.5;exec(block,env)
    self.assertTrue(helper.update.call_args.kwargs['enabled'])
    env['radarstate'].leadOne.vLead=.2;exec(block,env)
    self.assertFalse(helper.update.call_args.kwargs['enabled'])
    env['radarstate'].leadOne.vLead=5.;env['jam_accel']=1.
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


class TestTerminalStopRelease(unittest.TestCase):
  def setUp(self):
    self.h=m.TerminalStopRelease()
    self.t=10.
    self.l=lead(dRel=3.,vLead=0.,vRel=-.5,aLeadK=0.)

  def tick(self, count=1, **changes):
    values=dict(lead=self.l,other=lead(present=False),speed=.5,command=-1.2,previous=-1.2,
                stamp=self.t,now=self.t,enabled=True,dt=.01)
    values.update(changes)
    result=None
    for _ in range(count):
      self.t+=.05
      values['stamp']=self.t;values['now']=self.t
      result=self.h.update(**values)
    return result

  def arm(self):
    self.tick(15)

  def test_confirmed_stop_eases_at_bounded_rate_and_stays_negative(self):
    self.arm()
    previous=-1.2
    for _ in range(100):
      result=self.tick(previous=previous)
      self.assertLessEqual(result-previous,.0150001)
      self.assertLessEqual(result,-.5)
      self.assertGreaterEqual(result,-1.2)
      previous=result
    self.assertAlmostEqual(previous,-.5)

  def test_insufficient_room_and_high_closing_keep_original_braking(self):
    self.arm()
    self.l.dRel=2.55;self.l.vRel=-1.4
    self.assertEqual(self.tick(speed=1.4),-1.2)

  def test_strong_braking_and_gentle_or_positive_commands_unchanged(self):
    self.arm()
    for command in (-3.,-2.,-.4,0.,.3):
      self.assertEqual(self.tick(command=command),command)

  def test_no_relaxation_for_stale_or_unconfirmed_or_changed_lead(self):
    self.assertEqual(self.tick(),-1.2)
    self.arm()
    self.l.radarTrackId+=1
    self.assertEqual(self.tick(),-1.2)
    self.arm()
    result=self.h.update(self.l,lead(present=False),.5,-1.2,-1.2,self.t,self.t+.3,True,.01)
    self.assertEqual(result,-1.2)

  def test_new_other_obstacle_and_missing_radar_veto_release(self):
    self.arm()
    self.assertEqual(self.tick(other=lead(dRel=3.,radarTrackId=20)),-1.2)
    self.arm();self.l.radar=False
    self.assertEqual(self.tick(),-1.2)

  def test_overlapping_vision_duplicate_does_not_interrupt_release(self):
    self.arm()
    duplicate=lead(radar=False,radarTrackId=-1,dRel=3.3,vRel=-.4,dPath=.05)
    self.assertGreater(self.tick(other=duplicate),-1.2)
    duplicate.dPath=.8
    self.assertEqual(self.tick(other=duplicate),-1.2)

  def test_moving_lead_or_high_speed_cannot_relax(self):
    self.arm();self.l.vLead=.5;self.l.vRel=0.
    self.assertEqual(self.tick(),-1.2)
    self.l.vLead=0.;self.l.vRel=-2.
    self.assertEqual(self.tick(speed=2.),-1.2)

  def test_duplicate_observations_cannot_build_confirmation(self):
    for _ in range(1000):
      result=self.h.update(self.l,lead(present=False),.5,-1.2,-1.2,10.,10.01,True,.01)
    self.assertEqual(result,-1.2)
    self.assertEqual(self.h.confirm,0.)

  def test_stronger_terminal_request_can_ease_after_lead_has_settled(self):
    self.tick(30)
    self.assertGreater(self.tick(command=-2.5,previous=-2.5),-2.5)
    self.assertLessEqual(self.tick(command=-2.5,previous=-2.5),-2.485+1e-9)

  def test_strong_terminal_release_requires_long_confirmation_and_ended_lead_braking(self):
    self.arm()
    self.assertEqual(self.tick(command=-2.5,previous=-2.5),-2.5)
    self.tick(30);self.l.aLeadK=-1.
    self.assertEqual(self.tick(command=-2.5,previous=-2.5),-2.5)
    self.l.aLeadK=0.;self.l.vRel=-1.
    self.assertEqual(self.tick(speed=1.,command=-2.5,previous=-2.5),-2.5)

  def test_strong_terminal_request_cannot_ease_without_stopping_room(self):
    self.tick(30);self.l.dRel=2.55;self.l.vRel=-1.2
    # Closing speed includes radar evidence, even when measured ego speed is low.
    self.assertEqual(self.tick(command=-2.5,previous=-2.5),-2.5)
    self.l.aLeadK=float('nan')
    self.assertEqual(self.tick(command=-2.5,previous=-2.5),-2.5)

  def test_actual_longcontrol_update_applies_release_only_before_hold(self):
    import ast
    tree=ast.parse((Path(__file__).parents[1]/'lib/longcontrol.py').read_text(encoding='utf8'))
    nodes=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef))]
    states=NS(off=0,stopping=1,starting=2,pid=3)
    cushion_spec=importlib.util.spec_from_file_location('cushion_fixture',Path(__file__).parents[1]/'lib/final_stop_cushion.py')
    cushion_module=importlib.util.module_from_spec(cushion_spec);cushion_spec.loader.exec_module(cushion_module)
    env=dict(np=np,DT_CTRL=.01,LongCtrlState=states,FinalStopCushion=cushion_module.FinalStopCushion)
    exec(compile(ast.Module(body=nodes,type_ignores=[]),'actual_longcontrol','exec'),env)
    obj=env['LongControl'].__new__(env['LongControl'])
    obj.CP=NS(brand='hyundai',vEgoStarting=.5,startingState=False,stopAccel=-.5,
              longitudinalTuning=NS(kpBP=[0],kiBP=[0]),stoppingDecelRate=.8)
    obj.long_control_state=states.pid;obj.last_output_accel=-1.2;obj.terminal_stop_release=self.h
    obj.final_stop_cushion=cushion_module.FinalStopCushion()
    obj.readParamCount=0;obj.stopping_accel=-.5;obj.use_accel_pid=False
    obj.pid=NS(update=lambda *args,**kwargs:-1.2,reset=lambda:None)
    cs=NS(standstill=False,softHoldActive=0,vEgo=.5,aEgo=-.8,brakePressed=False,gasPressed=False,
          steeringPressed=False,leftBlinker=False,rightBlinker=False,cruiseState=NS(standstill=False))
    plan=NS(aTarget=-1.2,vTargetNow=0.,jTargetNow=0.,shouldStop=False,
            fcw=False,xState=0,longitudinalPlanSource='lead0')
    radar=NS(leadOne=self.l,leadTwo=lead(present=False))
    self.arm();self.t+=.05
    output,_,_=obj.update(True,cs,plan,(-3.5,2.),0.,radar,
                         radar_stamp=self.t,observation_time=self.t,terminal_stop_ready=True)
    self.assertAlmostEqual(output,-1.185)
    obj.long_control_state=states.stopping;plan.shouldStop=True
    self.t+=.05
    output,_,_=obj.update(True,cs,plan,(-3.5,2.),0.,radar,
                         radar_stamp=self.t,observation_time=self.t,terminal_stop_ready=True)
    self.assertAlmostEqual(output,-1.185)

    # Exercise the actual new taper after entering stopping at the final crawl.
    obj.CP.stopAccel=obj.stopping_accel=-2.
    cs.vEgo=.25;self.l.vRel=-.25;self.l.dRel=2.4
    obj.long_control_state=states.pid;plan.shouldStop=False
    for _ in range(16):
      self.t+=.05
      obj.update(True,cs,plan,(-3.5,2.),0.,radar,radar_stamp=self.t,
                 observation_time=self.t,terminal_stop_ready=True)
    previous=float(obj.last_output_accel)
    plan.shouldStop=True;self.t+=.05
    output,_,_=obj.update(True,cs,plan,(-3.5,2.),0.,radar,radar_stamp=self.t,
                         observation_time=self.t,terminal_stop_ready=True)
    self.assertAlmostEqual(output,previous+.006)
    # At standstill the original .8 m/s^3 hold build resumes; soft hold stays native.
    cs.standstill=True;cs.vEgo=0.;self.l.vRel=0.;self.t+=.05
    previous=float(output)
    output,_,_=obj.update(True,cs,plan,(-3.5,2.),0.,radar,radar_stamp=self.t,
                         observation_time=self.t,terminal_stop_ready=True)
    self.assertAlmostEqual(output,previous-.008)
    cs.softHoldActive=1;self.t+=.05
    output,_,_=obj.update(True,cs,plan,(-3.5,2.),0.,radar,radar_stamp=self.t,
                         observation_time=self.t,terminal_stop_ready=True)
    self.assertEqual(output,-2.)

  def test_actual_controller_gate_keeps_hold_events_and_pedals(self):
    import ast
    from pathlib import Path
    tree=ast.parse((Path(__file__).parents[1]/'lib/longcontrol.py').read_text(encoding='utf8'))
    call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and
              isinstance(n.func.value,ast.Attribute) and n.func.value.attr=='terminal_stop_release')
    expression=next(k.value for k in call.keywords if k.arg=='enabled')
    code=compile(ast.Expression(body=expression),'actual_terminal_stop_gate','eval')
    controller=NS(CP=NS(brand='hyundai'),long_control_state='pid')
    plan=NS(fcw=False,xState=0,longitudinalPlanSource='lead0')
    cs=NS(gasPressed=False,brakePressed=False,steeringPressed=False,leftBlinker=False,rightBlinker=False)
    env=dict(self=controller,CS=cs,long_plan=plan,LongCtrlState=NS(pid='pid'),
             terminal_stop_ready=True,active=True,soft_hold_active=False)
    self.assertTrue(eval(code,env))
    for obj,key,value in [(controller.CP,'brand','toyota'),(controller,'long_control_state','stopping'),
                          (plan,'fcw',True),(plan,'xState',1),(plan,'longitudinalPlanSource','cruise'),
                          (cs,'gasPressed',True),(cs,'brakePressed',True),(cs,'steeringPressed',True),
                          (cs,'leftBlinker',True),(cs,'rightBlinker',True)]:
      old=getattr(obj,key);setattr(obj,key,value)
      self.assertFalse(eval(code,env),key)
      setattr(obj,key,old)
    for key,value in (('terminal_stop_ready',False),('active',False),('soft_hold_active',True)):
      old=env[key];env[key]=value;self.assertFalse(eval(code,env),key);env[key]=old


if __name__=='__main__':unittest.main()
