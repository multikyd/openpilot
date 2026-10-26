import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

spec = importlib.util.spec_from_file_location('turn_exit_helper', Path(__file__).parents[1]/'lib/turn_exit_recovery.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


def lead(**kwargs):
  values = dict(present=False, dRel=50., vRel=2., aLeadK=0., radar=False)
  values.update(kwargs)
  return NS(**values)


class TestTurnExitRecovery(unittest.TestCase):
  def setUp(self):
    self.guard = helper.TurnExitRecovery()
    self.time = 0.

  def tick(self, count=1, **kwargs):
    values = dict(enabled=True, v_ego=6., steering_angle=-120., curvature=.04,
                  preview=.05, far_preview=.02, normal_max=.8, target_speed=15., leads=(lead(), lead()))
    values.update(kwargs)
    outputs = []
    for _ in range(count):
      self.time += .05
      outputs.append(self.guard.update(now=self.time, **values))
    return outputs

  def arm(self, side=-1):
    self.tick(15, steering_angle=120.*side, curvature=-side*.04, preview=-side*.05)

  def exit(self, count=30, **kwargs):
    values = dict(steering_angle=-15., curvature=.004, preview=.004, far_preview=.003)
    values.update(kwargs)
    return self.tick(count, **values)

  def test_straight_driving_does_not_raise_configured_limit(self):
    self.assertEqual(self.exit(200), [.8]*200)

  def test_both_turn_directions_recover_after_confirmation(self):
    for side in (-1, 1):
      self.setUp(); self.arm(side)
      values = self.exit(25, steering_angle=15.*side, curvature=-side*.004, preview=-side*.004)
      self.assertEqual(values[:5], [.8]*5)
      self.assertGreater(values[-1], 1.2)
      self.assertLessEqual(max(values), 1.25+1e-9)
      self.assertLessEqual(max(b-a for a,b in zip(values,values[1:])), .45*.05/.6+1e-9)

  def test_brief_turn_does_not_arm(self):
    self.tick(4)
    self.assertEqual(self.exit(), [.8]*30)

  def test_steering_or_curvature_alone_does_not_arm(self):
    for kwargs in (dict(preview=.001), dict(steering_angle=-10.)):
      self.setUp(); self.tick(30, **kwargs)
      self.assertEqual(self.exit(), [.8]*30)

  def test_unwinding_alone_does_not_override_upcoming_bend(self):
    self.arm()
    self.assertEqual(self.exit(far_preview=.06), [.8]*30)
    self.assertEqual(self.exit(preview=.04), [.8]*30)

  def test_remaining_large_steering_blocks_recovery(self):
    self.arm()
    self.assertEqual(self.exit(steering_angle=-70.), [.8]*30)

  def test_close_closing_braking_or_invalid_lead_vetoes_in_either_slot(self):
    for obstacle in (lead(present=True,dRel=15.),lead(present=True,vRel=-1.),
                     lead(present=True,aLeadK=-1.),lead(present=True,dRel=float('nan'))):
      for index in (0, 1):
        self.setUp(); self.arm(); self.exit()
        leads = [lead(),lead()]; leads[index] = obstacle
        self.assertEqual(self.exit(1,leads=leads), [.8])
        self.assertEqual(self.guard.strength,0.)

  def test_far_receding_vision_lead_is_allowed(self):
    self.arm()
    self.assertGreater(self.exit(leads=(lead(present=True),lead()))[-1],.8)

  def test_disable_or_stale_update_clears_recovery(self):
    self.arm(); self.exit()
    self.assertEqual(self.exit(1,enabled=False),[.8])
    self.assertEqual(self.exit(),[.8]*30)
    self.arm(); self.time += 1.
    self.assertEqual(self.exit(),[.8]*30)

  def test_nonfinite_preview_or_missing_headroom_cancels(self):
    for kwargs in (dict(preview=float('nan')),dict(far_preview=float('inf')),dict(target_speed=6.5)):
      self.setUp(); self.arm(); self.exit()
      self.assertEqual(self.exit(1,**kwargs),[.8])
      self.assertEqual(self.guard.strength,0.)

  def test_new_bend_cancels_active_recovery(self):
    self.arm(); self.exit()
    self.assertEqual(self.exit(1,steering_angle=50.),[.8])
    self.assertEqual(self.guard.strength,0.)

  def test_window_expires_and_cannot_retrigger_on_straight(self):
    self.arm(); values=self.exit(180)
    self.assertGreater(max(values),.8)
    self.assertEqual(values[-20:],[.8]*20)

  def test_existing_higher_limit_is_preserved(self):
    self.arm()
    self.assertEqual(self.exit(normal_max=2.),[2.]*30)

  def test_deceleration_limit_is_never_raised(self):
    self.arm()
    self.assertEqual(self.exit(normal_max=-.2),[-.2]*30)


  def test_uncertain_preview_pauses_without_extending_window(self):
    self.arm(); self.exit()
    until = self.guard.until
    self.assertEqual(self.exit(10, far_preview=.06), [.8]*10)
    self.assertEqual(self.guard.strength,0.)
    self.assertEqual(self.guard.until,until)
    values = self.exit(30)
    self.assertEqual(values[:5],[.8]*5)
    self.assertGreater(values[-1],.8)
    self.assertEqual(self.guard.until,until)

  def test_close_lead_during_turn_does_not_erase_turn_history(self):
    self.arm()
    self.assertEqual(self.exit(10,leads=(lead(present=True,dRel=15.),lead())),[.8]*10)
    self.assertGreater(self.exit()[-1],.8)


class TestPlannerRecoveryIntegration(unittest.TestCase):
  def setUp(self):
    import ast
    from unittest.mock import MagicMock
    root = Path(__file__).parents[1]/'lib'
    tree = ast.parse((root/'longitudinal_planner.py').read_text(encoding='utf8'))
    cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LongitudinalPlanner')
    update = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update')
    start = next(i for i,n in enumerate(update.body) if isinstance(n,ast.Assign) and
                 isinstance(n.targets[0],ast.Name) and n.targets[0].id=='cs')
    end = next(i for i,n in enumerate(update.body[start:],start) if isinstance(n,ast.If) and
               isinstance(n.test,ast.Name) and n.test.id=='reset_state')
    self.block = compile(ast.Module(body=update.body[start:end],type_ignores=[]),'actual_planner_recovery','exec')
    class SM(dict):
      def all_checks(self,*args,**kwargs): return self.valid_checks
    sm = self.sm = SM(carState=NS(gasPressed=False,brakePressed=False,steeringPressed=True,steeringAngleDeg=15.),
                      controlsState=NS(curvature=.004,desiredCurvature=.004),selfdriveState=NS(enabled=True),
                      radarState=NS(leadOne=lead(),leadTwo=lead()),
                      modelV2=NS(orientationRate=NS(z=[0.]*33),velocity=NS(x=[0.]*33)))
    sm.valid_checks = True;sm.logMonoTime={n:10_000_000_000 for n in ('carState','radarState','modelV2')}
    self.carrot = NS(desireState=0.,myDrivingMode=NS(value=3),xState=0,
                     signal_follow=NS(strength=0.),signal_stop_approach=NS(accel_cap=None),
                     get_carrot_accel=lambda speed:1.)
    recovery = MagicMock();recovery.update.return_value=1.5;recovery.strength=.8
    self.planner = NS(CP=NS(openpilotLongitudinalControl=True),mpc=NS(mode='acc'),
                      turn_exit_recovery=recovery,_turn_exit_active=False,output_should_stop=False)
    self.env = dict(self=self.planner,sm=sm,carrot=self.carrot,reset_state=False,force_slow_decel=False,
                    v_ego=6.,v_cruise=15.,ModelConstants=NS(IDX_N=33),
                    XState=NS(e2eStop=1,e2eStopped=2,e2ePrepare=3),cloudlog=MagicMock(),
                    get_future_curvature=lambda *args,**kwargs:.004,A_CRUISE_MIN=-2.,ACCEL_MIN=-3.5,ACCEL_MAX=2.,
                    limit_accel_in_turns=lambda speed,curve,limits,lat:[limits[0],min(.9,limits[1])])

  def enabled(self):
    exec(self.block,self.env)
    return self.planner.turn_exit_recovery.update.call_args.kwargs['enabled']

  def test_lateral_acceleration_cap_still_follows_recovery(self):
    self.assertTrue(self.enabled())
    self.assertEqual(self.env['accel_limits_turns'],[-2.,.9])

  def test_signal_stop_pedal_and_eco_gates(self):
    cases = [(self.carrot.signal_follow,'strength',.2),
             (self.carrot.signal_stop_approach,'accel_cap',.3),
             (self.sm['carState'],'gasPressed',True),(self.sm['carState'],'brakePressed',True),
             (self.carrot.myDrivingMode,'value',1),(self.carrot.myDrivingMode,'value',2),
             (self.carrot,'xState',1),(self.carrot,'desireState',1.),
             (self.planner,'output_should_stop',True),
             (self.sm['selfdriveState'],'enabled',False)]
    for obj,key,value in cases:
      old=getattr(obj,key);setattr(obj,key,value)
      self.assertFalse(self.enabled(),key)
      setattr(obj,key,old)

  def test_invalid_checks_missing_preview_reset_and_force_decel_gate(self):
    self.sm.valid_checks=False;self.assertFalse(self.enabled());self.sm.valid_checks=True
    self.sm['modelV2'].velocity.x=[];self.assertFalse(self.enabled());self.sm['modelV2'].velocity.x=[0.]*33
    self.env['reset_state']=True;self.assertFalse(self.enabled());self.env['reset_state']=False
    self.env['force_slow_decel']=True;self.assertFalse(self.enabled());self.env['force_slow_decel']=False
    self.sm.logMonoTime['radarState']-=1_000_000_000;self.assertFalse(self.enabled())

  def test_blended_mode_does_not_enable_recovery(self):
    self.planner.mpc.mode='blended'
    self.assertFalse(self.enabled())

  def test_mpc_comfort_change_is_disabled_when_a_safety_cap_binds(self):
    import ast
    import numpy as np
    from unittest.mock import MagicMock
    tree=ast.parse((Path(__file__).parents[1]/'lib/longitudinal_mpc_lib/long_mpc.py').read_text(encoding='utf8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LongitudinalMpc')
    update=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update')
    start=next(i for i,n in enumerate(update.body) if isinstance(n,ast.Assign) and
               isinstance(n.targets[0],ast.Name) and n.targets[0].id=='exit_strength')
    block=compile(ast.Module(body=update.body[start:start+3],type_ignores=[]),'actual_mpc_cost_recovery','exec')
    mpc=NS(max_a=1.5,turn_loss_limit=None,set_weights=MagicMock())
    env=dict(np=np,self=mpc,carrot=NS(jerk_factor_apply=.7,aChangeCostStarting=10.),
             turn_exit_strength=1.,mode='acc',reset_state=False,cruise_max_a=1.5,
             prev_accel_constraint=True,personality=0)
    exec(block,env)
    self.assertAlmostEqual(mpc.set_weights.call_args.kwargs['jerk_factor'],.455)
    for cap,loss in ((.3,None),(1.5,.5)):
      env['cruise_max_a']=cap;mpc.turn_loss_limit=loss
      exec(block,env)
      self.assertEqual(mpc.set_weights.call_args.kwargs['jerk_factor'],.7)


if __name__ == '__main__':
  unittest.main()
