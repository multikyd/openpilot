import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

from openpilot.selfdrive.kisapilot.turn_progress import TurnProgress
from openpilot.selfdrive.controls.lib.turn_exit_recovery import TurnExitRecovery
from openpilot.selfdrive.modeld.constants import ModelConstants


class TestTurnProgress(unittest.TestCase):
  def setUp(self):
    self.h=TurnProgress()
    self.now=0.

  def tick(self, count=1, **kw):
    out=[]
    for _ in range(count):
      self.now+=.05
      args=dict(now=self.now, enabled=True, code=2, distance=0., speed=5., angle=-120.,
                curvature=.04, preview=.04, far_preview=.02)
      args.update(kw)
      out.append(self.h.update(**args))
    return out

  def test_45_means_heading_change_not_steering_wheel_angle(self):
    self.assertEqual(self.tick(60, angle=120.), [0.]*60)
    self.assertLess(math.degrees(self.h.heading), 45.)
    values=self.tick(30, angle=120.)
    self.assertGreater(values[-1],0.)
    self.assertGreater(math.degrees(self.h.heading),45.)
    self.assertLessEqual(max(np.diff(values)), .35*.05+1e-9)

  def test_left_and_right_turns_match(self):
    a=self.tick(100)
    self.setUp()
    b=self.tick(100, code=1, angle=120.,curvature=-.04,preview=-.04,far_preview=-.02)
    np.testing.assert_allclose(a,b)

  def test_no_navigation_uturn_and_fork_do_not_release(self):
    for code in (0,3,4,5):
      self.setUp()
      self.assertEqual(self.tick(140,code=code),[0.]*140)

  def test_strong_or_opposite_next_curve_prevents_release(self):
    for args in (dict(far_preview=.08),dict(preview=.08),dict(far_preview=-.02)):
      self.setUp()
      self.assertEqual(self.tick(120,**args),[0.]*120)

  def test_new_same_direction_turn_cannot_inherit_heading(self):
    self.tick(100)
    self.assertGreater(self.h.extra_speed,0.)
    self.assertEqual(self.tick(distance=35.),[0.])
    self.assertEqual(self.h.heading,0.)

  def test_stale_disabled_gap_or_bad_sample_reset(self):
    for kind in ('off','nan','gap'):
      self.setUp();self.tick(100)
      if kind=='gap':self.now+=1.
      args={'enabled':False} if kind=='off' else {'curvature':float('nan')} if kind=='nan' else {}
      self.assertEqual(self.tick(**args),[0.])
      self.assertEqual(self.h.extra_speed,0.)

  def test_mild_turn_without_tight_steering_does_not_arm(self):
    self.assertEqual(self.tick(120,angle=30.),[0.]*120)

  def test_heading_reversal_does_not_count_toward_same_turn(self):
    self.tick(60)
    self.tick(5,curvature=-.04)
    self.assertEqual(self.h.extra_speed,0.)
    self.assertLess(self.h.heading, math.radians(10.))

  def test_mid_turn_accel_does_not_wait_for_straight_wheel(self):
    h=TurnExitRecovery();values=[]
    for i in range(110):
      values.append(h.update(now=i*.05,enabled=True,v_ego=5.,steering_angle=-120.,
          curvature=.04,preview=.04,far_preview=.02,normal_max=.8,target_speed=10.,leads=(),
          turn_code=2,turn_distance=0.,nav_fresh=True))
    self.assertEqual(values[:70],[.8]*70)
    self.assertGreater(values[-1],.8)
    self.assertLessEqual(max(values),1.)

  def test_lead_veto_keeps_normal_accel_during_mid_turn(self):
    h=TurnExitRecovery()
    lead=NS(present=True,dRel=10.,vRel=0.,aLeadK=0.)
    for i in range(110):
      out=h.update(now=i*.05,enabled=True,v_ego=5.,steering_angle=-120.,
          curvature=.04,preview=.04,far_preview=.02,normal_max=.8,target_speed=10.,leads=(lead,),
          turn_code=2,turn_distance=0.,nav_fresh=True)
      self.assertEqual(out,.8)
      self.assertEqual(h.strength,0.)


class TestTbtProgressIntegration(unittest.TestCase):
  def setUp(self):
    path=Path(__file__).parents[2]/'kisapilot/nav_guidance.py'
    tree=ast.parse(path.read_text(encoding='utf8'))
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_turn_progress_speed')
    env=dict(np=np,math=math,ModelConstants=ModelConstants)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),env)
    self.fn=env['_turn_progress_speed']
    self.h=NS(turn_progress=TurnProgress(),navi_highway_stamp=None,autoTurnControl=2,
              tbt_blinker_stamp=0.,xTurnInfo=2,xDistToTurn=0.,debugText='')
    class SM(dict):pass
    self.sm=SM(carState=NS(vEgo=5.,steeringAngleDeg=-120.,gasPressed=False,brakePressed=False),
               controlsState=NS(curvature=.04),carControl=NS(longActive=True),
               modelV2=NS(orientationRate=NS(z=[.1]*33),velocity=NS(x=[5.]*33)))
    self.sm.valid={key:True for key in self.sm}
    self.now=0.

  def tick(self,count=1):
    out=[]
    for _ in range(count):
      self.now+=.05
      self.h.tbt_blinker_stamp=self.now
      self.sm.recv_time={key:self.now for key in self.sm}
      out.append(self.fn(self.h,self.sm,self.now,20.))
    return out

  def test_tbt_speed_cap_itself_relaxes_before_turn_finishes(self):
    self.assertEqual(self.tick(60),[20.]*60)
    self.assertGreater(self.tick(50)[-1],20.)
    self.assertEqual(abs(self.sm['carState'].steeringAngleDeg),120.)

  def test_pedal_and_highway_prevent_tbt_release(self):
    for case in ('pedal','highway'):
      self.setUp()
      if case=='pedal':self.sm['carState'].brakePressed=True
      else:self.h.navi_highway_stamp=1000.
      if case=='highway':self.now=999.95
      self.assertEqual(self.tick(20 if case=='highway' else 110),[20.]*(20 if case=='highway' else 110))

  def test_invalid_model_message_removes_tbt_allowance(self):
    self.assertGreater(self.tick(110)[-1],20.)
    self.sm.valid['modelV2']=False
    self.assertEqual(self.tick(),[20.])
    self.assertEqual(self.h.turn_progress.extra_speed,0.)

  def test_other_speed_constraints_remain_in_actual_selection(self):
    relaxed=self.tick(110)[-1]
    path=Path(__file__).parents[2]/'kisapilot/nav_guidance.py'
    tree=ast.parse(path.read_text(encoding='utf8'))
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='update_navi')
    sources=next(n for n in method.body if isinstance(n,ast.Assign) and
                 isinstance(n.targets[0],ast.Name) and n.targets[0].id=='speed_n_sources')
    selection=next(n for n in method.body if isinstance(n,ast.Assign) and
                   isinstance(n.targets[0],ast.Tuple) and
                   all(isinstance(v,ast.Name) for v in n.targets[0].elts) and
                   [v.id for v in n.targets[0].elts]==['desired_speed','source'])
    code=compile(ast.Module(body=[sources,selection],type_ignores=[]),'actual_nav_speed_selection','exec')
    for key in ('atc_desired_next','sdi_speed','limit_speed'):
      env=dict(atc_desired=relaxed,atc_desired_next=250.,sdi_speed=250.,limit_speed=250.,sdi_source='cam')
      env[key]=15.
      exec(code,env)
      self.assertEqual(env['desired_speed'],15.)


if __name__=='__main__':
  unittest.main()
