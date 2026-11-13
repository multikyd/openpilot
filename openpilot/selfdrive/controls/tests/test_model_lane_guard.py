import copy
import math
import unittest
from types import SimpleNamespace as NS

import numpy as np

from openpilot.selfdrive.controls.lib.model_lane_guard import ModelLaneGuard, LanePositionGuard


def fixture(side=1, offset=.65):
  speed, curvature = 17., side*.008
  x = np.linspace(0., 100., 33)
  center = .5*curvature*x*x-side*offset
  lines = [NS(x=x, y=center+v) for v in (-5.25, -1.75, 1.75, 5.25)]
  model = NS(laneLines=lines, laneLineProbs=[.8, .95, .95, .8], laneLineStds=[.1]*4,
             position=NS(x=x, y=.5*curvature*x*x), meta=NS(laneChangeState='off', desire='none'))
  cs = NS(vEgo=speed, steeringPressed=False, leftBlinker=False, rightBlinker=False)
  return model, cs, curvature


class TestModelLaneGuard(unittest.TestCase):
  def settle(self, guard, model, cs, curvature, steps=30, start=100., **kwargs):
    result = []
    for i in range(steps):
      stamp = start+i*.05
      result.append(guard.update(model, cs, curvature, stamp, stamp+.02, camera_offset=0., **kwargs))
    return result

  def test_mirrored_curves_correct_only_outward(self):
    outputs = []
    for side in (-1, 1):
      model, cs, k = fixture(side);g=ModelLaneGuard()
      values=self.settle(g, model, cs, k)
      self.assertLess(abs(values[-1]), abs(k))
      self.assertTrue(all(.75*abs(k)-1e-12 <= abs(v) <= abs(k)+1e-12 and v*k>0 for v in values))
      self.assertLessEqual(abs(g.accel), .45)
      outputs.append(values[-1])
    self.assertAlmostEqual(outputs[0], -outputs[1])

  def test_centered_curve_remains_model_command(self):
    for side in (-1, 1):
      model, cs, k=fixture(side, offset=0.)
      self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))

  def test_outside_offset_does_not_reduce_needed_turn(self):
    model,cs,k=fixture(offset=-.8)
    self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))

  def test_model_prediction_close_to_boundary_can_trigger_before_current_offset(self):
    model,cs,k=fixture(offset=.1)
    model.position.y += .002*model.position.x**2
    g=ModelLaneGuard();self.settle(g,model,cs,k)
    self.assertLess(g.accel,0)

  def test_unique_frames_required_for_confirmation(self):
    model,cs,k=fixture();g=ModelLaneGuard()
    for i in range(15):
      self.assertEqual(g.update(model,cs,k,100.,100.+i*.01,camera_offset=0),k)
    self.assertLess(g.evidence,.2)
    self.assertEqual(g.accel,0)

  def test_correction_jerk_bound_on_valid_frames(self):
    model,cs,k=fixture();g=ModelLaneGuard();values=self.settle(g,model,cs,k)
    accel=(np.array([k]+values)-k)*cs.vEgo**2
    self.assertLessEqual(max(abs(np.diff(accel))),.05+1e-12)

  def test_missing_or_stale_data_removes_correction(self):
    for age in (.26, -.02):
      model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
      self.assertEqual(g.update(model,cs,k,102.,102.+age),k)
      self.assertEqual(g.accel,0)

  def test_driver_input_blinkers_and_disabled_remove_correction(self):
    for flag in ('steeringPressed','leftBlinker','rightBlinker'):
      model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k);setattr(cs,flag,True)
      self.assertEqual(g.update(model,cs,k,102.,102.02),k)
      self.assertEqual(g.accel,0)
    model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
    self.assertEqual(g.update(model,cs,k,102.,102.02,enabled=False),k)

  def test_lane_change_and_turn_desire_bypass(self):
    for field,value in [('laneChangeState','laneChangeStarting'),('desire','laneChangeLeft'),('desire','turnRight')]:
      model,cs,k=fixture();setattr(model.meta,field,value)
      self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))

  def test_weak_inside_line_bypasses_but_weak_outside_line_does_not(self):
    model,cs,k=fixture();model.laneLineProbs[2]=.5
    self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))
    model,cs,k=fixture();model.laneLineStds[2]=.31
    self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))
    model,cs,k=fixture();model.laneLineStds[1]=1.;model.laneLineProbs[1]=.1
    self.assertLess(self.settle(ModelLaneGuard(),model,cs,k)[-1],k)

  def test_confidence_reduces_correction(self):
    model,cs,k=fixture();a=self.settle(ModelLaneGuard(),model,cs,k)[-1]
    model.laneLineStds[2]=.25;b=self.settle(ModelLaneGuard(),model,cs,k)[-1]
    self.assertLess(abs(b-k),abs(a-k))

  def test_invalid_corridors_and_paths_bypass(self):
    for case in ('narrow','wide','diverging','crossed','nan','short','duplicate_x','behind'):
      model,cs,k=fixture()
      if case=='narrow':model.laneLines[2].y=model.laneLines[1].y+2.5
      elif case=='wide':model.laneLines[2].y=model.laneLines[1].y+5.
      elif case=='diverging':model.laneLines[2].y+=.08*model.position.x
      elif case=='crossed':model.laneLines[2].y=model.laneLines[1].y-1.
      elif case=='nan':model.laneLines[2].y[3]=math.nan
      elif case=='short':model.position.x=model.position.x[:5];model.position.y=model.position.y[:5]
      elif case=='duplicate_x':model.position.x[1]=model.position.x[0]
      elif case=='behind':model.position.x=model.position.x+2
      self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)),case)

  def test_s_curve_does_not_carry_old_boundary_correction(self):
    model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
    self.assertLess(g.accel,0)
    model2,cs2,k2=fixture(-1)
    result=g.update(model2,cs2,k2,101.5,101.52,camera_offset=0.)
    self.assertEqual(result,k2)
    self.assertEqual(g.accel,0)
    self.assertGreater(self.settle(g,model2,cs2,k2,start=101.55)[-1],k2)

  def test_straight_exit_resets_boundary_state(self):
    model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
    self.assertEqual(g.update(model,cs,.0005,102.,102.02),.0005)
    self.assertEqual(g.accel,0)

  def test_boundary_identity_jump_requires_new_evidence(self):
    model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
    for line in model.laneLines:line.y-=.6
    self.assertEqual(g.update(model,cs,k,101.5,101.52,camera_offset=0),k)
    self.assertEqual(g.accel,0)

  def test_speed_range_and_nonfinite_scalars(self):
    for speed in (0.,7.,36.,math.nan):
      model,cs,k=fixture();cs.vEgo=speed
      self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))
    model,cs,k=fixture()
    self.assertEqual(ModelLaneGuard().update(model,cs,k,100.,100.02,camera_offset=math.nan),k)

  def test_speed_change_does_not_exceed_fractional_curvature_bound(self):
    model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
    cs.vEgo=8.
    result=g.update(model,cs,.0013,101.45,101.49,camera_offset=0.)
    self.assertGreaterEqual(result,.0013*.75-1e-12)

  def test_invalid_frame_gap_restarts_confirmation(self):
    model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
    self.assertEqual(g.update(model,cs,k,105.,105.02,camera_offset=0),k)
    self.assertEqual(g.accel,0)


  def test_anticipates_before_old_margin_is_reached(self):
    model,cs,k=fixture(offset=.35)  # 1.40m margin: outside the old 1.32m trigger.
    g=ModelLaneGuard();values=self.settle(g,model,cs,k)
    self.assertLess(values[-1],k)
    self.assertGreater(g.penetration,0.)

  def test_far_ramp_fold_does_not_disable_local_prediction(self):
    model,cs,k=fixture()
    model.position.x[-3:]=[90.,88.,85.]
    g=ModelLaneGuard();self.settle(g,model,cs,k)
    self.assertLess(g.accel,0.)

  def test_short_local_coverage_does_not_start_correction(self):
    model,cs,k=fixture();model.position.x=np.linspace(0.,5.,33)
    self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))

  def test_prediction_noise_beyond_local_horizon_is_ignored(self):
    model,cs,k=fixture();model.position.y[-1]=math.nan
    self.assertLess(self.settle(ModelLaneGuard(),model,cs,k)[-1],k)

  def test_weak_line_releases_gradually_without_adding_correction(self):
    model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
    previous=g.accel
    model.laneLineProbs[2]=.4
    result=g.update(model,cs,k,101.5,101.52,camera_offset=0.)
    self.assertGreater(result,k+previous/cs.vEgo**2)
    self.assertLess(result,k)
    self.assertLessEqual(abs(g.accel-previous),.30*.05+1e-12)
    values=self.settle(g,model,cs,k,steps=160,start=101.55)
    self.assertAlmostEqual(values[-1],k,places=7)

  def test_observation_noise_has_bounded_slow_correction(self):
    model,cs,k=fixture(offset=.4);g=ModelLaneGuard();values=[0.]
    for i in range(120):
      model.laneLineProbs[2]=.64 if i%2 else .95
      stamp=100+i*.05
      g.update(model,cs,k,stamp,stamp+.02,camera_offset=0.)
      values.append(g.accel)
    self.assertLess(values[-1],0.)
    self.assertLessEqual(max(abs(np.diff(values))),.40*.05+1e-12)
    self.assertTrue(all(a<=0 for a in values))

  def test_smoothed_release_when_width_temporarily_uncertain(self):
    model,cs,k=fixture();g=ModelLaneGuard();self.settle(g,model,cs,k)
    previous=g.accel
    model.laneLines[1].y-=1.2  # Width unreliable, inside line identity unchanged.
    result=g.update(model,cs,k,101.5,101.52,camera_offset=0.)
    self.assertLess(result,k)
    self.assertLessEqual(abs(g.accel-previous),.30*.05+1e-12)

  def test_centered_narrow_corridor_does_not_get_preference_bias(self):
    model,cs,k=fixture(offset=0.)
    center=model.position.y
    model.laneLines[1].y=center-1.425;model.laneLines[2].y=center+1.425
    self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))

  def test_gentle_curve_keeps_original_margin(self):
    model,cs,k=fixture(offset=.30)
    k=.0025
    self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))

  def test_recent_painted_boundary_supports_a_narrow_corridor_in_both_directions(self):
    for side in (-1, 1):
      model,cs,k=fixture(side,offset=.4)
      center=model.position.y-side*.4
      model.laneLines[1].y=center-1.275;model.laneLines[2].y=center+1.275
      setattr(cs,'rightLaneLine' if side>0 else 'leftLaneLine',11)
      g=ModelLaneGuard();values=self.settle(g,model,cs,k)
      self.assertLess(abs(values[-1]),abs(k))
      model.laneLineProbs[2 if side>0 else 1]=.48
      self.settle(g,model,cs,k,start=101.55,steps=8)
      self.assertGreater(g.confidence,0.)
      self.assertGreater(g.evidence,0.)
      self.assertLess(g.accel*side,0.)

  def test_recent_painted_boundary_can_bridge_moderate_std_increase(self):
    model,cs,k=fixture();cs.rightLaneLine=11;g=ModelLaneGuard()
    self.settle(g,model,cs,k)
    model.laneLineProbs[2]=.6;model.laneLineStds[2]=.42
    values=self.settle(g,model,cs,k,start=101.5,steps=10)
    self.assertGreater(g.evidence,0.)
    self.assertLess(values[-1],k)
    fresh=ModelLaneGuard()
    self.assertTrue(all(v==k for v in self.settle(fresh,model,cs,k,steps=10)))

  def test_weak_painted_line_cannot_acquire_without_recent_reliable_geometry(self):
    model,cs,k=fixture();cs.rightLaneLine=11;model.laneLineProbs[2]=.48
    self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))

  def test_painted_line_memory_expires_without_reliable_observations(self):
    model,cs,k=fixture();cs.rightLaneLine=11;g=ModelLaneGuard();self.settle(g,model,cs,k)
    model.laneLineProbs[2]=.48
    self.settle(g,model,cs,k,start=101.5,steps=30)
    self.assertEqual(g.confidence,0.)
    self.assertEqual(g.evidence,0.)

  def test_centered_narrow_painted_corridor_keeps_original_turn(self):
    model,cs,k=fixture(offset=0.);cs.rightLaneLine=11
    model.laneLines[1].y=model.position.y-1.275;model.laneLines[2].y=model.position.y+1.275
    self.assertTrue(all(v==k for v in self.settle(ModelLaneGuard(),model,cs,k)))


class TestLanePositionGuard(unittest.TestCase):
  def setUp(self):
    self.md,self.cs,self.k=fixture(offset=.35)
    self.md.meta.laneWidthLeft=1.
    self.md.meta.laneWidthRight=3.2
    self.md.laneLineProbs[0]=.05
    self.h=LanePositionGuard()
    self.t=10.

  def tick(self,count=1,**kw):
    out=[]
    for _ in range(count):
      self.t+=.05
      args=dict(model=self.md,cs=self.cs,curvature=self.k,guarded=self.k,
                stamp=self.t,now=self.t,preference=-.07,camera_offset=0.,actuator_delay=.4,enabled=True)
      args.update(kw)
      out.append(self.h.update(**args))
    return out

  def test_recovers_left_in_both_turn_directions_with_bounded_command(self):
    for k in (.008,-.008):
      self.setUp();self.k=k
      outputs=self.tick(70)
      self.assertLess(outputs[-1],k)
      self.assertLessEqual(abs((outputs[-1]-k)*self.cs.vEgo**2),.300001)
      self.assertTrue(all(v*k>0 for v in outputs))
      self.assertLessEqual(max(abs(b-a)*self.cs.vEgo**2 for a,b in zip(outputs,outputs[1:])),.35*.05+1e-8)

  def test_centered_car_at_left_preference_needs_no_extra_command(self):
    self.setUp()
    for line,offset in zip(self.md.laneLines,(-5.25,-1.75,1.75,5.25)):
      line.y=self.md.position.y+offset+.07
    self.assertTrue(all(v==self.k for v in self.tick(70)))

  def test_reliable_left_can_continue_when_other_boundary_weakens(self):
    self.tick(35);self.md.laneLineProbs[2]=.25;self.md.laneLineStds[2]=.5
    self.assertLess(self.tick(20)[-1],self.k)

  def test_right_reference_needs_recent_confirmed_lane_width(self):
    self.md.laneLineProbs[1]=.2
    self.assertTrue(all(v==self.k for v in self.tick(40)))
    self.setUp();self.tick(30);self.md.laneLineProbs[1]=.2
    self.assertLess(self.tick(15)[-1],self.k)
    self.tick(180)
    self.assertAlmostEqual(self.h.accel,0.,places=5)

  def test_weak_data_cannot_build_and_extended_loss_releases(self):
    self.tick(25);before=self.h.accel
    self.md.laneLineProbs[1]=self.md.laneLineProbs[2]=.2
    self.tick(5)
    self.assertGreaterEqual(self.h.accel,before-1e-9)
    self.tick(160)
    self.assertAlmostEqual(self.h.accel,0.,places=6)

  def test_blinkers_driver_override_stale_model_and_disable_clear(self):
    for key in ('leftBlinker','rightBlinker','steeringPressed'):
      self.setUp();self.tick(30);setattr(self.cs,key,True)
      self.assertEqual(self.tick(),[self.k])
    self.setUp();self.tick(30)
    self.assertEqual(self.tick(now=self.t+1.),[self.k])
    self.setUp();self.tick(30);self.assertEqual(self.tick(enabled=False),[self.k])

  def test_clear_rightmost_context_retains_existing_profile(self):
    self.tick(30)
    self.md.meta.laneWidthRight=.8;self.md.meta.laneWidthLeft=3.2
    self.md.laneLineProbs[0]=.8;self.md.laneLineProbs[3]=.05
    self.assertEqual(self.tick(),[self.k])

  def test_single_lane_with_yellow_centerline_retains_opposing_traffic_margin(self):
    self.tick(30)
    self.md.meta.laneWidthLeft=.7;self.md.meta.laneWidthRight=.7
    self.md.laneLineProbs[0]=self.md.laneLineProbs[3]=.05
    self.cs.leftLaneLine=21
    self.assertEqual(self.tick(),[self.k])

  def test_zero_preference_and_lane_change_are_disabled(self):
    self.assertEqual(self.tick(30,preference=0.),[self.k]*30)
    self.md.meta.laneChangeState='laneChangeStarting'
    self.assertEqual(self.tick(30),[self.k]*30)

  def test_existing_boundary_correction_is_not_added_twice(self):
    self.tick(70)
    stronger=self.k-.0015
    self.assertEqual(self.tick(guarded=stronger)[0],stronger)
    protect_left=self.k+.001
    self.assertEqual(self.tick(guarded=protect_left)[0],protect_left)

  def test_duplicate_model_timestamp_cannot_integrate_again(self):
    self.tick(35);before=self.h.accel
    for _ in range(100):
      self.h.update(self.md,self.cs,self.k,self.k,self.t,self.t,
                    preference=-.07,camera_offset=0.)
    self.assertEqual(self.h.accel,before)

  def test_bad_geometry_and_missing_preference_cannot_keep_command(self):
    self.tick(30);self.md.laneLines[1].y[0]=math.nan
    self.assertEqual(self.tick(),[self.k])
    self.assertEqual(self.h.accel,0.)
    self.setUp();self.assertEqual(self.tick(preference=None),[self.k])

  def test_lane_callback_is_applied_before_filter_without_accumulation(self):
    import ast
    from pathlib import Path
    tree=ast.parse((Path(__file__).parents[1]/'controlsd.py').read_text(encoding='utf8'))
    f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='lane_curve_command')
    env=dict(np=np,DT_CTRL=.01,get_lag_adjusted_curvature=lambda *args:.008)
    exec(compile(ast.Module(body=[f],type_ignores=[]),'actual_lane_command','exec'),env)
    fn=env['lane_curve_command'];previous=.008
    calls=[]
    def adjust(k):
      calls.append(k)
      return k-.0005
    for _ in range(500):
      previous,_=fn(None,17.,[],[],[],.4,.1,previous,False,curvature_adjust=adjust)
    self.assertAlmostEqual(previous,.0075,places=8)
    self.assertEqual(len(calls),500)
    self.assertTrue(all(k==.008 for k in calls))


if __name__ == '__main__':
  unittest.main()
