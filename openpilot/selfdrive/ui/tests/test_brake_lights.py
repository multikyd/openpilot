import importlib.util
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace, ModuleType
from unittest.mock import patch, MagicMock
from concurrent.futures import Future
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('brake_core', ROOT/'onroad/brake_lights.py')
core=importlib.util.module_from_spec(spec);sys.modules[spec.name]=core;spec.loader.exec_module(core)


def scene(on=False, factor=1., left_only=False):
  im=np.full((140,160,3),110.,dtype=float)
  for x in (12,136):
    lit=on or (left_only and x==12)
    im[60:78,x:x+10]=(220 if lit else 100, 25, 25)
  im[20:25,68:92]=(220 if on else 80,25,25)
  return np.clip(im*factor,0,255).astype(np.uint8)


class CenterTransitionTest(unittest.TestCase):
  def run_frames(self, state, image, start, count):
    for i in range(count):
      result = state.update(image, start+i*.1)
    return result

  @staticmethod
  def center_on():
    image = scene()
    image[20:25,68:92] = (240,70,30)
    return image

  def test_center_onset_does_not_require_side_brightness_change_or_pair_fit(self):
    state = core.AdaptiveLamps()
    # Side lamps exist, but their pair fit can be ambiguous at close range.
    with patch.object(core, 'locate_lamps', return_value=()):
      self.run_frames(state, scene(), 0., 6)
      result = self.run_frames(state, self.center_on(), .6, 3)
    self.assertEqual((result.state, result.reason), ('ON', 'CENTER_LAMP_RISE'))
    self.assertEqual(len(result.lamps), 1)

  def test_center_single_flash_is_not_confirmed(self):
    state = core.AdaptiveLamps()
    self.run_frames(state, scene(), 0., 6)
    self.assertNotEqual(state.update(self.center_on(), .6).state, 'ON')
    self.assertNotEqual(state.update(scene(), .7).state, 'ON')

  def test_center_loss_immediately_hides_on(self):
    state = core.AdaptiveLamps()
    self.run_frames(state, scene(), 0., 6)
    self.assertEqual(self.run_frames(state, self.center_on(), .6, 3).state, 'ON')
    self.assertNotEqual(state.update(scene(), .9).state, 'ON')

  def test_confirmed_center_survives_dimming_without_erasing_off_reference(self):
    state = core.AdaptiveLamps()
    self.run_frames(state, scene(), 0., 6)
    self.assertEqual(self.run_frames(state, self.center_on(), .6, 3).state, 'ON')
    reference = state.center.reference.copy()
    dim = scene();dim[20:25,68:92] = (125,25,25)
    self.assertEqual(state.update(dim, .9).state, 'ON')
    np.testing.assert_array_equal(state.center.reference, reference)

  def test_confirmed_thin_strip_uses_maintenance_width_not_entry_width(self):
    state = core.AdaptiveLamps()
    self.run_frames(state, scene(), 0., 6)
    self.assertEqual(self.run_frames(state, self.center_on(), .6, 3).state, 'ON')
    thin = scene();thin[20:21,76:85] = (240,70,30)
    self.assertEqual(state.update(thin, .9).reason, 'CENTER_LAMP_RISE')
    fresh = core.AdaptiveLamps()
    self.run_frames(fresh, scene(), 0., 6)
    self.assertNotEqual(self.run_frames(fresh, thin, .6, 5).state, 'ON')

  def test_short_miss_hides_box_but_preserves_reference_for_reacquisition(self):
    state = core.AdaptiveLamps()
    self.run_frames(state, scene(), 0., 6)
    self.run_frames(state, self.center_on(), .6, 3)
    reference = state.center.reference.copy()
    self.assertNotEqual(state.update(scene(), .9).state, 'ON')
    np.testing.assert_array_equal(state.center.reference, reference)
    self.assertEqual(state.update(self.center_on(), 1.).state, 'ON')

  def test_long_loss_discards_confirmed_reference_without_stale_on(self):
    state = core.AdaptiveLamps()
    self.run_frames(state, scene(), 0., 6)
    self.run_frames(state, self.center_on(), .6, 3)
    for i in range(9,15):
      self.assertNotEqual(state.update(scene(), i*.1).state, 'ON')
    self.assertIsNone(state.center.reference)
    self.assertNotEqual(state.update(self.center_on(), 1.5).state, 'ON')

  def test_nearby_component_cannot_replace_a_confirmed_center_during_grace(self):
    state = core.AdaptiveLamps()
    self.run_frames(state, scene(), 0., 6)
    self.run_frames(state, self.center_on(), .6, 3)
    geometry = state.center.geometry.copy()
    different = scene();different[20:25,92:116] = (240,70,30)
    for t in (.9, 1., 1.1):
      self.assertNotEqual(state.update(different, t).reason, 'CENTER_LAMP_RISE')
    np.testing.assert_array_equal(state.center.geometry, geometry)

  def test_initial_center_red_is_not_an_onset(self):
    result = self.run_frames(core.AdaptiveLamps(), self.center_on(), 0., 12)
    self.assertNotEqual(result.state, 'ON')

  def test_global_brightness_change_does_not_confirm_center(self):
    for before, after in ((1., 1.1), (.3, 1.)):
      tracker = core.CenterLampTransitions()
      image = self.center_on()
      for i in range(6):
        rgb = np.clip(image.astype(float)*before,0,255).astype(np.uint8)
        tracker.update(rgb,i*.1,core.red_components(rgb))
      for i in range(6,10):
        rgb = np.clip(image.astype(float)*after,0,255).astype(np.uint8)
        self.assertIsNone(tracker.update(rgb,i*.1,core.red_components(rgb)))

  def test_small_distant_red_dot_and_wide_red_strip_are_rejected(self):
    for xs in (slice(75,80), slice(5,155)):
      tracker = core.CenterLampTransitions()
      for i in range(6):
        rgb = scene()
        tracker.update(rgb,i*.1,core.red_components(rgb))
      rgb = scene();rgb[20:24,xs] = (240,70,30)
      for i in range(6,10):
        self.assertIsNone(tracker.update(rgb,i*.1,core.red_components(rgb)))

  def test_center_without_lower_vehicle_lamp_evidence_is_not_initialized(self):
    tracker = core.CenterLampTransitions()
    rgb = np.full((140,160,3),110,np.uint8)
    for i in range(6):tracker.update(rgb,i*.1,core.red_components(rgb))
    rgb[20:25,68:92] = (240,70,30)
    for i in range(6,10):
      self.assertIsNone(tracker.update(rgb,i*.1,core.red_components(rgb)))

  def test_target_reset_and_time_gap_clear_center_evidence(self):
    for reset in (True, False):
      state = core.AdaptiveLamps()
      self.run_frames(state, scene(), 0., 6)
      self.assertEqual(self.run_frames(state, self.center_on(), .6, 3).state, 'ON')
      if reset:state.reset()
      self.assertNotEqual(state.update(self.center_on(), 2.).state, 'ON')


class AdaptiveTest(unittest.TestCase):
  def advance(self,state,image,start,n):
    result=None
    for i in range(n):result=state.update(image,start+i*.2)
    return result

  def test_initially_lit_is_unknown(self):
    result=self.advance(core.AdaptiveLamps(),scene(True),0,12)
    self.assertEqual(result.state,'UNKNOWN')

  def test_rise_then_fall(self):
    state=core.AdaptiveLamps()
    self.advance(state,scene(),0,8)
    self.assertEqual(self.advance(state,scene(True),1.6,3).state,'ON')
    self.assertEqual(self.advance(state,scene(),2.2,8).state,'OFF')

  def test_single_frame_does_not_confirm(self):
    state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
    self.assertNotEqual(state.update(scene(True),1.6).state,'ON')

  def test_one_side_and_global_exposure(self):
    for image in (scene(left_only=True),scene(factor=.7),scene(factor=1.1)):
      state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
      self.assertNotEqual(self.advance(state,image,1.6,5).state,'ON')

  def test_on_is_not_held_when_evidence_disappears(self):
    state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
    self.assertEqual(self.advance(state,scene(True),1.6,3).state,'ON')
    self.assertNotEqual(state.update(scene(),2.2).state,'ON')

  def test_loss_clears_on_and_baseline(self):
    state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
    self.assertEqual(self.advance(state,scene(True),1.6,3).state,'ON')
    result=state.update(np.zeros((140,160,3),np.uint8),2.2)
    self.assertEqual(result.state,'UNKNOWN');self.assertFalse(state.confirmed_on)
    self.assertFalse(result.lamps)
    state.update(np.zeros((140,160,3),np.uint8),2.7)
    self.assertIsNone(state.baseline)

  def test_gap_and_backward_time_reset(self):
    for t in (4.,0.):
      state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
      self.assertEqual(state.update(scene(True),t).state,'UNKNOWN')

  def test_no_pair_and_adjacent_pair_rejected(self):
    self.assertFalse(core.locate_lamps(np.zeros((140,160,3),np.uint8)))
    im=np.zeros((140,160,3),np.uint8)+110
    im[60:75,2:7]=(180,20,20);im[60:75,60:65]=(180,20,20)
    self.assertFalse(core.locate_lamps(im))

  def test_fragmented_background_has_bounded_candidates(self):
    im=np.zeros((140,160,3),np.uint8)+100
    for y in range(0,140,7):
      for x in range(0,160,7):im[y:y+2,x:x+2]=(200,20,20)
    self.assertEqual(core.red_components(im),[])
    self.assertFalse(core.locate_lamps(im))

  def test_bottom_ground_clipping_keeps_visible_lamps(self):
    k=np.array([[672,1141,0],[380,0,1141],[1,0,0]],float)
    roi=core.project_search(k,4,0,1.4,(760,1344))
    self.assertIsNotNone(roi)
    self.assertEqual(roi[1]+roi[3],760)
    self.assertIsNone(core.project_search(k,4,0,2.4,(760,1344)))

  def test_brief_miss_preserves_reference_without_old_on(self):
    state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
    before=state.baseline.copy()
    result=state.update(np.zeros((140,160,3),np.uint8),1.6)
    self.assertEqual(result.state,'UNKNOWN');self.assertFalse(result.lamps)
    self.assertTrue(np.array_equal(before,state.baseline))
    self.assertEqual(state.update(scene(),1.7).state,'UNKNOWN')
    self.assertGreater(len(state.history),4)

  def test_gradual_brightness_change_does_not_create_onset(self):
    state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
    for i in range(80):
      im=scene();im[60:78,12:22,0]=100+i;im[60:78,136:146,0]=100+i
      im[20:25,68:92,0]=80+i
      self.assertNotEqual(state.update(im,1.6+i*.1).state,'ON')

  def test_geometry_discontinuity_discards_old_reference(self):
    state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
    im=np.roll(scene(True),20,axis=0)
    self.assertEqual(state.update(im,1.6).state,'UNKNOWN')
    self.assertLessEqual(len(state.history),1)

  def test_body_shadow_without_lamp_increase_is_not_on(self):
    state=core.AdaptiveLamps();self.advance(state,scene(),0,8)
    shadow=scene()
    shadow[90:125,22:136]=55
    self.assertNotEqual(self.advance(state,shadow,1.6,6).state,'ON')

  def test_tiny_yellow_sticker_is_not_a_center_brake_lamp(self):
    image=scene();image[20:25,68:92]=110
    image[25:27,79:81]=(220,155,40)
    lamps=core.locate_lamps(image)
    self.assertEqual(len(lamps),2)

  def test_dark_body_is_unknown(self):
    im=scene();im[80:130,22:136]=0
    self.assertEqual(core.AdaptiveLamps().update(im,0).state,'UNKNOWN')

  def test_large_saturated_lamps_are_kept_and_reflectors_not_selected(self):
    im=np.full((160,180,3),80,np.uint8)
    im[50:88,15:35]=(230,15,15);im[50:88,145:165]=(230,15,15)
    im[55:82,20:30]=(255,240,225);im[55:82,150:160]=(255,240,225)
    im[20:27,65:115]=(255,20,20)
    im[125:132,15:35]=(180,15,15);im[125:132,145:165]=(180,15,15)
    lamps=core.locate_lamps(im)
    self.assertGreaterEqual(len(lamps),2)
    self.assertTrue(all(y<100 for x,y,w,h in lamps[:2]))

  def test_large_uniform_red_background_is_still_rejected(self):
    self.assertFalse(core.locate_lamps(np.full((140,160,3),(220,20,20),np.uint8)))

  def test_projection_rejects_offscreen_small_invalid(self):
    k=np.array([[672,1141,0],[380,0,1141],[1,0,0]],float)
    self.assertIsNotNone(core.project_search(k,20,0,1.2,(760,1344)))
    for d,y in ((2,0),(70,0),(20,50),(float('nan'),0)):
      self.assertIsNone(core.project_search(k,d,y,1.2,(760,1344)))

  def test_nv12_padding_and_bounded_copy(self):
    raw=np.zeros(30000,np.uint8);raw[:128*80]=80;raw[12000:]=128
    frame=SimpleNamespace(data=raw,stride=128,uv_offset=12000)
    rgb,step=core.nv12_crop(frame,(10,10,80,50),max_size=30)
    self.assertLessEqual(max(rgb.shape[:2]),30)
    self.assertTrue(np.all(rgb==80));raw[:]=0
    self.assertTrue(np.all(rgb==80))


class OverlayLifecycleTest(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    ray=MagicMock();ray.Rectangle=lambda x,y,w,h:SimpleNamespace(x=x,y=y,width=w,height=h)
    dependencies={
      'pyray':ray,
      'openpilot.common.swaglog':SimpleNamespace(cloudlog=MagicMock()),
      'openpilot.selfdrive.controls.radard':SimpleNamespace(RADAR_TO_CAMERA=1.52),
      'openpilot.selfdrive.ui.onroad.brake_lights':core,
      'openpilot.system.ui.lib.application':SimpleNamespace(gui_app=MagicMock(),FontWeight=SimpleNamespace(MEDIUM=1)),
    }
    spec=importlib.util.spec_from_file_location('brake_overlay_test',ROOT/'onroad/brake_light_overlay.py')
    cls.module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,dependencies):spec.loader.exec_module(cls.module)

  def setUp(self):
    self.o=self.module.BrakeLightOverlay()
    self.o.enabled=True
    self.sm=type('SM',(dict,),{})()
    for name in ('modelV2','radarState','extrinsicsCalibration'):
      self.sm[name]=SimpleNamespace()
    self.sm.valid={s:True for s in self.sm};self.sm.alive=self.sm.valid.copy();self.sm.recv_time={s:100. for s in self.sm}
    self.sm['extrinsicsCalibration']=SimpleNamespace(calStatus=1,height=[1.2])
    self.sm['radarState']=SimpleNamespace(leadOne=SimpleNamespace(present=True,dRel=20.,yRel=0.,vRel=0.,radarTrackId=3))
    self.sm['modelV2']=SimpleNamespace(frameId=100,position=SimpleNamespace(x=[0,100],z=[0,0]))
    self.frame=SimpleNamespace(frame_id=100,width=80,height=60,stride=80,uv_offset=4800,data=np.full(7200,128,np.uint8))
    self.view=SimpleNamespace(frame=self.frame,client=SimpleNamespace(timestamp_sof=100e9),_calc_frame_matrix=lambda rect:np.eye(3))
    self.config=SimpleNamespace(width=80,height=60,intrinsics=np.eye(3))
    self.rect=SimpleNamespace(x=0,y=0,width=80,height=60)
    self.o.last_submit=100. # no work submission in lifecycle tests

  def run_render(self):
    with patch.object(self.module,'project_search',return_value=(0,0,80,60)):
      self.o._render(self.view,self.rect,self.sm,self.config,np.eye(3),0,100.)

  def test_stale_radar_camera_or_calibration_clears(self):
    for kind in ('radar','camera','calibration'):
      self.setUp();self.o.previous=(99.9,21.52,0.,0.,3,0);self.o.display=('old',)
      if kind=='radar':self.sm.recv_time['radarState']=99.
      if kind=='camera':self.view.client.timestamp_sof=99e9
      if kind=='calibration':self.sm['extrinsicsCalibration'].calStatus=0
      self.run_render();self.assertIsNone(self.o.display);self.assertIsNone(self.o.previous)

  def test_track_switch_discards_old_worker_result(self):
    self.o.previous=(99.9,21.52,0.,0.,2,0)
    self.o.context=(0,tuple(np.eye(3).ravel()))
    result=Future();result.set_result((0,99.9,(0,0,80,60),1,core.Observation('ON'),99,1.,np.zeros((60,80,3),np.uint8),None))
    self.o.pending=result
    self.run_render()
    self.assertGreater(self.o.epoch,0);self.assertIsNone(self.o.display)

  def test_sidebar_observation_expires_and_hides_after_clear(self):
    self.o.previous=(99.9,21.52,0.,0.,3,0)
    self.o.display=(99.9,(0,0,80,60),1,core.Observation('ON'))
    self.assertEqual(self.o.latest_observation(100.).state,'ON')
    self.assertIsNone(self.o.latest_observation(100.5))
    self.assertIsNone(self.o.latest_observation(99.8))
    self.o.clear();self.assertIsNone(self.o.latest_observation(100.))

  def test_no_lead_clears(self):
    self.sm['radarState'].leadOne.present=False
    self.o.previous=(99.9,21.52,0.,0.,3,0);self.o.display=('old',)
    self.run_render();self.assertIsNone(self.o.display)

  def test_frame_model_mismatch_rejected(self):
    self.sm['modelV2'].frameId=70
    self.run_render();self.assertIsNone(self.o.previous)

  def test_manual_sample_expires_and_is_cleared_with_target(self):
    self.o.previous=(99.9,21.52,0.,0.,3,0)
    self.o.label_cache=dict(mono=99.9,source_sof=99.9)
    self.assertIsNotNone(self.o.label_sample(100.))
    self.assertIsNone(self.o.label_sample(100.3))
    self.o.clear();self.assertIsNone(self.o.label_sample(100.))

  def test_manual_sample_rejects_old_source_even_if_recently_copied(self):
    self.o.previous=(100.,21.52,0.,0.,3,0)
    self.o.label_cache=dict(mono=100.,source_sof=99.6)
    self.assertIsNone(self.o.label_sample(100.1))

  def test_completed_crop_and_boxes_have_same_frame(self):
    self.o.pending_sample=dict(mono=99.9,source_sof=99.9,frame_id=99,rgb=np.zeros((10,10,3),np.uint8))
    result=Future()
    lamps=((1,2,3,4),(6,2,3,4))
    result.set_result((0,99.9,(0,0,80,60),1,core.Observation('ON',lamps=lamps),99,1.,np.zeros((60,80,3),np.uint8),None))
    self.o.pending=result
    self.run_render()
    sample=self.o.label_sample(100.)
    self.assertEqual(sample['frame_id'],99)
    self.assertEqual(sample['lamps'],lamps)
    self.assertEqual(sample['prediction'],'ON')

  def test_delayed_worker_keeps_new_scale_and_detached_camera_pixels(self):
    class Deferred:
      def submit(self,fn):
        self.fn=fn;self.future=Future();return self.future
    deferred=Deferred();self.o.pool=deferred;self.o.last_submit=0.
    self.o.display=(99.9,(0,0,80,60),1,core.Observation('UNKNOWN'))
    original=core.nv12_crop
    with patch.object(self.module,'nv12_crop',side_effect=lambda fr,roi:original(fr,roi,max_size=30)):
      self.run_render()  # Old display is read before the new job completes.
      self.frame.data[:]=0
      result=deferred.fn()
    self.assertEqual(result[3],3)
    self.assertTrue(np.all(result[7]==128))
    deferred.future.set_result(result)
    self.run_render()
    sample=self.o.label_sample(100.)
    self.assertEqual(sample['step'],3)
    self.assertEqual(sample['frame_id'],100)
    self.assertEqual(sample['resampling'],'point')

  def test_only_vehicle_outline_is_drawn_for_all_lamp_states(self):
    ray=self.module.rl
    for state,expected in [('ON',0),('UNKNOWN',0),('OFF',0)]:
      ray.draw_rectangle_rec.reset_mock()
      self.o.display=(99.9,(0,0,80,60),1,core.Observation(state,lamps=((5,30,8,4),(60,30,8,4))), (2,2,74,54))
      self.run_render()
      self.assertEqual(ray.draw_rectangle_rec.call_count,expected)
      for call in ray.draw_rectangle_rec.call_args_list:self.assertEqual(call.args[0].width,8.)

  def test_model_outline_is_not_duplicated_with_heuristic_outline(self):
    ray=self.module.rl
    ray.draw_rectangle_lines_ex.reset_mock()
    self.o.display=(99.9,(0,0,80,60),1,core.Observation('UNKNOWN'),(2,2,74,54))
    with patch.object(self.o.vehicle_model,'tick',return_value=(4,4,70,50)):
      self.run_render()
    self.assertEqual(ray.draw_rectangle_lines_ex.call_count,1)
    self.assertEqual(self.o.latest_observation(100.).state,'UNKNOWN')

  def test_model_outline_can_render_without_inventing_lamp_boxes(self):
    ray=self.module.rl
    ray.draw_rectangle_lines_ex.reset_mock()
    ray.draw_rectangle_rec.reset_mock()
    self.o.display=None
    with patch.object(self.o.vehicle_model,'tick',return_value=(4,4,70,50)):
      self.run_render()
    self.assertEqual(ray.draw_rectangle_lines_ex.call_count,1)
    ray.draw_rectangle_rec.assert_not_called()

  def test_model_on_hint_colors_vehicle_without_drawing_lamp_boxes(self):
    ray=self.module.rl
    ray.draw_rectangle_rec.reset_mock()
    self.o.display=(99.9,(0,0,80,60),1,core.Observation('UNKNOWN',lamps=((5,30,8,4),(60,30,8,4))),None)
    with patch.object(self.o.vehicle_model,'tick',return_value=(4,4,70,50)), patch.object(self.o.vehicle_model,'current_state',return_value=dict(state='ON',confidence=.95,age=.1)):
      self.run_render()
      self.assertEqual(self.o.latest_observation(100.).state,'ON')
    self.assertEqual(ray.draw_rectangle_rec.call_count,0)
    self.assertEqual(self.o.display[3].state,'UNKNOWN')
    for call in ray.draw_rectangle_rec.call_args_list:self.assertEqual(call.args[0].width,8.)


class CropAndOutlineTest(unittest.TestCase):
  def test_odd_aligned_copy_matches_source_and_survives_camera_overwrite(self):
    raw=np.arange(24*18*3//2,dtype=np.uint8)
    frame=SimpleNamespace(data=raw,width=20,height=18,stride=24,uv_offset=24*18)
    roi=(3,1,13,13);expected,step=core.nv12_crop(frame,roi,max_size=7)
    copied,local=core.copy_nv12_roi(frame,roi)
    raw[:]=0
    result,result_step=core.nv12_crop(copied,local,max_size=7)
    np.testing.assert_array_equal(result,expected);self.assertEqual(result_step,step)

  def test_outline_uses_image_edges_and_requires_repeated_evidence(self):
    rgb=np.full((160,180,3),40,np.uint8);rgb[15:140,15:165]=140
    lamps=((24,80,12,8),(144,80,12,8));g=core.VehicleOutline()
    self.assertIsNone(g.update(rgb,lamps,0.));self.assertIsNone(g.update(rgb,lamps,.1))
    x,y,w,h=g.update(rgb,lamps,.2)
    self.assertAlmostEqual(x,14.);self.assertAlmostEqual(y,14.)
    self.assertAlmostEqual(w,150.);self.assertAlmostEqual(h,125.)
    self.assertIsNone(g.update(rgb,(),.3))

  def test_textureless_or_ambiguous_outline_is_not_a_vehicle_rectangle(self):
    g=core.VehicleOutline();rgb=np.full((160,180,3),100,np.uint8)
    for i in range(10):self.assertIsNone(g.update(rgb,((24,80,12,8),(144,80,12,8)),i*.1))


if __name__=='__main__':unittest.main()
