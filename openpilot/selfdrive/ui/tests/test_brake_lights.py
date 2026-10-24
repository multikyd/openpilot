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
    self.view=SimpleNamespace(frame=self.frame,client=SimpleNamespace(timestamp_sof=100e9))
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
    result=Future();result.set_result((0,99.9,(0,0,80,60),1,core.Observation('ON'),99,1.))
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


if __name__=='__main__':unittest.main()
