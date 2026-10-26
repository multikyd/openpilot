import unittest
from queue import Queue
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from openpilot.selfdrive.ui.onroad import brake_vehicle_model as model


class FakeQueue(Queue):
  def close(self):
    pass

  def cancel_join_thread(self):
    pass


class FakeProcess:
  def is_alive(self):
    return True

  def terminate(self):
    pass

  def join(self, timeout=None):
    pass


class VehicleModelTest(unittest.TestCase):
  def helper(self):
    with patch.dict('os.environ', {'LEAD_BRAKE_MODEL': '0'}):
      helper = model.BrakeVehicleModel()
    helper.enabled = helper.ready = True
    helper.process = FakeProcess()
    helper.inbox, helper.outbox = FakeQueue(1), FakeQueue(2)
    helper.last_submit = 100.
    return helper

  def test_on_off_display_hints_do_not_override_fast_release_or_onset(self):
    on, off = dict(state='ON'), dict(state='OFF')
    self.assertEqual(model.fuse_observation(model.Observation(), on).state, 'ON')
    self.assertEqual(model.fuse_observation(model.Observation(), off).state, 'OFF')
    self.assertEqual(model.fuse_observation(model.Observation('ON'), off).state, 'ON')
    self.assertEqual(model.fuse_observation(model.Observation('OFF'), on).state, 'UNKNOWN')
    self.assertEqual(model.fuse_observation(model.Observation('UNKNOWN','RELEASE_PENDING'), on).state, 'UNKNOWN')

  def test_model_state_is_more_short_lived_than_outline(self):
    h = self.helper()
    h.result = (0,100.,(100,100,100,100),dict(box=(110,110,80,80),state='ON',confidence=.9))
    self.assertEqual(h.current_state((100,100,100,100),0,100.5)['state'],'ON')
    self.assertIsNone(h.current_state((100,100,100,100),0,101.6))
    self.assertIsNotNone(h.current_box((100,100,100,100),0,101.6))
    h.result[-1]['confidence'] = .7
    self.assertIsNone(h.current_state((100,100,100,100),0,100.5))

  def test_decode_scale_and_class_agnostic_nms(self):
    raw = np.zeros((1,6,8400), np.float32)
    raw[0,:,0] = (150,200,100,80,.1,.9)
    raw[0,:,1] = (151,200,100,80,.85,.1)
    found = model.decode_predictions(raw,1280,640)
    self.assertEqual(len(found),1)
    np.testing.assert_allclose(found[0]['box'], (200,160,200,80))
    self.assertEqual(found[0]['state'],'ON')

  def test_malformed_and_nonfinite_model_output(self):
    with self.assertRaises(ValueError):
      model.decode_predictions(np.zeros((1,7,8400)),640,640)
    raw = np.zeros((1,6,8400),np.float32)
    raw[0,:,0] = (np.nan,200,100,80,.1,.99)
    self.assertEqual(model.decode_predictions(raw,640,640),[])

  def test_sign_and_neighbour_do_not_replace_projected_lead(self):
    car = dict(box=(110,110,90,100),confidence=.8,state='ON')
    sign = dict(box=(420,50,140,200),confidence=.99,state='ON')
    self.assertIs(model.select_lead_box([sign,car],(100,100,110,120)),car)
    self.assertIsNone(model.select_lead_box([sign],(100,100,110,120)))

  def test_ambiguous_overlapping_vehicles_hide_outline(self):
    a = dict(box=(100,100,100,100),confidence=.8,state='ON')
    b = dict(box=(105,100,100,100),confidence=.8,state='OFF')
    self.assertIsNone(model.select_lead_box([a,b],(100,100,100,100)))

  def test_current_box_follows_small_projection_motion_and_expires(self):
    h = self.helper()
    h.result = (2,100.,(100,100,100,100),dict(box=(110,110,80,80),state='ON',confidence=.9))
    np.testing.assert_allclose(h.current_box((105,105,100,100),2,100.5),(115,115,80,80))
    self.assertIsNone(h.current_box((105,105,100,100),3,100.5))
    self.assertIsNone(h.current_box((105,105,100,100),2,102.01))
    self.assertIsNone(h.current_box((200,100,100,100),2,100.5))
    h.clear()
    self.assertIsNone(h.result)

  def test_stale_generation_does_not_adopt_model_result(self):
    h = self.helper()
    h.busy = True
    h.outbox.put(('result',1,99.9,(0,0,80,60),dict(box=(0,0,80,60),confidence=.8,state='ON'),.1))
    frame = NS(width=80,height=60)
    self.assertIsNone(h.tick(frame,(0,0,80,60),2,100.))
    self.assertIsNone(h.result)

  def test_slow_inference_disables_without_affecting_lamp_observer(self):
    h = self.helper()
    h.outbox.put(('result',0,99.9,(0,0,80,60),None,1.1))
    self.assertIsNone(h.tick(NS(),(0,0,80,60),0,100.))
    self.assertFalse(h.enabled)
    self.assertEqual(h.status,'inference_too_slow')

  def test_one_pending_frame_and_adaptive_duty_cycle(self):
    h = self.helper()
    h.busy = True
    h.submitted = 100.
    h.last_submit = 90.
    with patch.object(model,'copy_nv12_roi') as copied:
      h.tick(NS(),(0,0,80,60),0,100.)
      copied.assert_not_called()
    h.outbox.put(('result',0,100.,(0,0,80,60),None,.4))
    h.last_submit = 100.
    h.tick(NS(),(0,0,80,60),0,100.5)
    self.assertEqual(h.interval,2.)
    self.assertFalse(h.busy)

  def test_sustained_low_ui_fps_disables_optional_worker(self):
    h = self.helper()
    h.tick(NS(),(0,0,80,60),0,100.,fps=25)
    self.assertTrue(h.enabled)
    self.assertIsNone(h.tick(NS(),(0,0,80,60),0,103.1,fps=25))
    self.assertFalse(h.enabled)
    self.assertEqual(h.status,'ui_fps_low')

  def test_gray_nv12_input_has_correct_scale_shape_and_no_shared_view(self):
    raw = np.full(80*60*3//2,128,np.uint8)
    frame = NS(data=raw,stride=80,uv_offset=80*60)
    tensor = model.nv12_input(frame,80,60)
    self.assertEqual(tensor.shape,(1,3,640,640))
    self.assertEqual(tensor.dtype,np.float32)
    np.testing.assert_allclose(tensor,128/255,atol=1e-6)
    raw[:]=0
    np.testing.assert_allclose(tensor,128/255,atol=1e-6)

  def test_launcher_drops_realtime_policy_before_queue_feeder_and_spawn(self):
    calls=[]
    class Process(FakeProcess):
      def start(self):
        calls.append('spawn')
    class Context:
      def Queue(self, maxsize):
        calls.append('queue')
        return FakeQueue(maxsize)
      def Process(self, **kwargs):
        return Process()
    h = self.helper()
    h.process = None
    h.launching = True
    with patch.object(model,'_background_priority',side_effect=lambda:calls.append('priority')), patch.object(model.mp,'get_context',return_value=Context()):
      h._launch()
    self.assertEqual(calls,['priority','queue','queue','spawn'])
    h.close()


if __name__ == '__main__':
  unittest.main()
