from concurrent.futures import Future
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as N
import unittest
from unittest.mock import MagicMock, patch
import numpy as np

UI = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('brake_labels_test_module', UI/'onroad/brake_labels.py')
labels = importlib.util.module_from_spec(spec)
spec.loader.exec_module(labels)


class LabelWriterTest(unittest.TestCase):
  def sample(self):
    return dict(mono=10., frame_id=42, radar_track_id=7, rgb=np.zeros((16,20,3), np.uint8))

  def test_fresh_owned_image_and_metadata_are_saved(self):
    with tempfile.TemporaryDirectory() as directory:
      writer = labels.BrakeLabelWriter(directory)
      sample = self.sample()
      self.assertEqual(writer.submit(sample, 'OFF', 10.2, 1790000000.), 'SAVING')
      self.assertEqual(writer.pending.result(timeout=5), 'SAVED')
      self.assertEqual(writer.result(), 'SAVED')
      path = next(Path(directory).glob('*.json'))
      saved = json.loads(path.read_text())
      self.assertEqual((saved['label'], saved['frame_id'], saved['radar_track_id']), ('OFF',42,7))
      self.assertTrue(saved['display_only'])
      self.assertEqual((Path(directory)/saved['image']).read_bytes(), b'P6\n20 16\n255\n'+sample['rgb'].tobytes())
      self.assertFalse(list(Path(directory).glob('*.tmp')))
      writer.close()

  def test_stale_future_absent_and_invalid_labels_rejected(self):
    writer = labels.BrakeLabelWriter()
    for sample,label,now in ((None,'ON',10.),(self.sample(),'ON',10.36),
                             (self.sample(),'ON',9.9),(self.sample(),'OTHER',10.)):
      self.assertEqual(writer.submit(sample,label,now,1790000000.), 'NO FRAME')
    self.assertIsNone(writer.pool)

  def test_frozen_review_and_box_feedback_saved(self):
    with tempfile.TemporaryDirectory() as directory:
      writer=labels.BrakeLabelWriter(directory);sample=self.sample();sample['review_started_mono']=10.1
      self.assertEqual(writer.submit(sample,'ON',12.,1790000000.,box_feedback='missing'),'SAVING')
      self.assertEqual(writer.pending.result(timeout=5),'SAVED')
      record=json.loads(next(Path(directory).glob('*.json')).read_text())
      self.assertEqual(record['box_feedback'],'missing');self.assertEqual(record['format_version'],2)
      writer.close()
      self.assertEqual(writer.submit(sample,'ON',21.,1790000000.,box_feedback='missing'),'NO FRAME')

  def test_no_unbounded_queue(self):
    writer = labels.BrakeLabelWriter()
    writer.pending = Future()
    self.assertEqual(writer.submit(self.sample(),'ON',10.,1790000000.), 'BUSY')
    self.assertIsNone(writer.pool)

  def test_invalid_crop_and_worker_error(self):
    writer = labels.BrakeLabelWriter()
    sample = self.sample();sample['rgb'] = np.zeros((193,20,3),np.uint8)
    self.assertEqual(writer.submit(sample,'ON',10.,1790000000.),'NO FRAME')
    writer.pending = Future();writer.pending.set_exception(OSError('disk full'))
    self.assertEqual(writer.result(),'SAVE ERROR')


class LabelControlsTest(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    ray = MagicMock()
    ray.Rectangle = lambda x,y,w,h: N(x=x,y=y,width=w,height=h)
    ray.Vector2 = lambda x,y: N(x=x,y=y)
    cls.now = 10.
    dependencies = {'pyray':ray,
      'openpilot.selfdrive.ui.mici.onroad':N(SIDE_PANEL_WIDTH=80),
      'openpilot.selfdrive.ui.mici.onroad.hud_layout':N(SPEED_HEADER_HEIGHT=70),
      'openpilot.selfdrive.ui.onroad.brake_labels':labels,
      'openpilot.system.ui.lib.application':N(gui_app=N(font=lambda *a:None),FontWeight=N(BOLD=1)),
      'openpilot.system.ui.lib.text_measure':N(measure_text_cached=lambda f,t,s:N(x=len(t)*s*.65,y=s))}
    spec = importlib.util.spec_from_file_location('brake_label_controls_test_module',UI/'mici/onroad/brake_label_controls.py')
    cls.module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,dependencies):spec.loader.exec_module(cls.module)

  def detector(self):
    self.rgb=np.zeros((40,60,3),np.uint8)
    return N(enabled=True,epoch=3,label_sample=lambda now:dict(rgb=self.rgb,mono=now,frame_id=42,lamps=[]))

  def test_popup_large_buttons_and_whole_icon_touch_area(self):
    controls=self.module.BrakeLabelControls();detector=self.detector();rect=N(x=0,y=0,width=536,height=240)
    with patch.object(self.module.time,'monotonic',return_value=10.):
      controls.render(rect,detector)
      hit=controls.buttons[0][0]
      self.assertGreaterEqual(hit.width,90);self.assertGreaterEqual(hit.height,88)
      self.assertTrue(controls.tap(N(x=490,y=130),detector))
      controls.render(rect,detector)
      self.assertEqual([a for r,a in controls.buttons],['ON','OFF','correct','missing','SAVE','CANCEL'])
      for r,a in controls.buttons:
        self.assertGreaterEqual(r.width,130);self.assertGreaterEqual(r.height,48)
        self.assertLessEqual(r.x+r.width,536);self.assertLessEqual(r.y+r.height,240)
    with patch.object(self.module.time,'monotonic',return_value=21.):
      controls.render(rect,detector);self.assertIsNone(controls.opened)

  def test_frozen_frame_feedback_matches_original_not_new_target(self):
    controls=self.module.BrakeLabelControls();detector=self.detector();rect=N(x=0,y=0,width=536,height=240)
    with patch.object(self.module.time,'monotonic',return_value=10.):
      controls.render(rect,detector);controls.tap(N(x=490,y=120),detector)
      controls.render(rect,detector)
    self.rgb[:]=255;detector.epoch=4
    self.assertTrue(np.all(controls.sample['rgb']==0))
    with patch.object(self.module.time,'monotonic',return_value=12.),patch.object(controls.writer,'submit',return_value='SAVING') as submit:
      controls.tap(N(x=300,y=60),detector) # ON
      controls.tap(N(x=450,y=125),detector) # missing box
      controls.tap(N(x=300,y=200),detector) # SAVE
      self.assertEqual(submit.call_args.args[0]['frame_id'],42)
      self.assertEqual(submit.call_args.args[1],'ON')
      self.assertEqual(submit.call_args.kwargs['box_feedback'],'missing')
      self.assertIsNone(controls.opened)

  def test_modal_touch_does_not_leak_and_cancel_frees_preview(self):
    controls=self.module.BrakeLabelControls();controls.opened=10.;controls.texture=N(id=5)
    with patch.object(self.module.time,'monotonic',return_value=10.):
      self.assertTrue(controls.tap(N(x=1,y=1),self.detector()))
    controls.dismiss();self.assertIsNone(controls.texture)

  def test_small_bookmark_direction_jitter_still_taps_car(self):
    import ast
    tree=ast.parse((UI/'mici/onroad/augmented_road_view.py').read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='AugmentedRoadView')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_handle_mouse_release')
    env=dict(ui_state=N(started=True),MousePos=object)
    exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_car_tap','exec'),env)
    tap=MagicMock(return_value=True)
    view=N(_bookmark_icon=N(interacting=lambda:True),_label_press=N(x=490,y=100),
           _label_dragged=False,_brake_labels=N(tap=tap),_brake_light_overlay=N())
    env['_handle_mouse_release'](view,N(x=489,y=100))
    tap.assert_called_once()
    tap.reset_mock();view._label_dragged=True;view._label_press=N(x=490,y=100)
    env['_handle_mouse_release'](view,N(x=300,y=100))
    tap.assert_not_called()


if __name__ == '__main__':
  unittest.main()
