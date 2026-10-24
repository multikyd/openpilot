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

  def test_popup_big_buttons_fit_c4_and_expire(self):
    controls = self.module.BrakeLabelControls()
    detector = N(enabled=True,epoch=3,label_sample=lambda now:None)
    rect = N(x=0,y=0,width=536,height=240)
    with patch.object(self.module.time,'monotonic',return_value=10.):
      controls.render(rect,detector)
      self.assertTrue(controls.tap(N(x=490,y=100),detector))
      controls.render(rect,detector)
      self.assertEqual([a for r,a in controls.buttons],['ON','OFF','CANCEL'])
      for r,a in controls.buttons:
        self.assertGreaterEqual(r.width,70);self.assertGreaterEqual(r.height,44)
        self.assertLessEqual(r.x+r.width,536);self.assertLessEqual(r.y+r.height,240)
    with patch.object(self.module.time,'monotonic',return_value=16.):
      controls.render(rect,detector);self.assertIsNone(controls.opened)

  def test_changed_target_cannot_be_labelled(self):
    controls = self.module.BrakeLabelControls()
    detector = N(enabled=True,epoch=3,label_sample=lambda now:{'mono':now})
    with patch.object(self.module.time,'monotonic',return_value=10.):
      controls.render(N(x=0,y=0,width=536,height=240),detector)
      controls.tap(N(x=490,y=100),detector)
      controls.render(N(x=0,y=0,width=536,height=240),detector)
      detector.epoch = 4
      self.assertTrue(controls.tap(N(x=490,y=90),detector))
      self.assertEqual(controls.message,'NO FRAME')


if __name__ == '__main__':
  unittest.main()
