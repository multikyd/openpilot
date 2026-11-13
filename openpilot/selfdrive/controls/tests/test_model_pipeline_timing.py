import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

from openpilot.selfdrive.modeld.pipeline_timing import PipelineTiming

ROOT = Path(__file__).parents[2]


class TestPipelineTiming(unittest.TestCase):
  def test_normal_frames_do_not_log(self):
    h=PipelineTiming()
    for i in range(100):
      t=100.+i*.05
      self.assertIsNone(h.update(t,t-.05,.02,.002,.03))

  def test_long_delivery_gap_reports_stage_times(self):
    h=PipelineTiming();h.update(100.,99.95,.02,.002,.03)
    x=h.update(100.15,100.04,.11,.003,.032)
    self.assertEqual(x['publish_gap_ms'],150.)
    self.assertEqual(x['frame_age_ms'],110.)
    self.assertEqual(x['inference_ms'],32.)

  def test_reports_are_sparse_and_invalid_values_are_ignored(self):
    h=PipelineTiming()
    self.assertIsNotNone(h.update(100.,99.8,.1,.002,.03))
    self.assertIsNone(h.update(100.2,100.,.1,.002,.03))
    self.assertIsNone(h.update(float('nan'),100.,.1,.002,.03))
    self.assertIsNotNone(h.update(102.1,101.9,.1,.002,.03))


class TestEarlyPosePublication(unittest.TestCase):
  def setUp(self):
    model=ast.parse((ROOT/'modeld/modeld.py').read_text(encoding='utf8'))
    filltree=ast.parse((ROOT/'modeld/fill_model_msg.py').read_text(encoding='utf8'))
    fill=next(n for n in filltree.body if isinstance(n,ast.FunctionDef) and n.name=='fill_pose_msg')
    fill.returns=None
    for a in fill.args.args:a.annotation=None
    publish=next(n for n in model.body if isinstance(n,ast.FunctionDef) and n.name=='publish_camera_odometry')
    self.env=dict(messaging=NS(new_message=lambda service:NS(cameraOdometry=NS())))
    exec(compile(ast.Module(body=[fill,publish],type_ignores=[]),'actual_pose_publish','exec'),self.env)
    self.sent=[]
    self.pm=NS(send=lambda service,msg:self.sent.append((service,msg)))
    self.output=dict(pose=np.arange(6.).reshape(1,6),pose_stds=np.ones((1,6)),
                     wide_from_device_euler=np.ones((1,3)),wide_from_device_euler_stds=np.ones((1,3)),
                     road_transform=np.ones((1,3)),road_transform_stds=np.ones((1,3)))
    self.tree=model

  def test_exact_pose_frame_and_validity_are_preserved(self):
    for dropped,calibrated,valid in [(0,True,True),(1,True,False),(0,False,False)]:
      self.env['publish_camera_odometry'](self.pm,self.output,NS(frame_id=51890,timestamp_eof=123456),dropped,calibrated)
      service,msg=self.sent[-1]
      self.assertEqual(service,'cameraOdometry')
      self.assertEqual(msg.valid,valid)
      self.assertEqual(msg.cameraOdometry.frameId,51890)
      self.assertEqual(msg.cameraOdometry.timestampEof,123456)
      self.assertEqual(msg.cameraOdometry.trans,[0.,1.,2.])

  def test_pose_is_published_once_before_model_and_desire_bookkeeping(self):
    main=next(n for n in self.tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    pose=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='publish_camera_odometry']
    model=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='fill_model_msg']
    desire=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='update' and isinstance(n.func.value,ast.Name) and n.func.value.id=='DH']
    self.assertEqual(len(pose),1)
    self.assertLess(pose[0].lineno,model[0].lineno)
    self.assertLess(pose[0].lineno,desire[0].lineno)
