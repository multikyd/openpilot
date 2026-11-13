import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("camera_validity", Path(__file__).resolve().parents[1] / "camera_odometry_validity.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class TestCameraOdometryValidity(unittest.TestCase):
  def setUp(self):
    self.guard = module.CameraOdometryValidity()
    self.assertTrue(self.guard.update(True, 100, 10., 10.01, True))

  def test_isolated_skip_and_recovery(self):
    self.assertTrue(self.guard.update(False, 102, 10.10, 10.11, False))
    self.assertTrue(self.guard.update(True, 103, 10.15, 10.16, True))

  def test_consecutive_invalid(self):
    self.assertTrue(self.guard.update(False, 102, 10.1, 10.11, False))
    self.assertFalse(self.guard.update(False, 104, 10.2, 10.21, False))

  def test_no_history_or_no_skipped_frame(self):
    self.assertFalse(module.CameraOdometryValidity().update(False, 102, 10.1, 10.11, False))
    self.assertFalse(self.guard.update(False, 101, 10.05, 10.06, False))

  def test_stale_or_large_frame_gap(self):
    for frame, eof, stamp in [(102,10.2,10.21),(104,10.1,10.11),(102,10.1,10.3)]:
      self.setUp()
      self.assertFalse(self.guard.update(False, frame, eof, stamp, False))

  def test_estimator_rejection_cannot_enable_grace(self):
    self.guard.update(True, 101, 10.05, 10.06, False)
    self.assertFalse(self.guard.update(False, 103, 10.15, 10.16, False))

  def test_timestamp_reset_or_nonfinite(self):
    for eof, stamp in [(9.9,10.11),(10.1,9.9),(float('nan'),10.11)]:
      self.setUp()
      self.assertFalse(self.guard.update(False, 102, eof, stamp, False))

  def test_startup_validity_preserved(self):
    self.assertTrue(module.CameraOdometryValidity().update(True, 0, 0., 0., False))


class TestCalibrationOdometryValidity(unittest.TestCase):
  def setUp(self):
    self.guard = module.CalibrationOdometryValidity()
    self.guard.update(True, 100, 10., 10.01, trusted=True, now=10.02)

  def skip(self, **kw):
    args = dict(valid=False, frame_id=102, timestamp_eof=10.1, log_time=10.11,
                trusted=True, now=10.12)
    args.update(kw)
    return self.guard.update(**args)

  def test_isolated_skip_uses_recent_trusted_calibration(self):
    self.assertTrue(self.skip())
    self.assertTrue(self.guard.can_hold(10.14, trusted=True))

  def test_consecutive_skips_share_one_half_second_deadline(self):
    self.assertTrue(self.skip())
    self.assertTrue(self.skip(frame_id=104, timestamp_eof=10.2, log_time=10.21, now=10.22))
    self.assertTrue(self.skip(frame_id=108, timestamp_eof=10.4, log_time=10.41, now=10.42))
    self.assertFalse(self.guard.can_hold(10.501, trusted=True))
    self.assertFalse(self.skip(frame_id=110, timestamp_eof=10.5, log_time=10.51, now=10.52))

  def test_timeout_and_future_or_nonfinite_clock(self):
    for now in (10.501, 10.09, float('nan')):
      self.setUp()
      self.assertTrue(self.skip())
      self.assertFalse(self.guard.can_hold(now, trusted=True))
      self.assertFalse(self.guard.can_hold(10.13, trusted=True))

  def test_actual_calibration_or_other_input_failure_revokes_hold(self):
    self.assertTrue(self.skip())
    self.assertFalse(self.guard.can_hold(10.13, trusted=False))
    self.assertFalse(self.guard.can_hold(10.14, trusted=True))

  def test_untrusted_previous_input_cannot_enable_hold(self):
    self.guard.update(True, 101, 10.05, 10.06, trusted=False, now=10.07)
    self.assertFalse(self.skip(frame_id=103, timestamp_eof=10.15, log_time=10.16, now=10.17))

  def test_untrusted_current_input_cannot_enable_hold(self):
    self.assertFalse(self.skip(trusted=False))

  def test_old_pose_or_large_skip_cannot_enable_hold(self):
    for changes in (dict(now=10.6), dict(frame_id=112), dict(log_time=10.7),
                    dict(timestamp_eof=float('nan'))):
      self.setUp()
      self.assertFalse(self.skip(**changes))

  def test_startup_and_recovery_do_not_hold(self):
    self.guard = module.CalibrationOdometryValidity()
    self.assertFalse(self.skip())
    self.setUp()
    self.assertTrue(self.skip())
    self.assertFalse(self.guard.update(True, 103, 10.15, 10.16, trusted=True, now=10.17))


class TestLocationCameraOdometryValidity(unittest.TestCase):
  def setUp(self):
    self.guard = module.LocationCameraOdometryValidity()
    self.assertTrue(self.guard.update(True, 100, 10., 10.01, True, allow_grace=True))

  def test_model_skip_can_bridge_point2_seconds(self):
    self.assertTrue(self.guard.update(False, 104, 10.2, 10.21, False, allow_grace=True))
    self.assertTrue(self.guard.holding)

  def test_consecutive_skips_stop_at_half_second_not_half_second_per_sample(self):
    for frame, elapsed in ((102,.1),(104,.2),(106,.3),(108,.4),(110,.5)):
      self.assertTrue(self.guard.update(False, frame, 10.+elapsed, 10.01+elapsed,
                                       False, allow_grace=True))
    self.assertFalse(self.guard.update(False, 111, 10.55, 10.56, False, allow_grace=True))
    self.assertFalse(self.guard.update(False, 112, 10.6, 10.61, False, allow_grace=True))

  def test_recovery_needs_an_accepted_good_frame(self):
    self.guard.update(False, 110, 10.5, 10.51, False, allow_grace=True)
    self.assertFalse(self.guard.update(False, 111, 10.55, 10.56, False, allow_grace=True))
    self.assertTrue(self.guard.update(True, 112, 10.6, 10.61, True, allow_grace=True))
    self.assertFalse(self.guard.holding)
    self.assertTrue(self.guard.update(False, 114, 10.7, 10.71, False, allow_grace=True))

  def test_no_grace_without_history_or_observed_frame_skip(self):
    h=module.LocationCameraOdometryValidity()
    self.assertFalse(h.update(False, 104, 10.2, 10.21, False, allow_grace=True))
    self.assertFalse(self.guard.update(False, 101, 10.05, 10.06, False, allow_grace=True))
    self.assertFalse(self.guard.update(False, 103, 10.15, 10.16, False, allow_grace=True))

  def test_unhealthy_sensor_filter_or_other_input_context_revokes_grace(self):
    self.assertFalse(self.guard.update(False, 104, 10.2, 10.21, False, allow_grace=False))
    self.assertFalse(self.guard.update(False, 106, 10.3, 10.31, False, allow_grace=True))

  def test_estimator_reject_does_not_seed_grace(self):
    self.assertTrue(self.guard.update(True, 101, 10.05, 10.06, False, allow_grace=True))
    self.assertFalse(self.guard.update(False, 103, 10.15, 10.16, False, allow_grace=True))

  def test_stale_nonfinite_duplicate_reversed_or_large_jump_rejected(self):
    for frame,eof,stamp in ((102,10.51,10.52),(120,10.1,10.11),(99,10.1,10.11),
                            (102,9.9,10.1),(102,10.1,9.9),(102,float('nan'),10.1),
                            (102,10.1,10.7),(102,10.1,10.09)):
      self.setUp()
      self.assertFalse(self.guard.update(False, frame, eof, stamp, False, allow_grace=True))
    self.setUp()
    self.assertTrue(self.guard.update(False, 104, 10.2, 10.21, False, allow_grace=True))
    self.assertFalse(self.guard.update(False, 104, 10.2, 10.21, False, allow_grace=True))

  def test_trusted_calibration_bridges_test13_five_missing_frames(self):
    calibration = module.CalibrationOdometryValidity()
    calibration.update(True, 100, 10., 10.01, trusted=True, now=10.02)
    self.assertTrue(calibration.update(False, 106, 10.3, 10.36, trusted=True, now=10.37))
    self.assertTrue(calibration.can_hold(10.49, trusted=True))
    self.assertFalse(calibration.can_hold(10.501, trusted=True))
    self.assertEqual(calibration.camera.MAX_GAP,.5)

  def test_real_locationd_publication_checks_all_other_inputs(self):
    import ast
    from types import SimpleNamespace as NS
    from unittest.mock import MagicMock
    source=Path(__file__).resolve().parents[1]/'locationd.py'
    tree=ast.parse(source.read_text(encoding='utf8'))
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    loop=next(n for n in main.body if isinstance(n,ast.While))
    block=next(n for n in loop.body if isinstance(n,ast.If) and
               ast.unparse(n.test)=="sm.updated['cameraOdometry']")
    class SM(dict):
      valid={'cameraOdometry':False,'carState':True,'extrinsicsCalibration':True}
      updated={'cameraOdometry':True}
      logMonoTime={'cameraOdometry':int(10.21e9)}
    sm=SM(cameraOdometry=NS(frameId=104,timestampEof=int(10.2e9)))
    services=['accelerometer','gyroscope','cameraOdometry']
    env=dict(sm=sm,critical_services=services,observation_input_invalid=dict.fromkeys(services,0),
             input_invalid_threshold=dict.fromkeys(services,1.5),camera_validity=self.guard,
             filter_initialized=True,camera_accepted=False,acc_msgs=[],gyro_msgs=[],sensor_valid={},
             sensor_recv_time={},sensor_alive={},SIMULATION=False,sensor_all_checks=lambda *a:True,
             estimator=MagicMock(),cloudlog=MagicMock(),pm=MagicMock(),
             last_input_health=None,time=NS(monotonic=lambda:10.22))
    code=compile(ast.Module(body=[block],type_ignores=[]),'actual_locationd_publication','exec')
    exec(code,env)
    self.assertEqual(env['estimator'].get_msg.call_args.args,(True,True,True))
    for bad in ('sensor','other','counter','uninitialized'):
      self.setUp();env['camera_validity']=self.guard
      env['filter_initialized']=bad!='uninitialized'
      env['sensor_all_checks']=lambda *a, ok=bad!='sensor':ok
      sm.valid['carState']=bad!='other'
      env['observation_input_invalid']['gyroscope']=2 if bad=='counter' else 0
      exec(code,env)
      self.assertFalse(env['estimator'].get_msg.call_args.args[1],bad)
