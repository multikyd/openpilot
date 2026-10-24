import ast
import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest

ROOT = Path(__file__).parents[4]
HYUNDAI = ROOT / 'opendbc_repo/opendbc/car/hyundai'
spec = importlib.util.spec_from_file_location('hyundai_angle_guard_under_test', HYUNDAI / 'angle_guard.py')
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)
Guard = mod.AngleSteeringGuard


class TestAngleGuard(unittest.TestCase):
  def setUp(self):
    self.guard = Guard()
    self.time = 0

  def tick(self, requested=20., candidate=20., actual=19., active=True, fault=False):
    self.time += 10_000_000
    return self.guard.update(self.time, requested, candidate, actual, active, fault)

  def test_normal_range_does_not_warn_or_change_settled_angle(self):
    for _ in range(100):
      out = self.tick()
    self.assertEqual((out.state, out.angle, out.active, out.torque_scale), (0, 20., True, 1.))

  def test_high_command_is_not_activated(self):
    out = self.tick(candidate=199., actual=126.)
    self.assertEqual((out.state, out.active, out.torque_scale, out.angle), (2, False, 0., 126.))

  def test_both_steering_directions_are_guarded(self):
    for sign in (-1, 1):
      out = Guard().update(10_000_000, sign*255., sign*199., sign*126., True, False)
      self.assertFalse(out.active)

  def test_high_measured_angle_cannot_reactivate_assist(self):
    self.assertFalse(self.tick(requested=300., candidate=170., actual=300.).active)

  def test_limiting_warns_before_unavailable(self):
    out = self.tick(requested=200., candidate=175., actual=120.)
    self.assertEqual(out.state, 1)
    self.assertTrue(out.active)
    self.assertLess(out.torque_scale, 1.)
    for _ in range(45):
      out = self.tick(requested=200., candidate=175., actual=120.)
    self.assertEqual(out.state, 2)
    self.assertFalse(out.active)

  def test_large_but_well_tracked_angle_warns_without_repeated_handover(self):
    for _ in range(100):
      out = self.tick(requested=175., candidate=175., actual=174.)
    self.assertEqual(out.state, 1)
    self.assertTrue(out.active)
    self.assertEqual(out.torque_scale, 1.)

  def test_moderate_high_angle_lag_keeps_steering_authority(self):
    # Ordinary lag must not create torque reduction -> more lag -> handover.
    for sign in (-1, 1):
      for candidate, actual in ((175., 150.), (180., 158.), (188., 150.), (189., 140.)):
        with self.subTest(sign=sign, candidate=candidate, actual=actual):
          guard = Guard()
          for i in range(100):
            out = guard.update((i+1)*10_000_000, sign*candidate, sign*candidate,
                               sign*actual, True, False)
          self.assertEqual((out.active, out.state, out.torque_scale), (True, 1, 1.))
          self.assertEqual(out.angle, sign*candidate)

  def test_severe_tracking_error_still_reduces_torque_and_hands_over(self):
    for sign in (-1, 1):
      guard = Guard()
      for i in range(60):
        guard.update((i+1)*10_000_000, sign*150., sign*150., sign*149., True, False)
      first = guard.update(610_000_000, sign*188., sign*188., sign*138., True, False)
      self.assertTrue(first.active)
      self.assertLess(first.torque_scale, 1.)
      for i in range(62, 110):
        out = guard.update(i*10_000_000, sign*188., sign*188., sign*138., True, False)
      self.assertEqual((out.active, out.state, out.torque_scale), (False, 2, 0.))

  def test_absolute_envelope_is_unchanged_even_with_small_tracking_error(self):
    for sign in (-1, 1):
      for candidate, actual in ((190., 189.), (189., 190.)):
        with self.subTest(sign=sign, candidate=candidate, actual=actual):
          out = Guard().update(10_000_000, sign*candidate, sign*candidate,
                               sign*actual, True, False)
          self.assertEqual((out.active, out.state, out.torque_scale), (False, 2, 0.))
          self.assertEqual(out.reason, 'command_envelope')

  def test_short_tracking_error_does_not_latch(self):
    for _ in range(10):
      self.tick(requested=155., candidate=150., actual=90.)
    out = self.tick()
    self.assertEqual(out.state, 0)
    self.assertTrue(out.active)

  def test_tracking_error_at_small_angle_is_not_a_false_high_angle_fault(self):
    for _ in range(100):
      out = self.tick(requested=60., candidate=60., actual=0.)
    self.assertEqual(out.state, 0)

  def test_existing_eps_fault_latches_and_does_not_flap_on_clear(self):
    self.tick(actual=160., fault=True)
    for _ in range(100):
      out = self.tick(requested=170., candidate=170., actual=160.)
    self.assertEqual(out.state, 2)
    self.assertFalse(out.active)

  def test_resume_requires_stable_safe_input_and_starts_at_measured_angle(self):
    self.tick(candidate=199., actual=126.)
    for _ in range(45):
      out = self.tick(requested=40., candidate=40., actual=39.)
      self.assertFalse(out.active)
    resumed = None
    for _ in range(10):
      out = self.tick(requested=40., candidate=40., actual=39.)
      if out.active:
        resumed = out
        break
    self.assertIsNotNone(resumed)
    self.assertEqual(resumed.angle, 39.)
    self.assertLessEqual(resumed.torque_scale, .03)

  def test_inactive_request_has_no_torque_and_tracks_wheel(self):
    out = self.tick(actual=120., active=False)
    self.assertEqual((out.active, out.angle, out.torque_scale), (False, 120., 0.))

  def test_invalid_input_is_not_sent_as_active_angle(self):
    for value in (float('nan'), float('inf')):
      self.assertFalse(Guard().update(10_000_000, 20., value, 19., True, False).active)

  def test_time_jump_does_not_count_as_elapsed_risk_or_resume(self):
    self.tick()
    self.time += 10_000_000_000
    out = self.tick()
    self.assertEqual(out.state, 0)
    self.assertLess(out.torque_scale, .11)


class TestActualCanBuilders(unittest.TestCase):
  def setUp(self):
    tree = ast.parse((HYUNDAI / 'hyundaicanfd.py').read_text(encoding='utf-8'))
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in
                 ('create_steering_messages_camera_scc', 'create_steering_messages')]
    self.env = dict(copy=copy, HyundaiFlags=N(CANFD_LKA_STEERING=1, CANFD_LKA_STEERING_ALT=2, CAMERA_SCC=N(value=4)))
    exec(compile(ast.Module(body=functions, type_ignores=[]), 'actual_can_builders', 'exec'), self.env)
    self.packer = N(make_can_msg=lambda name, bus, values: (name, bus, copy.deepcopy(values)))
    self.can = N(CAM=2, ECAN=0, ACAN=1)
    self.cs = N(adrv_info_161=None, mdps_info={}, lfa_alt_info={'LKAS_ANGLE_ACTIVE': 2, 'LKAS_ANGLE_CMD': 10},
                lfa_info={"STEER_REQ": 1}, steer_touch_info=None)

  def camera(self, active, angle_control=True):
    return self.env['create_steering_messages_camera_scc'](41, self.packer, N(), self.can,
            N(latActive=True), active, 10, self.cs, 123., 99, angle_control)

  def test_guarded_camera_angle_uses_request_not_cc_lat_active(self):
    values = next(v for name, _, v in self.camera(False) if name == 'LFA_ALT')
    self.assertEqual((values['LKAS_ANGLE_ACTIVE'], values['LKAS_ANGLE_MAX_TORQUE']), (1, 0))
    self.assertEqual(values['LKAS_ANGLE_CMD'], -123.)

  def test_active_camera_angle_retains_command(self):
    values = next(v for name, _, v in self.camera(True) if name == 'LFA_ALT')
    self.assertEqual((values['LKAS_ANGLE_ACTIVE'], values['LKAS_ANGLE_MAX_TORQUE']), (2, 99))

  def test_messages_are_not_dropped_during_guard(self):
    self.assertEqual([x[0] for x in self.camera(False)], ['MDPS', 'LFA_ALT', 'LFA'])

  def test_stock_emergency_angle_message_is_preserved(self):
    self.cs.adrv_info_161 = {'ALERTS_1': 11}
    self.cs.lfa_info = {'OEM': 42}
    values = next(v for name, _, v in self.camera(False) if name == 'LFA_ALT')
    self.assertEqual(values, self.cs.lfa_alt_info)

  def test_torque_control_request_packing_is_preserved(self):
    values = next(v for name, _, v in self.camera(False, False) if name == 'LFA')
    self.assertEqual((values['STEER_REQ'], values['TORQUE_REQUEST']), (0, 10))

  def test_non_camera_angle_also_disables_active_and_torque(self):
    messages = self.env['create_steering_messages'](self.packer, N(flags=0), self.can,
                                                  True, False, 10, 123., 99, True)
    values = messages[0][2]
    self.assertEqual((values['LKAS_ANGLE_ACTIVE'], values['LKAS_ANGLE_MAX_TORQUE']), (1, 0))


class TestGuardDriverEvents(unittest.TestCase):
  def test_guard_states_use_existing_warning_and_unavailable_events(self):
    source = ROOT / 'openpilot/selfdrive/car/car_events.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    block = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and
                 "self.CP.brand == 'hyundai'" == ast.unparse(n.test) and
                 'steeringAngleGuardState' in ast.unparse(n))
    for state, expected in ((0, []), (1, ['warning']), (2, ['unavailable'])):
      events = []
      env = dict(self=N(CP=N(brand='hyundai')), CS=N(steeringAngleGuardState=state),
                 events=N(add=events.append), EventName=N(steerSaturated='warning', steerTempUnavailable='unavailable'))
      exec(compile(ast.Module(body=[block], type_ignores=[]), str(source), 'exec'), env)
      self.assertEqual(events, expected)


if __name__ == '__main__':
  unittest.main()
