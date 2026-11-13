import ast
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from opendbc.car.hyundai.blinker_test import BlinkerTestSender, read_json, write_json
from openpilot.selfdrive.controls.lib.overtake_signal_test import OvertakeSignalTest


class TestOvertakeSignalTest(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    root = Path(self.tmp.name)
    self.test = OvertakeSignalTest(root/'request', root/'status', root/'cancel')
    self.args = dict(enabled=True, supported=True, eligible=True, candidate=True, lead_id=63, lamps_off=True)

  def update(self, now, **changes):
    return self.test.update(now, **(self.args | changes))

  def test_disabled_and_missing_capability_never_write_request(self):
    for changes in (dict(enabled=False), dict(supported=False), dict(eligible=False), dict(candidate=False)):
      self.assertFalse(self.update(100., **changes))
      self.assertFalse(self.test.request.exists())

  def test_bounded_left_request_and_no_start_sound_without_feedback(self):
    self.assertFalse(self.update(100.))
    value = read_json(self.test.request)
    self.assertEqual(value['action'], 'left')
    self.assertEqual(value['deadline'], 103.)
    self.assertAlmostEqual(value['expires'], 100.3)
    self.assertFalse(self.update(100.1))

  def test_can_feedback_announces_exactly_once(self):
    self.update(100.)
    write_json(self.test.status, dict(id=self.test.identity, time=100.1, reason='ok', lamp_confirmed=True))
    self.assertTrue(self.update(100.1, lamps_off=False, candidate=False))
    self.assertFalse(self.update(100.15, lamps_off=False, candidate=False))

  def test_wrong_or_stale_feedback_cannot_announce(self):
    self.update(100.)
    for stamp, identity in ((100., 'other'), (99., self.test.identity)):
      write_json(self.test.status, dict(id=identity, time=stamp, reason='ok', lamp_confirmed=True))
      self.assertFalse(self.update(100.1))

  def test_timeout_no_automatic_repeat_for_same_lead(self):
    self.update(100.)
    for i in range(1, 32): self.update(100.+i*.1)
    self.assertIsNone(self.test.identity)
    self.assertEqual(read_json(self.test.request)['action'], 'off')
    for i in range(1, 700): self.update(103.1+i*.1)
    self.assertIsNone(self.test.identity)

  def test_driver_guard_and_toggle_off_cancel_request(self):
    self.update(100.)
    self.update(100.1, eligible=False)
    self.assertEqual(read_json(self.test.request)['action'], 'off')
    self.assertIsNone(self.test.identity)
    self.update(100.2, enabled=False)
    self.assertIsNone(self.test.identity)

  def test_observation_gap_cancels_instead_of_extending(self):
    self.update(100.)
    self.update(100.4)
    self.assertEqual(read_json(self.test.request)['action'], 'off')

  def test_explicit_cancel_and_sender_rejection_are_terminal(self):
    self.update(100.)
    write_json(self.test.cancel, dict(id=self.test.identity))
    self.update(100.1)
    self.assertEqual(self.test.reason, 'cancelled')
    self.test.tested_lead = -1; self.test.cooldown = 0
    self.update(100.2)
    write_json(self.test.status, dict(id=self.test.identity, time=100.3, reason='driver_stalk'))
    self.update(100.3)
    self.assertEqual(self.test.reason, 'sender_guard')

  def test_existing_sender_accepts_request_but_expires_on_heartbeat_loss(self):
    self.update(100.)
    sender = BlinkerTestSender(self.test.request, self.test.status, self.test.cancel,
                              allow_moving=True, moving_road_test=True)
    cs = NS(canValid=True, gearShifter='drive', standstill=False, vEgo=27.,
            doorOpen=False, seatbeltUnlatched=False, blinkerStalkSupported=True, blinkerStalkValid=True,
            leftBlinkerStalk=False, rightBlinkerStalk=False, leftBlinker=False, rightBlinker=False)
    cc = NS(enabled=True, latActive=True, longActive=True)
    self.assertEqual(sender.update(100., cs, cc, True), 0)
    self.assertEqual(sender.update(100.02, cs, cc, True), 3)
    self.assertEqual(sender.update(100.4, cs, cc, True), 0)
    self.assertEqual(sender.reason, 'expired')

  def test_synthetic_lamp_does_not_become_driver_lane_change_request(self):
    path = Path(__file__).parents[1]/'lib/desire_helper.py'
    tree = ast.parse(path.read_text(encoding='utf8'))
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_update_driver_blinker')
    env = dict(BLINKER_LEFT=1, BLINKER_RIGHT=2)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    helper = NS(driver_blinker_state=0, laneChangeNeedTorque=0)
    cs = NS(canValid=True, blinkerStalkSupported=True, blinkerStalkValid=True,
            leftBlinkerStalk=False, rightBlinkerStalk=False, leftBlinker=True, rightBlinker=False)
    self.assertEqual(env['_update_driver_blinker'](helper, cs), (0, False, False))


class TestProductionSignalGuard(unittest.TestCase):
  def test_driver_and_stale_guards_and_io_failure(self):
    path = Path(__file__).parents[1]/'plannerd.py'
    method = next(n for n in ast.parse(path.read_text(encoding='utf8')).body
                  if isinstance(n, ast.FunctionDef) and n.name == 'update_overtake_signal_test')
    env = dict(cloudlog=NS(event=lambda *a, **kw: None))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    class SM(dict):
      def all_checks(self, services): return True
    cs = NS(vEgo=27., canValid=True, gearShifter='drive', doorOpen=False, seatbeltUnlatched=False,
            blinkerStalkSupported=True, blinkerStalkValid=True, leftBlinkerStalk=False, rightBlinkerStalk=False,
            steeringPressed=False, gasPressed=False, brakePressed=False, leftLaneLine=10, leftBlindspot=False,
            leftBlinker=False, rightBlinker=False)
    sm = SM(carState=cs, carControl=NS(latActive=True, longActive=True),
            carrotMan=NS(naviHighway=True, xTurnInfo=-1, xDistToTurn=10000.),
            modelV2=NS(meta=NS(laneChangeState='off')), radarState=NS(leadOne=NS(radarTrackId=63)))
    sm.logMonoTime = {k:100_000_000_000 for k in sm}
    class Probe:
      reason='waiting'; identity=None; announced=False
      def update(self, now, **kwargs): self.arguments=kwargs; return False
    probe = Probe(); run=env['update_overtake_signal_test']
    run(sm, probe, True, True, True)
    self.assertTrue(probe.arguments['eligible'])
    for field in ('steeringPressed', 'gasPressed', 'brakePressed', 'leftBlinkerStalk', 'leftBlindspot'):
      setattr(cs, field, True); run(sm, probe, True, True, True)
      self.assertFalse(probe.arguments['eligible']); setattr(cs, field, False)
    sm.logMonoTime['carState'] -= 400_000_000
    run(sm, probe, True, True, True); self.assertFalse(probe.arguments['eligible'])
    def fail(*a, **kw): raise OSError('unavailable')
    probe.update=fail
    self.assertFalse(run(sm, probe, True, True, True))
    self.assertEqual(probe.reason, 'io_error')
