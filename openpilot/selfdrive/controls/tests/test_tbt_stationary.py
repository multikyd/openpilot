"""Exercise TBT requests and the real SPAS sender without native car dependencies."""
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]


def load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


sender = load('stationary_sender', ROOT/'opendbc_repo/opendbc/car/hyundai/blinker_test.py')
with patch.dict(sys.modules, {'opendbc.car.hyundai.blinker_test': sender}):
  tbt = load('stationary_tbt', ROOT/'openpilot/selfdrive/carrot/tbt_blinker.py')


class TestStationaryTbt(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.path = Path(self.temp.name)/'request.json'
    self.cancel = Path(self.temp.name)/'cancel.json'
    self.cs = NS(canValid=True, blinkerStalkSupported=True, blinkerStalkValid=True,
                 gearShifter='drive', standstill=True, vEgo=0., gasPressed=False,
                 brakePressed=False, steeringAngleDeg=0., leftBlinker=False, rightBlinker=False,
                 leftBlinkerStalk=False, rightBlinkerStalk=False, doorOpen=False, seatbeltUnlatched=False)
    self.cc = NS(longActive=False, latActive=False, enabled=False)
    self.requester = tbt.TbtBlinker(self.path, self.cancel)

  def request(self, distance=40., width=5., now=100., code=13):
    self.requester.update(now, code, distance, .1, self.cs, True, next_road_width=width)
    return sender.read_json(self.path)

  def test_scc_stop_without_driver_brake_or_op_long_requests_and_sends(self):
    req = self.request()
    self.assertEqual(req['action'], 'right')
    self.assertTrue(req['stationary'])
    self.assertEqual(req['deadline'], 220.)
    tx = sender.BlinkerTestSender(self.path, Path(self.temp.name)/'status.json', self.cancel, tbt=True)
    self.assertEqual(tx.update(100., self.cs, self.cc, True), 0)
    self.assertEqual(tx.update(100.02, self.cs, self.cc, True), 4)
    self.assertEqual(tx.reason, 'ok')

  def test_stopped_distance_doubles_to_40_60_meters(self):
    for width, boundary in ((0.,40.),(5.,40.),(7.5,50.),(10.,60.),(20.,60.)):
      self.path.unlink(missing_ok=True)
      self.requester = tbt.TbtBlinker(self.path, self.cancel)
      self.assertEqual(self.request(boundary, width)['action'], 'right')
      self.path.unlink()
      self.requester = tbt.TbtBlinker(self.path, self.cancel)
      self.assertEqual(self.request(boundary+.01, width), {})

  def test_gas_stale_nav_gear_and_manual_stalk_still_block(self):
    for field, value in (('gasPressed',True),('gearShifter','neutral'),('leftBlinkerStalk',True),
                         ('canValid',False),('blinkerStalkValid',False),('vEgo',.06)):
      self.path.unlink(missing_ok=True)
      self.requester = tbt.TbtBlinker(self.path, self.cancel)
      old = getattr(self.cs,field)
      setattr(self.cs,field,value)
      self.assertEqual(self.request(), {}, field)
      setattr(self.cs,field,old)
    self.requester.update(100., 13, 30., 2.01, self.cs, True)
    self.assertEqual(sender.read_json(self.path), {})

  def test_wait_refreshes_past_ten_seconds_without_brake_and_expires_at_two_minutes(self):
    self.request()
    self.assertEqual(self.request(now=111.)['action'], 'right')
    req = self.request(now=220.)
    self.assertEqual(req['action'], 'off')

  def test_door_belt_and_gas_still_interrupt_sender(self):
    self.request()
    for field in ('doorOpen', 'seatbeltUnlatched', 'gasPressed'):
      tx = sender.BlinkerTestSender(self.path, Path(self.temp.name)/'status.json', self.cancel, tbt=True)
      self.assertEqual(tx.update(100., self.cs, self.cc, True), 0)
      self.assertEqual(tx.update(100.02, self.cs, self.cc, True), 4)
      setattr(self.cs,field,True)
      self.assertEqual(tx.update(100.04, self.cs, self.cc, True), 0)
      setattr(self.cs,field,False)


if __name__ == '__main__':
  unittest.main()
