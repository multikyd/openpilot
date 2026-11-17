import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from openpilot.selfdrive.kisapilot.vehicle_diagnostics import SERVICES, snapshot


class Sample:
  def __init__(self):
    self.valid = dict.fromkeys(SERVICES, True)
    self.alive = dict.fromkeys(SERVICES, True)
    self.recv_time = dict.fromkeys(SERVICES, 99.8)
    self.data = dict(carState=NS(vEgo=10., gearShifter='park', canValid=True, canTimeout=False),
                     carControl=NS(enabled=False, latActive=False, longActive=False),
                     selfdriveState=NS(enabled=False, active=False), pandaStates=[], deviceState=NS(started=True))
  def __getitem__(self, name):
    return self.data[name]


class TestVehicleDiagnostics(unittest.TestCase):
  def test_live_car_values_have_units_and_missing_values_stay_unknown(self):
    r = snapshot(Sample(), 100.)
    self.assertEqual(r['vehicle']['speed_kph'], 36.)
    self.assertIsNone(r['vehicle']['brakePressed'])
    self.assertTrue(r['read_only'])
    self.assertFalse(r['ecu_requests_sent'])
    self.assertEqual(r['dtc']['status'], 'not_queried')

  def test_invalid_dead_or_stale_car_does_not_show_defaults_as_healthy(self):
    for condition in ('valid', 'alive', 'stale', 'future', 'missing'):
      sm = Sample()
      if condition in ('valid', 'alive'):
        getattr(sm, condition)['carState'] = False
      else:
        sm.recv_time['carState'] = dict(stale=90., future=101., missing=0.)[condition]
      self.assertIsNone(snapshot(sm, 100.)['vehicle'])

  def test_bad_numeric_values_are_not_serialized_as_nan(self):
    sm = Sample();sm.data['carState'].vEgo = math.nan
    self.assertIsNone(snapshot(sm, 100.)['vehicle']['speed_kph'])

  def test_panda_voltage_is_explicitly_not_bms(self):
    sm = Sample();sm.data['pandaStates'] = [NS(pandaType='redPanda', faults=['relayMalfunction'], voltage=12500,
      canState0=NS(busOff=True, totalErrorCnt=2))]
    p = snapshot(sm, 100.)['pandas'][0]
    self.assertEqual(p['supply_voltage_v'], 12.5)
    self.assertEqual(p['voltage_source'], 'panda_supply_not_bms')
    self.assertTrue(p['buses'][0]['busOff'])
    sm.recv_time['pandaStates'] = 90.
    self.assertEqual(snapshot(sm, 100.)['pandas'], [])

  def test_profile_is_user_selected_and_battery_decoding_is_unverified(self):
    r = snapshot(Sample(), 100.)
    self.assertEqual(r['profile']['battery_kwh'], 84)
    self.assertEqual(r['profile']['drivetrain'], 'RWD')
    self.assertFalse(r['profile']['battery_pid_verified'])
    self.assertEqual(r['battery']['status'], 'not_available')

  def test_api_is_get_only_and_does_not_call_actuator_or_obd_test(self):
    path = Path(__file__).resolve().parents[1] / 'kisa_agent.py'
    tree = ast.parse(path.read_text(encoding='utf-8-sig'))
    route = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'vehicle_diagnostics')
    decorator = route.decorator_list[0]
    self.assertEqual(decorator.args[0].value, '/diagnostics/vehicle')
    self.assertEqual([e.value for e in decorator.keywords[0].value.elts], ['GET'])
    self.assertNotIn('vehicle_test_preflight', ast.unparse(route))
    self.assertNotIn('sendcan', ast.unparse(route))
