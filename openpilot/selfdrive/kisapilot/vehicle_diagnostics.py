"""Passive C4 telemetry only. No diagnostic CAN requests or actuator commands."""
import math
import threading
import time

SERVICES = ('carState', 'carControl', 'selfdriveState', 'pandaStates', 'deviceState')
PROFILE = dict(name='2024 더 뉴 아이오닉 5', battery_kwh=84, drivetrain='RWD', motors=1,
               source='user_selected', battery_pid_verified=False)


def number(value):
  try:
    result = float(value)
    return result if math.isfinite(result) else None
  except (ValueError, TypeError):
    return None


def fields(obj, names):
  result = {}
  for name in names:
    value = getattr(obj, name, None)
    result[name] = value if isinstance(value, (bool, int, str)) or value is None else number(value)
  return result


def snapshot(sm, now, detected=None):
  health = {}
  for name in SERVICES:
    stamp = number(sm.recv_time.get(name))
    age = None if stamp is None or stamp <= 0 else now-stamp
    fresh = bool(sm.valid.get(name) and sm.alive.get(name) and age is not None and 0 <= age < (2.5 if name == 'pandaStates' else 1.))
    health[name] = dict(fresh=fresh, valid=bool(sm.valid.get(name)), alive=bool(sm.alive.get(name)),
                        age_seconds=round(age, 3) if age is not None and age >= 0 else None)
  result = dict(protocol=1, read_only=True, ecu_requests_sent=False, profile=dict(PROFILE),
                detected=detected or {}, streams=health, vehicle=None, control=None, pandas=[], device=None,
                dtc=dict(status='not_queried', source='OBD adapter required'),
                battery=dict(status='not_available', reason='84kWh BMS payload requires vehicle verification'))
  if health['carState']['fresh']:
    cs = sm['carState']
    vehicle = fields(cs, ('standstill', 'canValid', 'canTimeout', 'canErrorCounter', 'brakePressed',
                         'gasPressed', 'steeringAngleDeg', 'steerFaultTemporary', 'steerFaultPermanent',
                         'accFaulted', 'espDisabled', 'doorOpen', 'seatbeltUnlatched'))
    speed = number(getattr(cs, 'vEgo', None))
    vehicle.update(speed_kph=round(speed*3.6, 2) if speed is not None and speed >= 0 else None,
                   gear=str(cs.gearShifter) if hasattr(cs, 'gearShifter') else None)
    result['vehicle'] = vehicle
  if health['carControl']['fresh']:
    result['control'] = fields(sm['carControl'], ('enabled', 'latActive', 'longActive'))
  if health['selfdriveState']['fresh']:
    result['selfdrive'] = fields(sm['selfdriveState'], ('enabled', 'active'))
  if health['pandaStates']['fresh']:
    for panda in sm['pandaStates']:
      p = fields(panda, ('controlsAllowed', 'safetyTxBlocked', 'safetyRxChecksInvalid', 'heartbeatLost'))
      p.update(type=str(panda.pandaType), faults=[str(f) for f in panda.faults],
               supply_voltage_v=(number(panda.voltage)/1000 if number(panda.voltage) is not None and panda.voltage > 0 else None),
               voltage_source='panda_supply_not_bms', buses=[])
      for index in range(3):
        bus = getattr(panda, f'canState{index}', None)
        if bus is not None:
          p['buses'].append(dict(index=index, **fields(bus, ('busOff', 'errorWarning', 'errorPassive',
                                  'receiveErrorCnt', 'transmitErrorCnt', 'totalErrorCnt', 'canfdEnabled'))))
      result['pandas'].append(p)
  if health['deviceState']['fresh']:
    device = sm['deviceState']
    result['device'] = fields(device, ('started', 'freeSpacePercent', 'memoryUsagePercent'))
  return result


class VehicleDiagnostics:
  def __init__(self):
    from openpilot.cereal import messaging
    self.sm = messaging.SubMaster(list(SERVICES))
    self.lock = threading.Lock()

  def read(self, params):
    with self.lock:
      self.sm.update(0)
      detected = {}
      try:
        from opendbc.car.structs import CarParams
        from opendbc.car.hyundai.values import CAR
        raw = params.get('CarParams') if params is not None else None
        if raw:
          with CarParams.from_bytes(raw) as cp:
            detected = dict(fingerprint=cp.carFingerprint, brand=cp.brand,
                            expected_model=cp.carFingerprint == CAR.HYUNDAI_IONIQ_5_PE,
                            firmware_ecu_count=len(cp.carFw))
      except Exception:
        detected = dict(status='unavailable')
      return snapshot(self.sm, time.monotonic(), detected)
