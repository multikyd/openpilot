"""Bounded read-only steering telemetry; never produces CAN commands."""
from collections import deque
import math


def _number(value):
  try:
    value = float(value)
    return round(value, 6) if math.isfinite(value) else None
  except (TypeError, ValueError, OverflowError):
    return None


class SteeringDiagnostics:
  SAMPLE_NS = 100_000_000
  PRE_NS = 5_000_000_000
  POST_NS = 2_000_000_000
  COOLDOWN_NS = 5_000_000_000
  ADDRESSES = frozenset((0x50, 0xCB, 0xEA, 0x110, 0x12A))
  MDPS_KEYS = ('COUNTER', 'LKA_ACTIVE', 'LFA2_ACTIVE', 'LKA_FAULT', 'LFA2_FAULT',
               'STEERING_ANGLE', 'STEERING_ANGLE_2', 'STEERING_COL_TORQUE', 'STEERING_OUT_TORQUE')

  def __init__(self):
    self.history = deque(maxlen=64)
    self.next_sample_ns = 0
    self.last_time_ns = None
    self.last_faults = (False, False, False, False)
    self.last_limit = False
    self.last_emitted = {}
    self.pending = deque(maxlen=4)
    self.sequence = 0

  def update(self, *, now_nanos, requested_deg, applied_deg, maximum_deg,
             speed_mps, actual_deg, driver_torque, eps_torque, driver_pressed,
             lat_active, request_active, temporary_fault, permanent_fault,
             can_valid, can_timeout, mdps_original, can_sends):
    now = int(now_nanos)
    if self.last_time_ns is not None and now < self.last_time_ns:
      self.__init__()
    self.last_time_ns = now
    requested, applied, maximum, actual = map(_number, (requested_deg, applied_deg, maximum_deg, actual_deg))
    mdps = {key: _number(mdps_original[key]) for key in self.MDPS_KEYS if key in mdps_original}
    faults = (bool(temporary_fault), bool(permanent_fault),
              bool(mdps.get('LKA_FAULT')), bool(mdps.get('LFA2_FAULT')))
    target_exceeds = bool(lat_active and requested is not None and maximum is not None and abs(requested) > maximum + .1)
    at_limit = bool(lat_active and applied is not None and maximum is not None and abs(applied) >= maximum - .1)
    fault_changed = faults != self.last_faults
    limit_changed = target_exceeds != self.last_limit
    events = []
    if now >= self.next_sample_ns or fault_changed or limit_changed:
      tx = []
      for address, payload, bus in can_sends:
        if address in self.ADDRESSES and len(tx) < 8:
          tx.append(dict(address=int(address), bus=int(bus), data_hex=bytes(payload)[:64].hex()))
      self.history.append(dict(
        mono_time_ns=now, requested_deg=requested, applied_deg=applied, maximum_deg=maximum,
        actual_deg=actual,
        tracking_error_deg=round(applied - actual, 6) if applied is not None and actual is not None else None,
        speed_mps=_number(speed_mps), driver_torque_native=_number(driver_torque),
        eps_torque_native=_number(eps_torque), driver_pressed=bool(driver_pressed),
        lat_active=bool(lat_active), request_active=bool(request_active),
        temporary_fault=bool(temporary_fault), permanent_fault=bool(permanent_fault),
        can_valid=bool(can_valid), can_timeout=bool(can_timeout),
        target_exceeds_angle_limit=target_exceeds, output_at_angle_limit=at_limit,
        mdps_original=mdps, generated_can=tx))
      self.next_sample_ns = now + self.SAMPLE_NS
    while self.history and self.history[0]['mono_time_ns'] < now - self.PRE_NS:
      self.history.popleft()
    reasons = []
    if fault_changed:
      reasons.append('eps_fault_set' if any(faults) else 'eps_fault_cleared')
    if limit_changed:
      reasons.append('angle_target_exceeds_limit' if target_exceeds else 'angle_target_returns_within_limit')
    for reason in reasons:
      previous = self.last_emitted.get(reason)
      if previous is not None and now - previous < self.COOLDOWN_NS:
        continue
      self.last_emitted[reason] = now
      self.sequence += 1
      events.append(dict(schema_version=1, diagnostic_id=self.sequence, reason=reason,
                         trigger_mono_time_ns=now, phase='trigger', samples=list(self.history)))
      self.pending.append((now + self.POST_NS, self.sequence, reason, now))
    while self.pending and now >= self.pending[0][0]:
      _, identifier, reason, trigger = self.pending.popleft()
      events.append(dict(schema_version=1, diagnostic_id=identifier, reason=reason,
                         trigger_mono_time_ns=trigger, phase='after',
                         samples=[s for s in self.history if trigger <= s['mono_time_ns'] <= trigger + self.POST_NS]))
    self.last_faults = faults
    self.last_limit = target_exceeds
    return events
