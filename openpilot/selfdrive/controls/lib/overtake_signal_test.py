"""Opt-in, single 3-second left lamp test. Never outputs steering or speed."""
import math
from uuid import uuid4

from opendbc.car.hyundai.blinker_test import CANCEL, MAX_SECONDS, OVERTAKE_TEST_REQUEST, STATUS, read_json, write_json


class OvertakeSignalTest:
  def __init__(self, request=OVERTAKE_TEST_REQUEST, status=STATUS, cancel=CANCEL):
    self.request, self.status, self.cancel = request, status, cancel
    self.identity = None
    self.start = self.written = self.cooldown = 0.
    self.previous = None
    self.lead_id = self.tested_lead = -1
    self.announced = False
    self.reason = 'disabled'

  def stop(self, now, reason):
    identity = self.identity
    self.identity = None
    self.reason = reason
    if identity is not None:
      self.cooldown = now+60.
      write_json(self.request, dict(id=identity, action='off', start=self.start, expires=now, deadline=now))

  def update(self, now, *, enabled, supported, eligible, candidate, lead_id, lamps_off):
    """Return a one-shot announcement only after CAN lamp feedback confirms."""
    if not math.isfinite(now):
      self.stop(self.previous or 0., 'invalid_time')
      return False
    gap = self.previous is not None and not 0. <= now-self.previous <= .25
    self.previous = now
    announce = False
    if not enabled or not supported or not eligible or gap:
      self.stop(now, 'disabled' if not enabled else 'capability_missing' if not supported else 'guard_or_stale')
      return False
    if self.identity is not None:
      if lead_id != self.lead_id or now-self.start >= MAX_SECONDS:
        self.stop(now, 'lead_changed' if lead_id != self.lead_id else 'completed')
        return False
      if read_json(self.cancel).get('id') == self.identity:
        self.stop(now, 'cancelled')
        return False
      status = read_json(self.status)
      stamp = status.get('time')
      if (status.get('id') == self.identity and type(stamp) in (int, float) and
          0. <= now-stamp <= .25):
        if status.get('reason') != 'ok':
          self.stop(now, 'sender_guard')
          return False
        if status.get('lamp_confirmed') and not self.announced:
          self.announced, announce = True, True
    elif candidate and lamps_off and lead_id >= 0 and lead_id != self.tested_lead and now >= self.cooldown:
      self.identity = 'overtake-signal-'+uuid4().hex
      self.lead_id = self.tested_lead = lead_id
      self.start = now
      self.written = -math.inf
      self.announced = False
    else:
      self.reason = 'waiting'
      return False
    deadline = self.start+MAX_SECONDS
    if now-self.written >= .1:
      write_json(self.request, dict(id=self.identity, action='left', start=self.start,
                                   expires=min(now+.3, deadline), deadline=deadline))
      self.written = now
    self.reason = 'lamp_confirmed' if self.announced else 'requested'
    return announce
