"""Short-lived SPAS2 requests for app tests and TBT."""
import json
import math
import os
from pathlib import Path

SAFETY_FLAG = 1024
TBT_SAFETY_FLAG = 2048
TBT_REQUEST = Path("/dev/shm/kisa_tbt_blinker_request.json")
TBT_MAX_SECONDS = 10.0
TBT_WAIT_MAX_SECONDS = 120.0
REQUEST = Path("/dev/shm/kisa_blinker_request.json")
STATUS = Path("/dev/shm/kisa_blinker_status.json")
CANCEL = Path("/dev/shm/kisa_blinker_cancel.json")
LEASE_SECONDS = 0.65
MAX_SECONDS = 3.0


def write_json(path, value):
  path = Path(path)
  temporary = path.with_name(path.name + ".tmp")
  temporary.write_text(json.dumps(value, allow_nan=False))
  os.replace(temporary, path)


def read_json(path):
  try:
    path = Path(path)
    if path.stat().st_size >= 8192:
      return {}
    value = json.loads(path.read_text())
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError, UnicodeError):
    return {}


def moving_speed_ok(cs):
  return not cs.standstill and abs(cs.vEgo) <= 40 / 3.6


def conditions(cs, *, allow_moving=False):
  if allow_moving and not cs.standstill:
    checks = {"can_valid": cs.canValid, "gear": str(cs.gearShifter) == "drive",
              "moving_speed": moving_speed_ok(cs), "doors_closed": not cs.doorOpen,
              "seatbelt": not cs.seatbeltUnlatched,
              "stalk_unavailable": cs.blinkerStalkSupported and cs.blinkerStalkValid,
              "driver_stalk": not (cs.leftBlinkerStalk or cs.rightBlinkerStalk)}
    return next((key for key, value in checks.items() if not value), "ok")
  checks = {"can_valid": bool(cs.canValid), "gear": str(cs.gearShifter) in ("park", "drive"),
            "stationary": bool(cs.standstill), "moving_speed": abs(cs.vEgo) <= 40 / 3.6,
            "doors_closed": not cs.doorOpen, "seatbelt": not cs.seatbeltUnlatched}
  return next((key for key, value in checks.items() if not value), "ok")


def active_status(now, status=STATUS, cancel=CANCEL):
  value = read_json(status)
  stamp = value.get("time")
  identity = value.get("id")
  if (type(stamp) not in (int, float) or not 0 <= now - stamp <= .25 or
      not isinstance(identity, str) or not identity or value.get("reason") != "ok" or
      read_json(cancel).get("id") == identity):
    return {}
  return value


def cancel_active(now):
  status = active_status(now)
  if not status:
    return False
  try:
    write_json(CANCEL, {"id": status["id"]})
    return True
  except OSError:
    return False


class BlinkerTestSender:
  def __init__(self, request=REQUEST, status=STATUS, cancel=CANCEL, *, tbt=False, allow_moving=False):
    self.tbt = tbt
    self.allow_moving = allow_moving
    self.max_seconds = TBT_WAIT_MAX_SECONDS if tbt else MAX_SECONDS
    self.request_path, self.status_path, self.cancel_path = request, status, cancel
    self.lamp_was_off = False
    self.confirmed = False
    self.blocked_id = None
    self.active_id = None
    self.active_action = None
    self.clear_frames = 0
    self.sent = 0
    self.reason = "idle"
    self.last_status = -1.0

  def has_request(self, now):
    request = read_json(self.request_path)
    expires = request.get("expires")
    return (request.get("action") in ("left", "right") and
            type(expires) in (int, float) and 0 < expires - now <= LEASE_SECONDS + .05)

  def update(self, now, cs, cc, supported, stock_spas_active=False):
    """Called at 50Hz. Return 0/3/4 to pack, or None for no test message."""
    req = read_json(self.request_path)
    identity = req.get("id")
    if self.tbt:
      waiting = (req.get("stationary") is True and cs.standstill and abs(cs.vEgo) < .05 and
                 not cs.gasPressed and (cs.brakePressed or cc.longActive))
      moving = moving_speed_ok(cs)
      permitted = (cs.canValid and cs.blinkerStalkSupported and cs.blinkerStalkValid and
                   str(cs.gearShifter) == "drive" and not cs.doorOpen and not cs.seatbeltUnlatched and
                   math.isfinite(cs.vEgo) and (waiting or moving))
      reason = "ok" if permitted else "tbt_vehicle_guard"
    else:
      reason = conditions(cs, allow_moving=self.allow_moving)
    action = req.get("action")
    if not supported:
      reason = "unsupported_car_or_bus"
    elif stock_spas_active:
      reason = "stock_spas_active_or_unknown"
    elif not isinstance(identity, str) or not identity or len(identity) > 100:
      reason = "no_request"
    elif identity == self.blocked_id or read_json(self.cancel_path).get("id") == identity:
      reason = "request_cancelled"
    else:
      try:
        start, expires, deadline = (float(req[k]) for k in ("start", "expires", "deadline"))
        if not all(math.isfinite(x) for x in (start, expires, deadline)):
          reason = "invalid_time"
        elif not (0 <= now - start < self.max_seconds and now < expires <= now + LEASE_SECONDS + .05 and
                  now < deadline <= start + self.max_seconds and expires <= deadline):
          reason = "expired"
        elif action not in ("left", "right"):
          reason = "stopped"
      except (ValueError, TypeError, KeyError):
        reason = "invalid_request"
    if reason == "ok" and ((action == "left" and cs.rightBlinker) or
                           (action == "right" and cs.leftBlinker)):
      reason = "opposite_blinker_or_hazards"
    if reason == "ok" and (getattr(cs, "leftBlinkerStalk", False) or getattr(cs, "rightBlinkerStalk", False)):
      reason = "driver_stalk"
    lamp_on = bool(cs.leftBlinker if action == "left" else cs.rightBlinker)
    result = None
    if reason == "ok":
      self.clear_frames = 10
      same_action = self.active_id == identity and self.active_action == action
      if not same_action:
        self.lamp_was_off = not lamp_on
        self.confirmed = False
        self.active_action = action
      if self.active_id != identity:
        self.active_id = identity
        self.sent = 0
        result = 0  # explicit neutral first; resets Panda's previous burst after cooldown
      else:
        result = 3 if action == "left" else 4
        self.confirmed |= same_action and self.sent > 0 and self.lamp_was_off and lamp_on
        self.sent += 1
    else:
      if identity:
        self.blocked_id = identity  # a renewed lease cannot revive an interrupted press
      self.active_id = None
      if self.clear_frames:
        result = 0
        self.clear_frames -= 1
    changed = self.reason != reason
    self.reason = reason
    try:
      if not changed and now - self.last_status < .1:
        return result
      self.last_status = now
      write_json(self.status_path, {"time": now, "id": identity, "reason": reason,
                                   "command": result, "requested_frames": self.sent,
                                   "action": action if reason == "ok" else None,
                                   "voice_id": req.get("voice_id", identity) if self.tbt else identity,
                                   "lamp_confirmed": reason == "ok" and self.confirmed,
                                   "left_observed": bool(cs.leftBlinker), "right_observed": bool(cs.rightBlinker)})
    except OSError:
      # Telemetry failure must never crash card or create another request.
      pass
    return result
