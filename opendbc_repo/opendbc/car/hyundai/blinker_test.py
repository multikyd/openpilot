"""Stationary SPAS2-only experiment. No steering, ECU takeover, or driving use."""
import json
import math
import os
from pathlib import Path

SAFETY_FLAG = 1024
REQUEST = Path("/dev/shm/kisa_blinker_request.json")
STATUS = Path("/dev/shm/kisa_blinker_status.json")
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


def conditions(cs, cc):
  checks = {"can_valid": bool(cs.canValid), "gear": str(cs.gearShifter) in ("park", "drive"),
            "stationary": bool(cs.standstill and math.isfinite(cs.vEgo) and abs(cs.vEgo) < 0.05),
            "brake": bool(cs.brakePressed), "gas_released": not cs.gasPressed,
            "doors_closed": not cs.doorOpen, "seatbelt": not cs.seatbeltUnlatched,
            "automatic_control_off": not (cc.enabled or cc.latActive or cc.longActive)}
  return next((key for key, value in checks.items() if not value), "ok")


class BlinkerTestSender:
  def __init__(self, request=REQUEST, status=STATUS):
    self.request_path, self.status_path = request, status
    self.blocked_id = None
    self.active_id = None
    self.clear_frames = 0
    self.sent = 0
    self.reason = "idle"
    self.last_status = -1.0

  def update(self, now, cs, cc, supported, stock_spas_active=False):
    """Called at 50Hz. Return 0/3/4 to pack, or None for no test message."""
    req = read_json(self.request_path)
    identity = req.get("id")
    reason = conditions(cs, cc)
    action = req.get("action")
    if not supported:
      reason = "unsupported_car_or_bus"
    elif stock_spas_active:
      reason = "stock_spas_active_or_unknown"
    elif not isinstance(identity, str) or not identity or len(identity) > 100:
      reason = "no_request"
    elif identity == self.blocked_id:
      reason = "request_cancelled"
    else:
      try:
        start, expires, deadline = (float(req[k]) for k in ("start", "expires", "deadline"))
        if not all(math.isfinite(x) for x in (start, expires, deadline)):
          reason = "invalid_time"
        elif not (0 <= now - start < MAX_SECONDS and now < expires <= now + LEASE_SECONDS + .05 and
                  now < deadline <= start + MAX_SECONDS and expires <= deadline):
          reason = "expired"
        elif action not in ("left", "right"):
          reason = "stopped"
      except (ValueError, TypeError, KeyError):
        reason = "invalid_request"
    if reason == "ok" and ((action == "left" and cs.rightBlinker) or
                           (action == "right" and cs.leftBlinker)):
      reason = "opposite_blinker_or_hazards"
    result = None
    if reason == "ok":
      self.clear_frames = 10
      if self.active_id != identity:
        self.active_id = identity
        self.sent = 0
        result = 0  # explicit neutral first; resets Panda's previous burst after cooldown
      else:
        result = 3 if action == "left" else 4
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
                                   "left_observed": bool(cs.leftBlinker), "right_observed": bool(cs.rightBlinker)})
    except OSError:
      # Telemetry failure must never crash card or create another request.
      pass
    return result
