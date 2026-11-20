"""Select a guidance session and arbitrate live signal observations independently."""
from dataclasses import dataclass, field
import math
import threading
import time

from openpilot.selfdrive.kisapilot.nav_signal_input import directional_signal, signal_from_sinf, usable_signal

FRESH_SECONDS = 4.0
GUIDANCE_EVENTS = ("rgdata", "route", "complexCrossroad")
SIGNAL_EVENTS = ("ssinf", "sinf")
PACKET_FIELDS = frozenset((
  "source", "bridge_version", "timestamp_ms", "raw_rgdata", "raw_naver",
  "raw_ssinf", "raw_sinf", "signal_update", "navi_state",
))


def valid_position(value):
  if not isinstance(value, dict):
    return False
  try:
    lat = float(value["vpPosPointLat"])
    lon = float(value["vpPosPointLon"])
  except (KeyError, TypeError, ValueError, OverflowError):
    return False
  return abs(lat) <= 90 and abs(lon) <= 180 and (lat != 0 or lon != 0)


@dataclass
class GuidanceSession:
  address: tuple
  source: str = "legacy"
  instance: str = ""
  sequence: int = -1
  active: bool = True
  foreground: bool = False
  foreground_since: float = 0
  position_at: float | None = None
  guidance: dict = field(default_factory=dict)

  def accept_state(self, state):
    if not isinstance(state, dict):
      return True
    sequence = state.get("sequence")
    instance = state.get("instance_id")
    if type(sequence) is not int or not isinstance(instance, str):
      return False
    if instance == self.instance and sequence <= self.sequence:
      return False
    if instance != self.instance:
      self.guidance.clear()
      self.position_at = None
    self.instance = instance
    self.sequence = sequence
    self.active = state.get("active") is True
    self.foreground = state.get("foreground") is True
    epoch = state.get("foreground_since_elapsed_ms", 0)
    self.foreground_since = epoch if isinstance(epoch, (int, float)) and math.isfinite(epoch) else 0
    return True

  def priority(self, current):
    return (self.foreground, self.foreground_since if self.foreground else 0, current, self.position_at)

  def is_fresh(self, now):
    return self.active and self.position_at is not None and now - self.position_at <= FRESH_SECONDS


class NaviSourceMux:
  def __init__(self, dispatch, changed, received=lambda *args: None, clock=time.monotonic):
    self.dispatch = dispatch
    self.changed = changed
    self.received = received
    self.clock = clock
    self.lock = threading.RLock()
    self.sessions = {}
    self.selected = None
    self.signal_source = None
    self.signal_original_at = float("-inf")

  def open(self, key, addr):
    with self.lock:
      self.sessions[key] = GuidanceSession(addr)

  def close(self, key):
    with self.lock:
      self.sessions.pop(key, None)
      self._choose_guidance(self.clock())

  def tick(self):
    with self.lock:
      self._choose_guidance(self.clock())

  def feed(self, key, payload):
    if not isinstance(payload, dict):
      return
    with self.lock:
      session = self.sessions.get(key)
      if session is None or not session.accept_state(payload.get("navi_state")):
        return
      source = payload.get("source")
      if source in ("naver-map", "tmap"):
        session.source = source
      now = self.clock()
      metadata = {k: v for k, v in payload.items() if k in PACKET_FIELDS}
      outgoing = {}
      for kind in GUIDANCE_EVENTS:
        key_in_payload = "vrtx" if kind == "route" and "vrtx" in payload else kind
        if key_in_payload not in payload:
          continue
        value = payload[key_in_payload]
        if kind == "rgdata":
          if not valid_position(value):
            continue
          session.position_at = now
        packet = {**metadata, kind: value}
        session.guidance[kind] = (now, packet)
        outgoing[kind] = packet
      changed = self._choose_guidance(now)
      for kind in SIGNAL_EVENTS:
        if kind in payload:
          self._deliver_signal(key, {**metadata, kind: payload[kind]}, kind, now)
      if key == self.selected:
        if not changed:
          for packet in outgoing.values():
            self.dispatch(packet)
        if "rgdata" in outgoing:
          self.received(session.source, outgoing["rgdata"])

  def _choose_guidance(self, now):
    active = (key for key, session in self.sessions.items() if session.is_fresh(now))
    target = max(active, key=lambda key: self.sessions[key].priority(key == self.selected), default=None)
    if target == self.selected:
      return False
    self.selected = target
    session = self.sessions.get(target)
    if session is None:
      self.changed(None, None)
      return True
    self.changed(session.source, session.address)
    for kind in GUIDANCE_EVENTS:
      cached = session.guidance.get(kind)
      if cached is not None and (kind == "route" or now - cached[0] <= FRESH_SECONDS):
        self.dispatch(cached[1])
    return True

  def _deliver_signal(self, key, packet, kind, now):
    data = packet[kind]
    if not isinstance(data, dict):
      return
    signal = signal_from_sinf(data) if kind == "sinf" else data
    if not usable_signal(directional_signal(signal, now, {}, {})):
      return
    update = packet.get("signal_update")
    if not isinstance(update, dict):
      update = {}
    if update.get("status") == "phase_expired":
      return
    local_countdown = update.get("kind") == "local_countdown"
    if local_countdown and key != self.signal_source and now - self.signal_original_at <= FRESH_SECONDS:
      return
    self.signal_source = key
    if not local_countdown:
      self.signal_original_at = now
    self.dispatch(packet)
