"""Corner-object identity, adapted from ajouatom/openpilot's slot-handoff logic.

Object IDs are reused, so an ID alone never joins two returns. Allocation is
monotonic and independent of CAN slots; inactive slots retain their legacy IDs.
"""
from dataclasses import dataclass
import math

STABLE_CORNER_TRACK_ID_START = 1000
STABLE_CORNER_TRACK_ID_END = 2**31
CORNER_IDENTITY_TIMEOUT = .15


def is_corner_track_id(track_id, stable_enabled=True):
  return (200 <= track_id < 220 or 240 <= track_id < 250 or
          (stable_enabled and STABLE_CORNER_TRACK_ID_START <= track_id < STABLE_CORNER_TRACK_ID_END))


def deduplicate_corner_candidates(candidates):
  """Prefer older/better close copies, retaining far objects with the same ID."""
  objects = []
  for candidate in sorted(candidates, key=lambda c: (-c[2], -c[3], c[0])):
    _, object_id, _, _, d_rel, y_rel, v_rel, *_ = candidate
    if any(object_id == old[1] and abs(d_rel-old[4]) <= 2. and
           abs(y_rel-old[5]) <= 1. and abs(v_rel-old[6]) <= 3. for old in objects):
      continue
    objects.append(candidate)
  return objects


@dataclass(frozen=True)
class CornerIdentity:
  slot: int
  object_id: int
  age: int
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  timestamp: float


class CornerObjectTrackIdManager:
  def __init__(self):
    self.next_track_id = STABLE_CORNER_TRACK_ID_START
    self.states = {}
    self.last_time = None

  def clear(self):
    self.states.clear()
    self.last_time = None

  def assign(self, candidates, now):
    if not math.isfinite(now):
      self.clear()
      return {}
    if self.last_time is not None and now < self.last_time:
      self.clear()
    self.last_time = now
    self.states = {key: state for key, state in self.states.items()
                   if 0. <= now-state.timestamp <= CORNER_IDENTITY_TIMEOUT}
    # Sort every possible association first, so input CAN-slot order cannot
    # steal an established slot from a better geometric match.
    matches = []
    for candidate in candidates:
      slot, object_id, age, _, d_rel, y_rel, v_rel, yv_rel, _ = candidate
      if not all(math.isfinite(v) for v in (d_rel,y_rel,v_rel,yv_rel)):
        continue
      for key, state in self.states.items():
        dt = now-state.timestamp
        if object_id != state.object_id or age < state.age:
          continue
        dx = abs(d_rel-(state.d_rel+state.v_rel*dt))
        dy = abs(y_rel-(state.y_rel+state.yv_rel*dt))
        # More restrictive than the source's 7m/3.2m unpredicted gate:
        # rapid replacement or impossible motion starts a fresh history.
        if dx > 2.+abs(state.v_rel)*dt*.1 or dy > .5+1.8*dt or abs(v_rel-state.v_rel) > 3.:
          continue
        matches.append((state.slot != slot, dx+1.5*dy+.2*abs(v_rel-state.v_rel), slot, key))
    assignments = {}
    used = set()
    for _, _, slot, key in sorted(matches):
      if slot not in assignments and key not in used:
        assignments[slot] = key
        used.add(key)
    for candidate in candidates:
      slot, object_id, age, _, d_rel, y_rel, v_rel, yv_rel, _ = candidate
      if not all(math.isfinite(v) for v in (d_rel,y_rel,v_rel,yv_rel)):
        continue
      if slot not in assignments:
        assignments[slot] = self.next_track_id
        self.next_track_id += 1
      key = assignments[slot]
      self.states[key] = CornerIdentity(slot,object_id,age,d_rel,y_rel,v_rel,yv_rel,now)
    return assignments
