"""Prefer the model while a nearby lead obscures lane lines at low speed."""
import math


class CloseLeadModelMode:
  def __init__(self):
    self.active = False
    self.entry_since = None
    self.clear_since = None
    self.last_time = None

  def update(self, now, speed, lead, lane_probs, *, fresh, enabled):
    if not enabled or (self.last_time is not None and now < self.last_time):
      self.__init__()
      return False
    self.last_time = now
    # Missing observations cannot start a mode change or count toward recovery.
    if not fresh:
      self.entry_since = self.clear_since = None
      return self.active
    probs = list(lane_probs)
    valid = (math.isfinite(speed) and speed >= 0 and len(probs) == 4 and
             all(math.isfinite(p) and 0 <= p <= 1 for p in probs))
    if not valid:
      self.entry_since = self.clear_since = None
      return self.active
    distance = lead.dRel
    close = (lead.present and math.isfinite(distance) and
             0 < distance <= min(24., max(12., speed * 2.5)))
    weak_lanes = min(probs[1:3]) < .95
    entry = speed <= 30. / 3.6 and close and weak_lanes
    if not self.active:
      self.entry_since = (now if self.entry_since is None else self.entry_since) if entry else None
      if self.entry_since is not None and now-self.entry_since >= .10:
        self.active = True
        self.entry_since = None
    else:
      # Remain in model mode through stop-and-go and brief lane reappearances.
      clear = (speed >= 40. / 3.6 or (not close and min(probs[1:3]) >= .98))
      self.clear_since = (now if self.clear_since is None else self.clear_since) if clear else None
      if self.clear_since is not None and now-self.clear_since >= 2.:
        self.active = False
        self.clear_since = None
    return self.active
