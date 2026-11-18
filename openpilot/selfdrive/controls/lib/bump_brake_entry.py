"""Limit only the first acceleration drop of an unconstrained bump approach."""
import math


class BumpBrakeEntry:
  JERK = 1.5  # m/s^3, downward acceleration change during entry only

  def __init__(self):
    self.reset()

  def reset(self):
    self.seen = self.running = self.done = False
    self.output = 0.
    self.last_time = self.last_distance = None

  def update(self, now, distance, target, speed, desired, previous, measured, *, active, enabled, max_decel=2.):
    if not active:
      self.reset()
      return desired
    if not all(math.isfinite(v) for v in (now, distance, target, speed, desired, previous, measured, max_decel)):
      self.done = True
      return desired
    dt = .05 if self.last_time is None else now-self.last_time
    new_bump = self.last_distance is not None and distance-self.last_distance > 20.
    if new_bump:
      self.reset()
      dt = .05
    if not .001 <= dt <= .25:
      self.done = True
    self.last_time, self.last_distance = now, distance
    if not self.seen:
      self.seen = True
      self.output = min(previous, measured)
    if not enabled or distance <= 0. or target <= 0. or speed <= target+.3:
      self.done = True
    if self.done:
      return desired
    if not self.running:
      if desired >= min(previous, measured)-.05:
        return desired
      self.running = True
      self.output = min(previous, measured)
    decel = min(2., max(.5, max_decel))
    ramp_time = max(0., (self.output+decel)/self.JERK)
    # Reserve the extra distance needed for the initial jerk ramp. A late or
    # suddenly closer bump keeps the original braking request immediately.
    needed = (max(0., speed**2-target**2)/(2.*decel) + speed*ramp_time*.5 +
              max(0., self.output)*ramp_time**2*.5 + 3.)
    if distance <= needed:
      self.done = True
      return desired
    baseline = min(previous, self.output)
    self.output = max(desired, baseline-self.JERK*dt)
    if self.output <= desired+1e-6:
      self.done = True
    return self.output
