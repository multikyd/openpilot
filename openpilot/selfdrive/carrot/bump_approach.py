"""Latch a bump's final target once its distance-based braking point is reached."""
import math


class BumpApproach:
  def __init__(self):
    self.states = {}

  def update(self, now, distance, target, speed, approach_limit, channel):
    if (not all(math.isfinite(v) for v in (now, distance, target, speed, approach_limit)) or
        distance < 0. or not 0. < target <= 80. or not 0. <= speed <= 250.):
      self.states.pop(channel, None)
      return 250.
    previous = self.states.get(channel)
    continuous = (previous is not None and 0. <= now-previous[0] <= 1. and
                  distance-previous[1] <= 20.)
    # Before the braking point, do not gradually lower the cruise target.
    # Once braking starts, falling ego speed must not release/reapply the cap.
    active = (continuous and previous[2]) or approach_limit <= max(speed, target)+.5
    self.states[channel] = (now, distance, active)
    return target if active else 250.
