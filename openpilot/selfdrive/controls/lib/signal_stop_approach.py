"""Bound an already-confirmed, lead-free signal stop; never create a stop."""
import math


class SignalStopApproach:
  def __init__(self):
    self.reset()

  def reset(self):
    self.distance = None
    self.confirm = 0.
    self.ceiling = None
    self.accel_cap = None

  def update(self, distance, speed, acceleration, comfort_brake, buffer, enabled, dt):
    values = (distance, speed, acceleration, comfort_brake, buffer, dt)
    if (not enabled or not all(math.isfinite(v) for v in values) or
        not 0. <= distance < 300. or not 0. <= speed <= 40. or
        not .5 <= comfort_brake <= 4. or not 0. <= buffer <= 15. or not .01 <= dt <= .1):
      self.reset()
      return distance

    # Keep a world-fixed target, allowing only 0.25 m/s of outward correction.
    # A nearer target is still accepted; pedal/lead/green transitions reset us.
    predicted = max(0., self.distance-speed*dt) if self.distance is not None else distance
    self.distance = min(distance, predicted+.25*dt)
    self.confirm = min(1., self.confirm+dt)
    if self.ceiling is None:
      self.ceiling = acceleration
    if self.confirm < .2:
      self.accel_cap = None
      return distance

    remaining = max(1., self.distance-buffer)
    required = speed*speed/(2.*remaining)
    target = -min(comfort_brake, max(.15, required))
    # Build braking at 0.6 m/s^3, and release at 0.35 m/s^3.
    rate = .6 if target < self.ceiling else .35
    self.ceiling += max(-rate*dt, min(rate*dt, target-self.ceiling))
    self.accel_cap = self.ceiling if speed >= .3 else None
    return self.distance
