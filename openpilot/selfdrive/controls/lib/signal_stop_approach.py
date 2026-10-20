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
    self.travel = 0.
    self.outward_candidate = None
    self.outward_confirm = 0.
    self.extension_used = 0.

  def update(self, distance, speed, acceleration, comfort_brake, buffer, enabled, dt):
    values = (distance, speed, acceleration, comfort_brake, buffer, dt)
    if (not enabled or not all(math.isfinite(v) for v in values) or
        not 0. <= distance < 300. or not 0. <= speed <= 40. or
        not .5 <= comfort_brake <= 4. or not 0. <= buffer <= 15. or not .01 <= dt <= .1):
      self.reset()
      return distance

    # Compare targets in a travelled-distance frame: a stationary road target
    # should stay constant here even while the vehicle approaches it.
    self.travel += speed*dt
    predicted = max(0., self.distance-speed*dt) if self.distance is not None else distance
    gap = distance-predicted
    world_target = distance+self.travel
    eligible = (.5 <= speed <= 15. and distance > buffer+3. and
                .25 < gap <= 20. and distance > buffer+10. and self.extension_used < 12.)
    if eligible:
      if self.outward_candidate is None or abs(world_target-self.outward_candidate) > 1.5:
        self.outward_candidate = world_target
        self.outward_confirm = 0.
      else:
        self.outward_confirm += dt
    else:
      self.outward_candidate = None
      self.outward_confirm = 0.
    # Follow only a sustained correction of a still-distant model target.
    # Bound extension to 2 m/s and 12 m total; never extend a near stop.
    extension = 0.
    if eligible and self.outward_confirm >= .75:
      extension = min(gap, min(2., .25*speed)*dt, 12.-self.extension_used)
      self.extension_used += extension
    self.distance = min(distance, predicted+(.25*dt if speed >= .5 else 0.)+extension)
    self.confirm = min(1., self.confirm+dt)
    if self.ceiling is None:
      self.ceiling = acceleration
    if self.confirm < .2:
      self.accel_cap = None
      return distance

    remaining = max(1., self.distance-buffer)
    required = speed*speed/(2.*remaining)
    target = -min(comfort_brake, max(.15, required))
    # Build braking at 0.6 m/s^3. Unwind an over-strong cap smoothly
    # when a persistent outward correction was confirmed.
    rate = .6 if target < self.ceiling else .7 if extension > 0. else .35
    self.ceiling += max(-rate*dt, min(rate*dt, target-self.ceiling))
    self.accel_cap = self.ceiling if speed >= .3 else None
    return self.distance


class SignalStopOverride:
  """Recheck a previous signal stop after a short, low-speed pedal correction."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.active = False
    self.duration = 0.
    self.travel = 0.
    self.eligible = False

  def update(self, gas, stopping, speed, enabled, lead, green, dt):
    if (not enabled or lead or green or not math.isfinite(speed) or
        not 0. <= speed <= 40. or not .01 <= dt <= .1):
      self.reset()
      return False
    if gas:
      if not self.active and stopping:
        self.active = True
        self.eligible = speed <= 5.
      if self.active:
        self.duration += dt
        self.travel += speed*dt
        self.eligible = self.eligible and self.duration <= 2. and self.travel <= 8. and speed <= 5.
      return False
    recheck = self.active and self.eligible
    self.reset()
    return recheck
