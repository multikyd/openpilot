"""Conservative heading-progress estimate for navigation left/right turns."""
import math


class TurnProgress:
  def __init__(self):
    self.reset()

  def reset(self):
    self.last_time = self.started = self.code = self.distance = None
    self.direction = self.heading = self.confirm = self.strength = self.extra_speed = 0.
    self.reverse = 0.
    self.armed = False

  def update(self, *, now, enabled, code, distance, speed, angle, curvature, preview, far_preview):
    values = (now, distance, speed, angle, curvature, preview, far_preview)
    previous = self.last_time
    dt = 0. if previous is None else now-previous
    if (not enabled or not all(math.isfinite(v) for v in values) or code not in (1, 2) or
        not -15. <= distance <= 40. or not 1. <= speed <= 60./3.6 or not 0. <= dt <= .25):
      self.reset()
      return 0.
    if self.code is not None and (code != self.code or
        (self.distance is not None and distance-self.distance >= 25.)):
      self.reset()
      dt = 0.
    self.last_time, self.code, self.distance = now, code, distance
    if not self.direction:
      if speed > 12. or abs(angle) < 25. or abs(curvature) < .003:
        return 0.
      self.direction = math.copysign(1., curvature)
      self.started = now
    if now-self.started > 20.:
      self.reset()
      return 0.
    # Curvature is the measured vehicle-model curvature, not desired curvature.
    # Integrating v*k estimates road heading change; steering-wheel degrees are
    # never treated as the angle through the intersection.
    self.reverse = self.reverse+dt if self.direction*curvature < -.003 else 0.
    if self.reverse >= .15:
      self.reset()
      return 0.
    self.heading = max(0., self.heading+self.direction*curvature*speed*dt)
    if abs(angle) >= 60. and abs(curvature)*speed**2 >= .3:
      self.confirm += dt
      self.armed = self.armed or self.confirm >= .3
    elif not self.armed:
      self.confirm = 0.
    opening = (self.armed and math.radians(45.) <= self.heading <= math.radians(110.) and
               abs(curvature)*speed**2 <= 1.6 and abs(preview)*speed**2 <= 1.6 and
               abs(far_preview)*speed**2 <= 1.2 and
               self.direction*preview >= -.001 and self.direction*far_preview >= -.001)
    fraction = min(1., max(0., (math.degrees(self.heading)-45.)/30.))
    target = .2+.8*fraction if opening else 0.
    self.strength += max(-dt/.4, min(dt/.6, target-self.strength))
    if not opening:
      # Never relax a speed cap while the current/future bend is too strong.
      self.strength = self.extra_speed = 0.
      return 0.
    # Start at a small 0.15 m/s² allowance and grow with heading progress.
    # This relaxes only the TBT cap, not camera, signal, lead or curve caps.
    self.extra_speed = min(2.5, self.extra_speed+self.strength*(.15+.2*fraction)*dt)
    return self.extra_speed
