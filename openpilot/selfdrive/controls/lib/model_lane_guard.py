"""Small outward-only constraint on model curvature near a trusted inside lane.

This does not replace the model path, change the turn direction, or change
steering authority. The caller must retain its normal curvature/safety limits.
"""
import math

import numpy as np


class ModelLaneGuard:
  def __init__(self):
    self.reset()

  def reset(self):
    self.side = 0  # positive curvature/y is right
    self.accel = 0.0  # added lateral acceleration, m/s^2
    self.evidence = 0.0
    self.last_model_time = None
    self.last_boundary = None
    self.penetration = 0.0

  def _bounded_result(self, curvature, speed):
    # Always outward, at most 25% of the model turn or 0.45 m/s^2.
    limit = min(.45, .25 * abs(curvature) * speed**2)
    magnitude = min(abs(self.accel), limit)
    self.accel = -self.side * magnitude
    return float(curvature + self.accel / speed**2)

  def update(self, model, cs, curvature, model_time, now, *, enabled=True,
             camera_offset=.04, actuator_delay=.4):
    scalars = (curvature, model_time, now, camera_offset, actuator_delay, cs.vEgo)
    valid = (enabled and all(math.isfinite(v) for v in scalars) and
             0. <= now-model_time <= .25 and abs(camera_offset) <= .2 and
             8. <= cs.vEgo <= 35. and .0012 <= abs(curvature) <= .04 and
             not cs.steeringPressed and not cs.leftBlinker and not cs.rightBlinker)
    if not valid:
      self.reset()
      return curvature
    side = 1 if curvature > 0. else -1
    # Do not carry a right-boundary correction into a left bend (or vice versa).
    if side != self.side:
      self.reset()
      self.side = side
    if self.last_model_time == model_time:
      return self._bounded_result(curvature, cs.vEgo)
    dt = model_time-self.last_model_time if self.last_model_time is not None else .05
    if not .015 <= dt <= .12:
      self.reset()
      self.side = side
      dt = .05
    try:
      if str(model.meta.laneChangeState) != 'off' or str(model.meta.desire) != 'none':
        self.reset()
        return curvature
      if len(model.laneLines) < 4 or len(model.laneLineProbs) < 4 or len(model.laneLineStds) < 4:
        self.reset()
        return curvature
      inside = 2 if side > 0 else 1
      probability = float(model.laneLineProbs[inside])
      std = float(model.laneLineStds[inside])
      if not (math.isfinite(probability) and math.isfinite(std) and .65 <= probability <= 1. and 0. <= std < .3):
        self.reset()
        return curvature
      lookahead = float(np.clip(cs.vEgo * (np.clip(actuator_delay, .1, .6) + .6), 12., 24.))
      stations = [0., lookahead*.5, lookahead]
      paths = []
      for path in (model.laneLines[1], model.laneLines[2], model.position):
        x, y = np.asarray(path.x, dtype=float), np.asarray(path.y, dtype=float)
        if (len(x) < 8 or len(x) != len(y) or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)) or
            np.any(np.diff(x) <= 0.) or x[0] > .5 or x[-1] < lookahead):
          self.reset()
          return curvature
        paths.append(np.interp(stations, x, y))
      left, right, predicted = paths
      left += camera_offset
      right += camera_offset
      widths = right-left
      # Diverging/merged lines and sudden identity changes are not a corridor.
      if (np.any(widths < 2.8) or np.any(widths > 4.2) or np.ptp(widths) > .7 or
          left[0] >= -.1 or right[0] <= .1 or abs(predicted[0]) > .3):
        self.reset()
        return curvature
      boundary = float(right[0] if side > 0 else left[0])
      if self.last_boundary is not None and abs(boundary-self.last_boundary) > .25 + 2.*dt:
        self.reset()
        return curvature
      clearances = right-predicted if side > 0 else predicted-left
      # Center-to-line margin includes a small buffer beyond half vehicle width.
      penetration = max(0., 1.35-float(np.min(clearances))-.03)
      confidence = min(float(np.interp(probability, [.65, .9], [0., 1.])),
                       float(np.interp(std, [.12, .3], [1., 0.])))
    except (AttributeError, IndexError, TypeError, ValueError):
      self.reset()
      return curvature
    self.last_model_time = model_time
    self.last_boundary = boundary
    self.penetration = penetration
    self.evidence = min(.5, self.evidence+dt) if penetration > .02 else 0.0
    target = 0.0
    if self.evidence >= .2:
      # A bounded pure-pursuit correction of the model target, not a scaling
      # of all steering. The model retains at least 75% of its turn curvature.
      correction_accel = 2. * min(penetration, .75) * cs.vEgo**2 / lookahead**2
      limit = min(.45, .25 * abs(curvature) * cs.vEgo**2)
      target = -side * min(correction_accel, limit) * confidence
    # At most 1 m/s^3 additional lateral jerk. Invalid geometry or a change of
    # turn side removes this contribution immediately; the caller still clips.
    self.accel += float(np.clip(target-self.accel, -1.*dt, 1.*dt))
    return self._bounded_result(curvature, cs.vEgo)
