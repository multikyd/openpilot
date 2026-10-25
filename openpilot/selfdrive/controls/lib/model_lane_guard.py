"""Predictive, smoothly released constraint on model curvature near an inside lane.

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
    self.last_reliable_time = None
    self.penetration = 0.0
    self.confidence = 0.0

  def _bounded_result(self, curvature, speed):
    # Always outward, at most 25% of the model turn or 0.45 m/s^2.
    limit = min(.45, .25 * abs(curvature) * speed**2)
    magnitude = min(abs(self.accel), limit)
    self.accel = -self.side * magnitude
    return float(curvature + self.accel / speed**2)

  def _step(self, target, dt):
    # Build early and release slowly, so observation noise does not repeatedly
    # add/remove a steering impulse. The ordinary controller retains its limits.
    releasing = abs(target) < abs(self.accel)
    tau = .6 if releasing else .35
    jerk = .30 if releasing else .40
    change = (1.-math.exp(-dt/tau))*(target-self.accel)
    self.accel += float(np.clip(change, -jerk*dt, jerk*dt))
    if abs(self.accel) < 1e-5:
      self.accel = 0.

  def _release(self, curvature, speed, dt):
    # Fresh model but uncertain line/corridor: no new correction, gradual release.
    self.evidence = 0.
    self.confidence = 0.
    self._step(0., dt)
    return self._bounded_result(curvature, speed)

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
    self.last_model_time = model_time
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
      if not (math.isfinite(probability) and math.isfinite(std)):
        self.reset()
        return curvature
      continuing = self.evidence > 0. or abs(self.accel) > 1e-4
      lane_code = getattr(cs, 'rightLaneLine' if side > 0 else 'leftLaneLine', 0)
      painted_boundary = math.isfinite(lane_code) and 10 <= lane_code < 30
      recent_reliable = (self.last_reliable_time is not None and
                         0. <= model_time-self.last_reliable_time <= .75)
      min_probability, max_std = (.55, .35) if continuing else (.65, .3)
      if painted_boundary and recent_reliable:
        # Brief weaker observations may continue a recently reliable painted
        # boundary, but cannot acquire one from low confidence alone.
        min_probability, max_std = .4, .35
      if not (min_probability <= probability <= 1. and 0. <= std < max_std):
        return self._release(curvature, cs.vEgo, dt)
      lookahead = float(np.clip(cs.vEgo * (np.clip(actuator_delay, .1, .6) + .6), 12., 24.))
      stations = [0., lookahead*.5, lookahead]
      paths = []
      for path in (model.laneLines[1], model.laneLines[2], model.position):
        x, y = np.asarray(path.x, dtype=float), np.asarray(path.y, dtype=float)
        if len(x) < 8 or len(x) != len(y) or x[0] > .5:
          self.reset()
          return curvature
        # A ramp can turn back in vehicle coordinates far beyond our horizon.
        # Validate only the first forward segment that reaches the local horizon.
        reach = np.flatnonzero(x >= lookahead)
        if not len(reach):
          return self._release(curvature, cs.vEgo, dt)
        end = int(reach[0])+1
        local_x, local_y = x[:end], y[:end]
        if (len(local_x) < 2 or not np.all(np.isfinite(local_x)) or not np.all(np.isfinite(local_y)) or
            np.any(np.diff(local_x) <= 0.)):
          self.reset()
          return curvature
        paths.append(np.interp(stations, local_x, local_y))
      left, right, predicted = paths
      left += camera_offset
      right += camera_offset
      widths = right-left
      if (np.any(widths <= 0.) or left[0] >= -.1 or right[0] <= .1 or abs(predicted[0]) > .3):
        self.reset()
        return curvature
      boundary = float(right[0] if side > 0 else left[0])
      if self.last_boundary is not None and abs(boundary-self.last_boundary) > .25 + 2.*dt:
        self.reset()
        return curvature
      # Mild width uncertainty may release a correction, but cannot introduce one.
      min_width, max_width, spread = (2.6, 4.4, .9) if continuing else (2.8, 4.2, .7)
      narrow_painted = painted_boundary and (recent_reliable or (probability >= .65 and std < .3))
      if narrow_painted:
        min_width = 2.45
      if np.any(widths < min_width) or np.any(widths > max_width) or np.ptp(widths) > spread:
        return self._release(curvature, cs.vEgo, dt)
      if probability >= .65 and std < .3:
        self.last_reliable_time = model_time
      clearances = right-predicted if side > 0 else predicted-left
      # Start before the old 1.35m center-to-line boundary, including future
      # model path points. A centered narrow corridor must remain untouched.
      ramp_weight = float(np.interp(abs(curvature), [.003, .010], [0., 1.]))
      margin = min(1.35+.20*ramp_weight, .5*float(np.min(widths))-(.08 if narrow_painted else .15))
      penetration = max(0., margin-float(np.min(clearances))-.03)
      probability_weight = float(np.interp(probability, [.55, .9], [0., 1.]))
      if painted_boundary and recent_reliable:
        probability_weight = max(probability_weight, .35*float(np.interp(probability, [.4, .65], [0., 1.])))
      confidence = min(probability_weight, float(np.interp(std, [.12, .35], [1., 0.])))
    except (AttributeError, IndexError, TypeError, ValueError):
      self.reset()
      return curvature
    self.last_boundary = boundary
    self.confidence += (1.-math.exp(-dt/.3))*(confidence-self.confidence)
    self.penetration = penetration
    self.evidence = min(.5, self.evidence+dt) if penetration > .02 else 0.0
    target = 0.0
    if self.evidence >= .2:
      # A bounded pure-pursuit correction of the model target, not a scaling
      # of all steering. The model retains at least 75% of its turn curvature.
      correction_accel = 2. * min(penetration, .75) * cs.vEgo**2 / lookahead**2
      limit = min(.45, .25 * abs(curvature) * cs.vEgo**2)
      target = -side * min(correction_accel, limit) * self.confidence
    self._step(target, dt)
    return self._bounded_result(curvature, cs.vEgo)
