"""Limit new, unsupported curvature while creeping to a stop.

Retain the approach bend; reliable local lane geometry can extend that bend.
This is neither a zero-angle command nor a replacement for actuator limits.
"""
import math

import numpy as np


class StopSteeringGuard:
  ENTER_SPEED = 2.5  # m/s, before the observed low-speed model divergence
  CURVATURE_ROOM = .0015

  def __init__(self):
    self.reset()

  def reset(self):
    self.reference = None
    self.reference_time = None
    self.last_time = self.model_time = None
    self.active = False
    self.low = self.high = 0.
    self.geometry = None
    self.decelerating_time = 0.

  @staticmethod
  def lane_curvature(model):
    """Use geometry, not the low-speed action head, to support a tighter bend."""
    try:
      # Real Cap'n Proto lists accept integer indices, not Python slices.
      if (min(model.laneLineProbs[i] for i in (1, 2)) < .75 or
          max(model.laneLineStds[i] for i in (1, 2)) > .3):
        return None
      stations = np.linspace(0., 8., 17)
      ys = []
      for index in (1, 2):
        line = model.laneLines[index]
        x, y = np.asarray(line.x), np.asarray(line.y)
        if len(x) < 8 or len(x) != len(y) or x[0] > .5:
          return None
        reach = np.flatnonzero(x >= stations[-1])
        if not len(reach):
          return None
        end = int(reach[0])+1
        if not np.all(np.isfinite(x[:end])) or not np.all(np.isfinite(y[:end])) or np.any(np.diff(x[:end]) <= 0.):
          return None
        ys.append(np.interp(stations, x[:end], y[:end]))
      width = ys[1]-ys[0]
      if np.any(width < 2.4) or np.any(width > 4.5) or np.ptp(width) > .6:
        return None
      center = (ys[0]+ys[1])*.5
      a, b, c = np.polynomial.polynomial.polyfit(stations, center, 2)
      fitted = a+b*stations+c*stations**2
      curvature = 2.*c/(1.+b*b)**1.5
      if np.max(np.abs(fitted-center)) > .08 or abs(curvature) > .08:
        return None
      return float(curvature)
    except (AttributeError, IndexError, TypeError, ValueError):
      return None

  def update(self, model, cs, curvature, measured_curvature, stamp, now, *, stopping, enabled):
    values = (curvature, measured_curvature, stamp, now, cs.vEgo)
    if (not enabled or not all(math.isfinite(v) for v in values) or
        not 0. <= now-stamp <= .2 or cs.vEgo < 0. or cs.steeringPressed or
        cs.leftBlinker or cs.rightBlinker or str(model.meta.desire) != 'none' or
        str(model.meta.laneChangeState) != 'off'):
      self.reset()
      return curvature
    if self.last_time is not None and not 0. < now-self.last_time <= .2:
      self.reset()
    dt = .01 if self.last_time is None else now-self.last_time
    self.last_time = now
    # Stock SCC stops need not set the openpilot planner's shouldStop flag.
    # Require sustained physical slowing, rather than classifying all creep as a stop.
    decelerating = (math.isfinite(cs.aEgo) and cs.aEgo < -.15 and not cs.gasPressed)
    self.decelerating_time = min(.4, self.decelerating_time+dt) if decelerating else 0.
    if cs.vEgo >= self.ENTER_SPEED:
      self.reference, self.reference_time = curvature, now
      self.active = False
      return curvature
    if not self.active:
      if not stopping and self.decelerating_time < .35:
        return curvature
      # With no recent approach history, retain the current physical bend.
      if self.reference_time is None or now-self.reference_time > .3:
        self.reference = measured_curvature
      self.low = min(0., self.reference)-self.CURVATURE_ROOM
      self.high = max(0., self.reference)+self.CURVATURE_ROOM
      self.active = True
    # Keep the envelope during creep, including small stop-request flicker.
    # Disengagement, driver input, maneuvers and normal speed reset it above.
    if self.model_time != stamp:
      geometry = self.lane_curvature(model)
      self.geometry = geometry
      self.model_time = stamp
    geometry = self.geometry
    low = min(0., self.reference, geometry if geometry is not None else self.reference)-self.CURVATURE_ROOM
    high = max(0., self.reference, geometry if geometry is not None else self.reference)+self.CURVATURE_ROOM
    # Lane observations may change at crosswalks; do not open the envelope in a jump.
    step = .003*dt
    self.low += float(np.clip(low-self.low, -step, step))
    self.high += float(np.clip(high-self.high, -step, step))
    return float(np.clip(curvature, self.low, self.high))
