"""Translate a confirmed narrow-right path preference into bounded model steering.

This uses model geometry, not a claim that curbs, pedestrians or parked cars have
been classified. The existing boundary guard and final steering limits remain.
"""
import math
from types import SimpleNamespace

import numpy as np
from openpilot.selfdrive.controls.lib.single_lane_reference import SingleLaneReference


class ModelNarrowRoadGuard:
  def __init__(self):
    self.bias = SimpleNamespace(shift=0., confirm=0.)
    self.single_lane = SingleLaneReference()
    self.reset()

  def reset(self, reason='off', reset_reference=True):
    if reset_reference:
      self.single_lane.reset()
    self.bias.shift = self.bias.confirm = 0.
    self.stamp = self.reliable_stamp = self.geometry = None
    self.accel = 0.
    self.reason = reason

  @property
  def active(self):
    return self.accel > 1e-5 or self.bias.shift > 1e-5

  def result(self, curvature, guarded, speed):
    # An existing leftward boundary correction vetoes the right preference.
    if guarded < curvature - 1e-6:
      self.reset('boundary veto')
      return guarded
    # Do not stack two corrections toward the right.
    return max(guarded, curvature + self.accel / max(speed, 2.)**2)

  @staticmethod
  def samples(line, stations):
    x, y = np.asarray(line.x, dtype=float), np.asarray(line.y, dtype=float)
    if (len(x) < 8 or len(x) != len(y) or not np.all(np.isfinite(x)) or
        not np.all(np.isfinite(y)) or not np.all(np.diff(x) > 0.) or
        x[0] > .5 or x[-1] < stations[-1]):
      raise ValueError('incomplete corridor')
    return np.interp(stations, x, y)

  def update(self, model, cs, curvature, guarded, stamp, now, *, enabled, actuator_delay=.4):
    try:
      valid = (enabled and all(math.isfinite(v) for v in
               (curvature, guarded, stamp, now, cs.vEgo, actuator_delay)) and
               0. <= now-stamp <= .2 and 2. <= cs.vEgo <= 60./3.6 and
               abs(curvature) < .0125 and abs(curvature)*cs.vEgo**2 < 1.2 and not cs.steeringPressed and
               not cs.leftBlinker and not cs.rightBlinker and not getattr(cs, 'rightBlindspot', False) and
               str(model.meta.desire) in ('none', '0') and
               str(model.meta.laneChangeState) in ('off', '0'))
    except (AttributeError, TypeError, ValueError):
      valid = False
    if not valid or guarded < curvature - 1e-6:
      self.reset('gated')
      return guarded
    if stamp == self.stamp:
      return self.result(curvature, guarded, cs.vEgo)
    dt = .05 if self.stamp is None else stamp-self.stamp
    if not .015 <= dt <= .15:
      self.reset()
      dt = .05
    self.stamp = stamp
    single_shift = self.single_lane.update(model, cs, curvature, stamp, now, enabled=enabled)
    if self.single_lane.supported:
      self.bias.shift = single_shift
      self.bias.confirm = self.single_lane.confirm
      lookahead = float(np.clip(cs.vEgo*(np.clip(actuator_delay, .1, .6)+.8), 6., 15.))
      target = min(.15, 2.*single_shift*cs.vEgo**2/lookahead**2)
      change = (1.-math.exp(-dt/.5))*(target-self.accel)
      self.accel += float(np.clip(change, -.15*dt, .15*dt))
      self.accel = min(self.accel, 2.*self.single_lane.room*cs.vEgo**2/lookahead**2)
      self.reason = self.single_lane.reason
      return self.result(curvature, guarded, cs.vEgo)
    try:
      probs = np.asarray(model.laneLineProbs, dtype=float)
      stds = np.asarray(model.laneLineStds, dtype=float)
      space = float(model.meta.laneWidthRight)
      if (len(probs) < 4 or len(stds) < 4 or not np.all(np.isfinite([*probs, *stds, space])) or
          np.any(probs < 0.) or np.any(probs > 1.) or np.any(stds < 0.) or space < 0.):
        raise ValueError('invalid confidence')
      lookahead = float(np.clip(cs.vEgo*(np.clip(actuator_delay, .1, .6)+.8), 6., 15.))
      stations = np.linspace(0., max(lookahead, min(15., cs.vEgo+3.)), 9)
      left = self.samples(model.laneLines[1], stations)
      right = self.samples(model.laneLines[2], stations)
      path_y = self.samples(model.position, stations)
      width = right-left
      if np.any(width <= 2.5) or np.any(width >= 3.5) or np.ptp(width) > .4:
        raise ValueError('not a narrow corridor')
      if probs[3] >= .3 or (space >= 2.6 and probs[3] >= .15):
        raise ValueError('neighbor to the right')
      # No initial command without a reliable right edge. A briefly weaker edge
      # may retain a previously supported target, but can never increase it.
      edge_std = float(model.roadEdgeStds[1])
      if not math.isfinite(edge_std) or not 0. <= edge_std < .6:
        raise ValueError('untrusted road edge')
      edge = self.samples(model.roadEdges[1], stations)-.4*edge_std
      bound = np.minimum(right, edge)
      room = max(0., float(np.min(bound-1.25-.3*stds[2]-path_y)))
      if room <= 0.:
        raise ValueError('no right margin')
      geometry = np.array([left[0], right[0], edge[0]])
      jumping = self.geometry is not None and np.max(np.abs(geometry-self.geometry)) > .18+dt
      reliable = (probs[1] > .3 and probs[2] > .4 and stds[1] < .6 and stds[2] < .45 and
                  (stds[1] < .45 or (probs[2] > .65 and stds[2] < .25)))
      if reliable and not jumping:
        context = space < 2.2 and probs[3] < .15 and float(np.median(width)) < 3.3
        self.bias.confirm = min(1.5, self.bias.confirm+dt) if context else max(0., self.bias.confirm-dt)
        target_shift = 0.
        if self.bias.confirm >= 1.:
          near = stations <= 5.
          error = float(np.median((left[near]+right[near])*.5+.04-path_y[near]))
          curve_fade = float(np.interp(abs(curvature), [.00375, .0125], [1., 0.]))
          width_fade = float(np.interp(float(np.min(width)), [2.5, 2.7, 3.1, 3.5], [0., 1., 1., 0.]))
          target_shift = min(.12, max(0., error))*curve_fade*width_fade
        rate = .025 if target_shift > self.bias.shift else .03
        self.bias.shift += float(np.clip(target_shift-self.bias.shift, -rate*dt, rate*dt))
        self.reliable_stamp, self.geometry = stamp, geometry
      else:
        # Bridge 0.35 s of bumps/probability dips, then release at 3 cm/s.
        if self.reliable_stamp is None or stamp-self.reliable_stamp > .35:
          self.bias.shift = max(0., self.bias.shift-.03*dt)
          self.bias.confirm = 0.
          self.geometry = None
      self.bias.shift = min(self.bias.shift, room)
      fade = float(np.interp(cs.vEgo, [2., 4., 50./3.6, 60./3.6], [0., 1., 1., 0.]))
      target = min(.15, 2.*self.bias.shift*cs.vEgo**2/lookahead**2)*fade
      if not reliable or jumping:
        target = min(target, self.accel)
      change = (1.-math.exp(-dt/.5))*(target-self.accel)
      self.accel += float(np.clip(change, -.15*dt, .15*dt))
      # Freshly reduced clearance wins over smooth release.
      self.accel = min(self.accel, 2.*room*cs.vEgo**2/lookahead**2)
      if self.bias.shift == 0. and self.accel < 1e-5:
        self.accel = 0.
      self.reason = ('tracking' if self.active else 'confirming') if reliable and not jumping else 'holding'
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
      self.reset(str(exc), reset_reference=False)
      return guarded
    return self.result(curvature, guarded, cs.vEgo)
