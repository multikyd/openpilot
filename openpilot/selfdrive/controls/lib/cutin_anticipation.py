import math

import numpy as np


class CutInAnticipation:
  """Yield gently to persistent inward side tracks; never invent a lead obstacle."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.history = {}
    self.last_sample = None
    self.last_now = None
    self.limit = None
    self.stage = 'off'
    self.track_id = None

  def update(self, radar, model, speed, normal_accel, curvature, sample_time, now, enabled, current_accel=None):
    values = (speed, normal_accel, curvature, sample_time, now)
    if (not enabled or not all(math.isfinite(x) for x in values) or
        (current_accel is not None and not math.isfinite(current_accel)) or not 1. < speed < 40. or
        not 0. <= now-sample_time <= .2 or abs(curvature) > .01 or abs(curvature)*speed**2 > 1.3):
      self.reset()
      return None
    if self.last_now is not None and not 0. <= now-self.last_now <= .25:
      self.reset()
    dt = min(.1, max(0., now-self.last_now)) if self.last_now is not None else .05
    self.last_now = now
    if self.last_sample is not None and sample_time <= self.last_sample:
      if sample_time < self.last_sample:
        self.reset()
        return None
      return min(normal_accel, self.limit) if self.limit is not None else None
    self.last_sample = sample_time

    if len(model.laneLines) != 4 or len(model.laneLineProbs) != 4 or len(model.laneLineStds) != 4:
      self.reset()
      return None
    left, right = model.laneLines[1], model.laneLines[2]
    probs = [float(model.laneLineProbs[i]) for i in (1, 2)]
    stds = [float(model.laneLineStds[i]) for i in (1, 2)]
    xs = np.asarray(left.x, dtype=float)
    lx, rx = np.asarray(left.y, dtype=float), np.asarray(right.y, dtype=float)
    rxs = np.asarray(right.x, dtype=float)
    if (len(xs) < 4 or len(xs) != len(lx) or len(xs) != len(rx) or len(xs) != len(rxs) or
        not all(np.isfinite(a).all() for a in (xs, lx, rx, rxs)) or not np.all(np.diff(xs) > 0.) or
        not np.allclose(xs, rxs, atol=.1) or
        not all(math.isfinite(x) for x in (*probs, *stds)) or min(probs) < .65 or max(stds) > .35):
      self.reset()
      return None

    horizon = float(np.interp(speed, [3., 15., 30.], [1.5, 2., 2.5]))
    target, chosen, stage = None, None, 'off'
    seen = set()
    self.history = {key: h for key, h in self.history.items() if now-h[-1][0] < .3}
    candidates = list(radar.leadsLeft)[:8]+list(radar.leadsRight)[:8]
    for lead in candidates:
      key = int(lead.radarTrackId)
      if key in seen:
        continue
      seen.add(key)
      vals = (lead.dRel, lead.yRel, lead.dPath, lead.vRel, lead.vLead, lead.vLat)
      if (not lead.present or not lead.radar or key < 0 or not all(math.isfinite(x) for x in vals) or
          not .5 < lead.dRel < min(80., max(20., speed*2.5)) or not .9 < abs(lead.dPath) < 4.5 or
          not 0. <= lead.vLead < 50. or abs(lead.vLead-speed-lead.vRel) > 2.5 or
          lead.vRel > 3. or abs(lead.vLat) > 1.8):
        self.history.pop(key, None)
        continue
      future_x = max(0., lead.dRel+lead.vRel*horizon)
      if not xs[0] <= min(lead.dRel, future_x) <= max(lead.dRel, future_x) <= xs[-1]:
        self.history.pop(key, None)
        continue
      lane_l = np.interp([lead.dRel, future_x], xs, lx)
      lane_r = np.interp([lead.dRel, future_x], xs, rx)
      width = lane_r-lane_l
      center = (lane_l+lane_r)/2.
      offset = lead.yRel+center[0]
      if (np.any(width < 2.6) or np.any(width > 4.4) or abs(width[1]-width[0]) > .5 or
          abs(offset-lead.dPath) > .6 or abs(center[1]-center[0]) > .4):
        self.history.pop(key, None)
        continue
      h = self.history.get(key, [])
      if h:
        elapsed = sample_time-h[-1][0]
        if (not 0. < elapsed <= .2 or offset*h[-1][1] <= 0. or
            abs(lead.dRel-h[-1][2]) > 2.+abs(lead.vRel)*elapsed or
            abs(offset-h[-1][1]) > .5+1.8*elapsed):
          h = []
      h.append((sample_time, offset, lead.dRel))
      h = [p for p in h if sample_time-p[0] <= .8]
      self.history[key] = h
      span = sample_time-h[0][0]
      inward = -math.copysign(1., offset)*lead.vLat
      observed = (abs(h[0][1])-abs(offset))/span if span > 0. else 0.
      if span < .35 or inward < .15 or observed < .08 or abs(h[0][1])-abs(offset) < .04:
        continue
      # Both measured transverse motion and observed path approach must agree.
      inward = min(inward, observed+.1)
      # Approximate combined half-width of two passenger vehicles, with margin.
      overlap_time = max(0., (abs(offset)-2.0)/inward)
      close_side = lead.dRel < max(8., speed*.5)
      prepare_horizon = 6. if close_side else horizon+1.
      room = max(4., speed*.95)
      entry_x = lead.dRel+lead.vRel*min(horizon, overlap_time)
      if overlap_time > prepare_horizon or entry_x < -1. or entry_x > room+max(2., speed*.2):
        continue
      candidate_target, candidate_stage = min(normal_accel, .3), 'yield'
      predicted_offset = lead.yRel+lead.vLat*horizon+center[1]
      if (span >= .6 and overlap_time <= horizon and abs(predicted_offset) < 2.0 and
          lead.vRel <= .5 and entry_x < room):
        severity = min(1., max(0., (room-entry_x)/max(room, 1.)))
        candidate_target, candidate_stage = -.15-.15*severity, 'coast'
      if target is None or candidate_target < target:
        target, chosen, stage = candidate_target, key, candidate_stage

    self.stage, self.track_id = stage, chosen
    if target is not None:
      if self.limit is None:
        self.limit = min(normal_accel, max(0., current_accel)) if current_accel is not None else normal_accel
      # Positive cap and negative cruise goal both change smoothly.
      self.limit += max(-.7*dt, min(.35*dt, target-self.limit))
    elif self.limit is not None:
      self.limit = min(normal_accel, self.limit+.35*dt)
      if self.limit >= normal_accel-1e-6:
        self.limit = None
    return min(normal_accel, self.limit) if self.limit is not None else None

