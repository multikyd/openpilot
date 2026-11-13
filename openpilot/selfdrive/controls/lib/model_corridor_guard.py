"""A bounded corridor target for model-mode steering, before command filtering.

Position preference and boundary protection share one output. Uncertain data
can briefly support a previously trusted line, never a stale invented corridor.
"""
import math

import numpy as np


class ModelCorridorGuard:
  def __init__(self):
    self.reset()

  def reset(self):
    self.stamp = self.width_stamp = self.context_stamp = None
    self.width = self.confirm = self.error = 0.
    self.position_accel = self.boundary_accel = self.accel = 0.
    self.line_times = [None, None]
    self.boundaries = [None, None]
    self.line_confirm = [0., 0.]
    self.context_confirm = 0.
    self.rightmost = False
    self.corridor_type = 'unknown'
    self.target_offset = 0.

  @staticmethod
  def sample(path, stations):
    x, y = np.asarray(path.x, dtype=float), np.asarray(path.y, dtype=float)
    reach = np.flatnonzero(x >= stations[-1])
    if len(x) < 8 or len(x) != len(y) or not len(reach) or x[0] > .5:
      raise ValueError('incomplete local geometry')
    end = int(reach[0])+1
    if not np.all(np.isfinite(x[:end])) or not np.all(np.isfinite(y[:end])) or np.any(np.diff(x[:end]) <= 0.):
      raise ValueError('invalid local geometry')
    return np.interp(stations, x[:end], y[:end])

  @staticmethod
  def step(previous, target, dt, jerk):
    delta = (1.-math.exp(-dt/.4))*(target-previous)
    return previous+float(np.clip(delta,-jerk*dt,jerk*dt))

  def result(self, curvature, guarded, cs):
    old = (guarded-curvature)*cs.vEgo**2
    boundary = self.boundary_accel
    # Boundary protection wins over a conflicting position preference. Existing
    # and new corrections in the same direction use the stronger, not their sum.
    delta = boundary if abs(boundary) > .005 else self.position_accel
    if old*delta < 0. or abs(old) > abs(delta):
      delta = old
    if (delta > 0. and getattr(cs,'rightBlindspot',False) or
        delta < 0. and getattr(cs,'leftBlindspot',False)):
      delta = old
    if curvature*delta < 0. and abs(curvature) > .001:
      delta = math.copysign(min(abs(delta),.5*abs(curvature)*cs.vEgo**2),delta)
    delta = float(np.clip(delta,-.6,.6))
    self.accel = delta
    return float(curvature+delta/cs.vEgo**2)

  def update(self, model, cs, curvature, guarded, stamp, now, *, preference=0.,
             camera_offset=0., actuator_delay=.4, enabled=True):
    values = (curvature,guarded,stamp,now,preference,camera_offset,actuator_delay,cs.vEgo)
    if (not enabled or not all(math.isfinite(v) for v in values) or
        not 0. <= now-stamp <= .2 or not 3. <= cs.vEgo <= 30. or
        abs(curvature) > .04 or abs(camera_offset) > .2 or abs(preference) > .1 or
        cs.steeringPressed or cs.leftBlinker or cs.rightBlinker or
        str(model.meta.desire) != 'none' or str(model.meta.laneChangeState) != 'off'):
      self.reset()
      return guarded
    if self.stamp == stamp:
      return self.result(curvature,guarded,cs)
    dt = .05 if self.stamp is None else stamp-self.stamp
    if not .015 <= dt <= .15:
      self.reset()
      dt = .05
    self.stamp = stamp
    position_target = boundary_target = 0.
    try:
      probs = np.asarray(model.laneLineProbs,dtype=float)
      stds = np.asarray(model.laneLineStds,dtype=float)
      if (len(probs) != 4 or len(stds) != 4 or not np.all(np.isfinite([*probs,*stds])) or
          np.any(probs < 0.) or np.any(probs > 1.) or np.any(stds < 0.)):
        raise ValueError('invalid confidence')
      horizon = float(np.clip(cs.vEgo*(np.clip(actuator_delay,.1,.6)+.6),8.,18.))
      stations = [0.,horizon*.5,horizon]
      predicted = self.sample(model.position,stations)
      if abs(predicted[0]) > .3:
        raise ValueError('invalid path origin')
      lines, supported, strong = [], [], []
      for j, i in enumerate((1,2)):
        try:
          line = self.sample(model.laneLines[i],stations)+camera_offset
          sane = line[0] < -.2 if j == 0 else line[0] > .2
          jump = (self.boundaries[j] is not None and
                  abs(line[0]-self.boundaries[j]) > .25+2.*dt)
          self.boundaries[j] = float(line[0])
          reliable = bool(sane and not jump and .65 <= probs[i] <= 1. and 0. <= stds[i] < .35)
          recent = self.line_times[j] is not None and 0. <= stamp-self.line_times[j] <= .5
          usable = reliable or (sane and not jump and recent and probs[i] >= .45 and 0. <= stds[i] < .5)
          if reliable:
            self.line_times[j] = stamp
          self.line_confirm[j] = min(.5,self.line_confirm[j]+dt) if usable else 0.
        except (IndexError,TypeError,ValueError):
          line,reliable,usable = None,False,False
          self.line_confirm[j] = 0.
        lines.append(line)
        strong.append(reliable)
        supported.append(usable)
      width_recent = self.width_stamp is not None and 0. <= stamp-self.width_stamp <= 1.5
      if all(strong):
        widths = lines[1]-lines[0]
        if np.all((widths >= 2.4)&(widths <= 4.3)) and np.ptp(widths) < .8:
          observed_width = float(np.median(widths))
          self.width = observed_width if not width_recent else self.width+min(1.,dt/.4)*(observed_width-self.width)
          self.width_stamp = stamp
          width_recent = True
      # Only current observations are used for a boundary constraint. A short
      # confidence dip may use current geometry with a recent trusted identity.
      hazards = []
      for j, i in enumerate((1,2)):
        if not supported[j]:
          continue
        clearance = predicted-lines[j] if j == 0 else lines[j]-predicted
        # Two consistent observations can protect an already-close boundary;
        # a more distant preview still needs the ordinary confirmation window.
        confirmation = .1 if clearance[0] < 1.1 and strong[j] else .15
        if self.line_confirm[j]+1e-9 < confirmation:
          continue
        width = self.width if width_recent else 2.8
        margin = min(1.30,.5*width-.1)
        penetration = max(0.,margin-float(np.min(clearance))-.03)
        confidence = min(float(np.interp(probs[i],[.45,.85],[0.,1.])),
                         float(np.interp(stds[i],[.15,.5],[1.,.25])))
        if strong[j] and clearance[0] < 1.1:
          confidence = max(confidence,.65)
        outward = 1. if j == 0 else -1.
        target = outward*min(.6,2.*min(penetration,.9)*cs.vEgo**2/horizon**2)*confidence
        hazards.append((penetration,target))
      if hazards:
        boundary_target = max(hazards,key=lambda v:v[0])[1]

      wl,wr = float(model.meta.laneWidthLeft),float(model.meta.laneWidthRight)
      if not math.isfinite(wl) or not math.isfinite(wr):
        raise ValueError('invalid lane context')
      outer = (0. <= wr < 1.8 and probs[3] < .15)
      edge_gaps = [None,None]
      for j in (0,1):
        if supported[j] and len(model.roadEdges) > j and len(model.roadEdgeStds) > j:
          if 0. <= model.roadEdgeStds[j] < .6:
            try:
              edge = self.sample(model.roadEdges[j],stations)+camera_offset
              edge_gaps[j] = float(lines[j][0]-edge[0] if j == 0 else edge[0]-lines[j][0])
            except (IndexError,TypeError,ValueError):
              pass
      # A yellow left edge on a ramp is not proof of an opposing traffic lane.
      # Two nearby road edges mean an isolated corridor: no right preference.
      isolated = all(gap is not None and 0. <= gap < 1.5 for gap in edge_gaps)
      left_adjacent = wl > 2. and probs[0] > .3
      yellow_center = 20 <= getattr(cs,'leftLaneLine',0) < 30
      opposing_road = yellow_center and edge_gaps[0] is not None and edge_gaps[0] > 2.
      context = outer and not isolated and (left_adjacent or opposing_road)
      explicit_other_lane = wr >= 2.2 or probs[3] >= .3
      self.context_confirm = min(1.,self.context_confirm+dt) if context else max(0.,self.context_confirm-dt)
      if explicit_other_lane or isolated:
        self.context_stamp = None
        self.context_confirm = 0.
      elif context and self.context_confirm >= .5:
        self.context_stamp = stamp
      self.rightmost = bool(self.context_stamp is not None and 0. <= stamp-self.context_stamp <= .75)
      self.corridor_type = 'isolated' if isolated else 'rightmost' if self.rightmost else 'ordinary'
      self.target_offset = 0.
      if width_recent and any(supported):
        source = 0 if supported[0] else 1
        left = lines[0] if supported[0] else lines[1]-self.width
        right = lines[1] if supported[1] else lines[0]+self.width
        if self.rightmost and self.width <= 3.35:
          offset = .04*float(np.interp(abs(curvature),[.003,.008],[1.,0.]))
        elif isolated:
          offset = 0.
        else:
          offset = float(np.clip(preference,-.08,.08))
        low = left+1.15+.2*np.clip(stds[1],0.,.5)
        high = right-1.15-.2*np.clip(stds[2],0.,.5)
        if np.all(low <= high):
          wanted = np.clip((left+right)*.5+offset,low,high)
          # The observed current position and model preview must agree on the
          # recovery direction; do not steer against a model already returning.
          current_error = float(wanted[0]-predicted[0])
          preview_error = float(np.median(wanted-predicted))
          error = math.copysign(min(abs(current_error),abs(preview_error)),current_error) if current_error*preview_error > 0. else 0.
          self.error = error
          self.target_offset = offset
          deadband = .02 if abs(offset) > .005 else .08
          self.confirm = min(1.,self.confirm+dt) if abs(error) > deadband else 0.
          room = float(np.min(right-predicted)) if error > 0. else float(np.min(predicted-left))
          room_margin = 1.17+.2*float(np.clip(stds[2 if error > 0. else 1],0.,.5))
          if self.confirm >= .4 and room > room_margin:
            confidence = float(np.interp(probs[source+1],[.45,.9],[0.,1.]))
            position_target = math.copysign(min(.2,2.*max(0.,abs(error)-deadband*.5)*cs.vEgo**2/horizon**2)*confidence,error)
        else:
          self.confirm = 0.
      else:
        self.confirm = 0.
    except (AttributeError,IndexError,TypeError,ValueError):
      self.confirm = 0.
      self.rightmost = False
      self.target_offset = 0.
    # A discontinuity never starts a correction. Existing correction decays
    # under bounded jerk instead of repeatedly switching a steering impulse.
    self.position_accel = self.step(self.position_accel,position_target,dt,.25)
    self.boundary_accel = self.step(self.boundary_accel,boundary_target,dt,.5)
    return self.result(curvature,guarded,cs)
