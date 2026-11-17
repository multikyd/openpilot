"""A bounded right preference from a painted centre line and verified right clearance."""
import math
import numpy as np


class SingleLaneReference:
  def __init__(self):
    self.reset()

  def reset(self):
    self.stamp = self.yellow_stamp = self.width_stamp = self.context_stamp = self.left_stamp = None
    self.width = None
    self.confirm = self.shift = self.room = 0.
    self.supported = False
    self.reason = 'off'
    self.stations = self.target_y = self.bound_margin = None

  @staticmethod
  def samples(line, stations):
    x, y = np.asarray(line.x, dtype=float), np.asarray(line.y, dtype=float)
    if (len(x) < 8 or len(x) != len(y) or not np.all(np.isfinite(x)) or
        not np.all(np.isfinite(y)) or np.any(np.diff(x) <= 0.) or
        x[0] > .5 or x[-1] < stations[-1]):
      raise ValueError('invalid local geometry')
    return np.interp(stations, x, y)

  def update(self, model, cs, curvature, stamp, now, *, enabled, path=None):
    dt = .05
    try:
      if (not enabled or not all(math.isfinite(v) for v in (stamp, now, curvature, cs.vEgo)) or
          not 0. <= now-stamp <= .2 or not 2. <= cs.vEgo <= 60./3.6 or
          abs(curvature) >= .0125 or abs(curvature)*cs.vEgo**2 >= 1.2 or
          cs.steeringPressed or cs.leftBlinker or cs.rightBlinker or getattr(cs, 'rightBlindspot', False) or
          str(model.meta.desire) not in ('none', '0') or str(model.meta.laneChangeState) not in ('off', '0')):
        self.reset()
        return 0.
      if stamp == self.stamp:
        return self.shift
      dt = .05 if self.stamp is None else stamp-self.stamp
      if not .015 <= dt <= .15:
        self.reset()
        dt = .05
      self.stamp = stamp
      self.supported = False
      probs, stds = list(model.laneLineProbs), list(model.laneLineStds)
      space, edge_std = float(model.meta.laneWidthRight), float(model.roadEdgeStds[1])
      if (len(probs) != 4 or len(stds) != 4 or
          not all(math.isfinite(v) for v in (*probs, *stds, space, edge_std)) or
          any(not 0. <= p <= 1. for p in probs) or any(s < 0. for s in stds) or
          space < 0. or space >= 1.8 or probs[3] >= .15):
        raise ValueError('right neighbor or invalid confidence')
      stations = np.linspace(0., max(6., min(12., cs.vEgo+2.)), 9)
      left = self.samples(model.laneLines[1], stations)
      right = self.samples(model.laneLines[2], stations)
      predicted = self.samples(model.position, stations) if path is None else np.interp(stations, path[:,0], path[:,1])
      edge = self.samples(model.roadEdges[1], stations)
      if not np.all(np.isfinite(predicted)) or abs(predicted[0]) > .8:
        raise ValueError('invalid path')
      strong_left = probs[1] >= .65 and stds[1] < .3
      if strong_left:
        self.left_stamp = stamp
      recent_left = self.left_stamp is not None and 0. <= stamp-self.left_stamp <= .75
      maintained_left = recent_left and probs[1] >= .55 and stds[1] < .45
      strong_left = strong_left or maintained_left
      yellow = 20 <= getattr(cs, 'leftLaneLine', 0) < 30
      # Opposing lanes/road edge on the left do not count as same-direction lanes.
      if yellow and strong_left:
        self.yellow_stamp = stamp
      recent_yellow = self.yellow_stamp is not None and 0. <= stamp-self.yellow_stamp <= 2.
      width = right-left
      near_width = width[stations <= 5.]
      strong_right = probs[2] >= .65 and stds[2] < .3
      # A painted, moderately confident right line may maintain a previously
      # verified width when the independent road edge and left line agree.
      painted_right = 10 <= getattr(cs, 'rightLaneLine', 0) < 30
      recent_width = self.width_stamp is not None and 0. <= stamp-self.width_stamp <= 5.
      maintained_right = (recent_width and recent_yellow and strong_left and painted_right and
                          probs[2] >= .25 and stds[2] < .4 and 0. <= edge_std < .6 and
                          np.max(np.abs(near_width-self.width)) < .2)
      strong_right = strong_right or maintained_right
      if (strong_left and strong_right and np.all((near_width > 2.2) & (near_width < 3.5)) and np.ptp(near_width) < .4):
        self.width = float(np.median(near_width))
        self.width_stamp = stamp
      recent_width = self.width_stamp is not None and 0. <= stamp-self.width_stamp <= 5.
      if not (strong_left and recent_yellow and recent_width and 0. <= edge_std < .8):
        raise ValueError('missing centre line, recent width or right edge')
      if np.any(left >= -.3) or np.any(edge-left < 2.2) or np.any(edge-left > 4.8):
        raise ValueError('invalid corridor')
      # Anchor a brief right-line dropout to the recent confirmed width, never to an untrusted line.
      reference_right = right if strong_right else left+self.width
      bound = np.minimum(reference_right, edge-.25*edge_std)
      margin = 1.20 + .25*(stds[2] if strong_right else .3)
      room = self.room = max(0., float(np.min(bound-margin-predicted)))
      centre = (left+reference_right)*.5
      self.stations, self.target_y, self.bound_margin = stations, centre+.06, bound-margin
      near = stations <= 5.
      error = min(float(centre[0]+.06), float(np.median(centre[near]+.06-predicted[near])))
      self.context_stamp = stamp
      self.confirm = min(1., self.confirm+dt)
      target = min(.25, max(0., error), room) if self.confirm >= .3 else 0.
      target *= float(np.interp(abs(curvature), [.004, .0125], [1., 0.]))
      self.shift += float(np.clip(target-self.shift, -.08*dt, .04*dt))
      self.shift = min(self.shift, room)
      self.supported = self.confirm >= .3
      self.reason = 'single tracking' if strong_right else 'single width hold'
      return self.shift
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
      # Do not retain steering into uncertain roadside space.
      retain_context = (str(exc) == 'missing centre line, recent width or right edge' and
                        self.context_stamp is not None and 0. <= stamp-self.context_stamp <= 2. and
                        self.width_stamp is not None and 0. <= stamp-self.width_stamp <= 5. and
                        self.yellow_stamp is not None and 0. <= stamp-self.yellow_stamp <= 2.)
      self.confirm = max(0., self.confirm-.5*dt) if retain_context else 0.
      self.shift = 0.
      self.supported = False
      self.reason = str(exc)
      return 0.

  def for_path(self, path):
    """Apply the same reference to a calibrated lane path without stacking offsets."""
    if not self.supported:
      return 0.
    try:
      path = np.asarray(path, dtype=float)
      if (path.ndim != 2 or path.shape[1] < 2 or len(path) < 3 or
          not np.all(np.isfinite(path)) or np.any(np.diff(path[:,0]) <= 0.) or
          path[0,0] > .5 or path[-1,0] < self.stations[-1]):
        return 0.
      predicted = np.interp(self.stations, path[:,0], path[:,1])
      near = self.stations <= 5.
      error = float(np.median(self.target_y[near]-predicted[near]))
      room = float(np.min(self.bound_margin-predicted))
      return max(0., min(self.shift, error, room))
    except (IndexError, TypeError, ValueError):
      return 0.
