"""Bounded launch acceleration headroom behind the same confirmed stopped lead."""
import math


class LaunchFollow:
  def __init__(self):
    self.reset()

  def reset(self):
    self.previous = self.stamp = self.identity = None
    self.distance = self.relative = self.stopped_time = 0.
    self.armed = False
    self.started = None
    self.extra = 0.

  def update(self, *, now, stamp, enabled, can_accelerate, speed, target_speed, normal_max,
             lead, other, follow):
    values = (now, stamp, speed, target_speed, normal_max, follow,
              lead.dRel, lead.vRel, lead.vLead, lead.aLeadK, lead.modelProb, lead.dPath)
    if (not enabled or not all(math.isfinite(v) for v in values) or
        not 0. <= now-stamp <= .2 or not 0. <= speed <= 50./3.6 or normal_max <= 0. or
        not .3 <= follow <= 2.5 or not lead.present or not lead.radar or lead.radarTrackId < 0 or
        lead.modelProb < .95 or abs(lead.dPath) > .7 or not 1.8 <= lead.dRel < 80. or
        abs(lead.vLead-speed-lead.vRel) > 1.5 or lead.aLeadK < -.5):
      self.reset()
      return normal_max
    if other.present and not ((other.radar and other.radarTrackId == lead.radarTrackId) or
                              (not other.radar and other.radarTrackId < 0 and
                               all(math.isfinite(v) for v in (other.dRel, other.vRel, other.dPath)) and
                               abs(other.dRel-lead.dRel) < .75 and abs(other.vRel-lead.vRel) < .35 and
                               abs(other.dPath-lead.dPath) < .2)):
      # A distinct close obstacle vetoes recovery; no target switching shortcut.
      if not math.isfinite(other.dRel) or other.dRel < lead.dRel+5.:
        self.reset()
        return normal_max
    dt = 0. if self.previous is None else now-self.previous
    self.previous = now
    if not 0. <= dt <= .2:
      self.reset()
      return normal_max
    if self.stamp != stamp:
      elapsed = 0. if self.stamp is None else stamp-self.stamp
      same = (self.identity == lead.radarTrackId and .01 <= elapsed <= .2 and
              abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative)*elapsed) < .75)
      if not same:
        self.stopped_time = self.extra = 0.
        self.armed = False
        self.started = None
      elif speed < .25 and abs(lead.vLead) < .3:
        if self.started is not None:
          self.started = None
          self.armed = False
          self.stopped_time = self.extra = 0.
        self.stopped_time += elapsed
        self.armed |= self.stopped_time >= .5
      else:
        self.stopped_time = 0.
      self.stamp, self.identity = stamp, lead.radarTrackId
      self.distance, self.relative = lead.dRel, lead.vRel
    if self.armed and self.started is None and lead.vLead > .8 and lead.vRel > .35:
      self.started = now
    if self.started is not None and now-self.started > 8.:
      self.armed = False
    opening = (self.armed and self.started is not None and can_accelerate and
               target_speed > speed+.5 and lead.vRel > .35 and
               lead.dRel > max(4., 2.5+follow*speed))
    if not can_accelerate:
      self.extra = 0.
    elif opening:
      self.extra = min(.35, max(0., 1.8-normal_max), self.extra+.2*dt)
    else:
      self.extra = max(0., self.extra-.35*dt)
    self.extra = min(self.extra, max(0., 1.8-normal_max))
    return normal_max+self.extra
