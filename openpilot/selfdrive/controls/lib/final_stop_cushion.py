"""A small brake taper during the final moving instant; standstill hold stays native."""
import math


class FinalStopCushion:
  def __init__(self):
    self.reset()

  def reset(self):
    self.identity = self.stamp = None
    self.distance = self.relative = self.confirm = 0.
    self.used = self.finished = False

  def update(self, lead, other, speed, standstill, command, previous, stamp, now,
             enabled, stopping, dt):
    values = (speed, command, previous, stamp, now, dt, lead.dRel, lead.vRel,
              lead.vLead, lead.modelProb, lead.dPath, lead.aLeadK)
    if (not enabled or not all(math.isfinite(v) for v in values) or
        not 0. < dt <= .05 or not 0. <= now-stamp <= .2 or
        not lead.present or not lead.radar or lead.radarTrackId < 0 or lead.modelProb < .95 or
        abs(lead.dPath) > .7 or not 0. <= speed <= 1.5 or abs(lead.vLead) >= .3 or
        not 1.9 < lead.dRel < 4.5 or abs(lead.vLead-speed-lead.vRel) > .8 or lead.aLeadK < -.8):
      self.reset()
      return command
    # Exact radar duplicates are harmless; independent/ambiguous close objects veto release.
    if other.present and not ((other.radar and other.radarTrackId == lead.radarTrackId) or
                              (not other.radar and other.radarTrackId < 0 and
                               all(math.isfinite(v) for v in (other.dRel, other.vRel, other.dPath)) and
                               abs(other.dRel-lead.dRel) < .75 and abs(other.vRel-lead.vRel) < .35 and
                               abs(other.dPath-lead.dPath) < .2)):
      if not math.isfinite(other.dRel) or other.dRel < lead.dRel+5.:
        self.reset()
        return command
    if self.stamp != stamp:
      elapsed = 0. if self.stamp is None else stamp-self.stamp
      same = (self.identity == lead.radarTrackId and .01 <= elapsed <= .2 and
              abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative)*elapsed) < .5)
      if not same:
        self.confirm = 0.
        self.used = self.finished = False
      else:
        self.confirm += elapsed
      self.identity, self.stamp = lead.radarTrackId, stamp
      self.distance, self.relative = lead.dRel, lead.vRel
    # Hyundai standstill becomes true below about 0.1 m/s, while the wheels
    # can still be moving. Keep cushioning this last crawl until near-zero
    # measured speed; a flag alone must not start the full hold build.
    if self.used and speed <= .015:
      self.finished = True
    if (self.finished or (standstill and speed <= .015) or not stopping or self.confirm < .5 or
        not .015 < speed <= .35 or not -1.3 < command < -.4 or previous <= -1.3):
      return command
    closing = max(speed, -lead.vRel, 0.)
    room = lead.dRel-1.8-.5*closing
    required = closing**2/(2.*max(.01,room))
    if room <= 0. or required > .45:
      return command
    floor = -max(.4, 1.3*required)
    self.used = True
    # Release by at most .6 m/s^3, never add positive propulsion. Native
    # stopping/StopReq/soft-hold paths resume unchanged once motion ends.
    return max(command, min(floor, previous+.6*dt))
