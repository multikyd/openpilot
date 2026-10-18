import math


class LeadGapReserve:
  """A bounded extra deceleration ceiling, never a brake-release override."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.stamp = None
    self.identity = None
    self.distance = self.relative_speed = 0.
    self.confirm = 0.
    self.cap = 0.

  def update(self, lead, speed, follow, stop_distance, stamp, now, enabled):
    values = (speed, follow, stop_distance, stamp, now, lead.dRel, lead.vRel,
              lead.vLead, lead.aLeadK, lead.modelProb, lead.dPath)
    if (not enabled or not all(math.isfinite(v) for v in values) or
        not 0. <= now-stamp <= .2 or not lead.present or
        lead.modelProb < (.8 if lead.radar else .95) or abs(lead.dPath) > 1.5 or
        not 2. <= speed <= 30. or not 1. < lead.dRel < 100. or lead.vLead < 0. or
        not .3 <= follow <= 2.5 or not 2. <= stop_distance <= 10. or
        abs(lead.vLead-speed-lead.vRel) > 1.5):
      self.reset()
      return None
    identity = ('radar', lead.radarTrackId) if lead.radar else ('vision',)
    if self.stamp is not None and stamp == self.stamp:
      return self.cap if self.cap < 0. else None
    dt = stamp-self.stamp if self.stamp is not None else 0.
    continuous = (self.identity == identity and .01 <= dt <= .2 and
                  abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative_speed)*dt) <= 1.5)
    self.stamp, self.identity = stamp, identity
    self.distance, self.relative_speed = lead.dRel, lead.vRel
    if not continuous:
      # A new/cut-in target must establish its own history. MPC remains in charge.
      self.confirm, self.cap = 0., 0.
      return None
    self.confirm = min(1., self.confirm+dt)
    if self.confirm < .5:
      return None

    closing = max(0., -lead.vRel)
    # Small moving reserve while a lead is already slowing; fades at low speed.
    extra_follow = .15*min(1., speed/4.) if closing > .4 and lead.aLeadK < -.2 else 0.
    reserve = stop_distance + (max(.9, min(1.6, follow))+extra_follow)*speed
    deficit = reserve-lead.dRel
    target = 0.
    if closing > .6 and lead.aLeadK < .3 and lead.dRel < reserve+2.*closing+5.:
      target = -min(.4, max(.12, .1*closing))
    if deficit > 1. and lead.vRel < .5:
      # Restore a moving time gap even when lead speed temporarily equals ours.
      target = min(target, -min(1.6, (closing+deficit/2.5)/1.4))
    # Extra comfort action builds and releases gently. Stronger MPC braking wins.
    rate = .6 if target < self.cap else .35
    self.cap += max(-rate*dt, min(rate*dt, target-self.cap))
    if abs(self.cap) < 1e-5:
      self.cap = 0.
    return self.cap if self.cap < 0. else None


def apply_lead_gap_cap(accel, velocity, jerk, cap, speed, action_time, previous_accel, dt):
  if cap is None or cap >= accel:
    return accel, velocity, jerk
  # Keep velocity PID/feedforward consistent with the added acceleration ceiling.
  velocity = min(velocity, max(0., speed+max(0., action_time)*cap))
  return cap, velocity, min(jerk, (cap-previous_accel)/dt)
