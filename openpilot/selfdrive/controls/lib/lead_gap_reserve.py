import math


class LeadGapReserve:
  """Extra comfort cap; hand back to MPC smoothly without overriding its brakes."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.stamp = None
    self.identity = None
    self.distance = self.relative_speed = 0.
    self.confirm = 0.
    self.cap = 0.
    self.recovery_confirm = 0.
    self.handover_confirm = 0.
    self.releasing = False
    self.reason = 'off'

  def update(self, lead, speed, follow, stop_distance, stamp, now, enabled,
             desired_distance=None, planned_accel=None):
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
      return self.cap if self.cap < 0. or self.releasing else None
    dt = stamp-self.stamp if self.stamp is not None else 0.
    continuous = (self.identity == identity and .01 <= dt <= .2 and
                  abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative_speed)*dt) <= 1.5)
    self.stamp, self.identity = stamp, identity
    self.distance, self.relative_speed = lead.dRel, lead.vRel
    if not continuous:
      # A new/cut-in target must establish its own history. MPC remains in charge.
      self.confirm, self.cap, self.recovery_confirm, self.handover_confirm = 0., 0., 0., 0.
      self.releasing, self.reason = False, 'confirming'
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
    # Release only with a stable opening gap AND room according to the current
    # MPC lead-distance objective. Relative speed alone is not sufficient.
    room_confirmed = (desired_distance is not None and planned_accel is not None and
                      math.isfinite(desired_distance) and math.isfinite(planned_accel) and
                      lead.vRel >= 1. and lead.aLeadK >= -.2 and
                      lead.dRel >= max(stop_distance+3., .5*speed) and
                      lead.dRel >= max(0., desired_distance)+max(1., .1*speed))
    self.recovery_confirm = self.recovery_confirm+dt if room_confirmed else 0.
    recovering = self.recovery_confirm >= .25
    # Stop adding comfort braking once the lead is pulling away and MPC has
    # already relaxed its brakes. Full room is still required before allowing
    # a positive cap: this phase can only return toward MPC's negative/zero
    # command, never manufacture acceleration into a short gap.
    opening = (desired_distance is not None and planned_accel is not None and
               math.isfinite(desired_distance) and math.isfinite(planned_accel) and
               lead.vRel >= .8 and lead.aLeadK >= -.2 and
               lead.dRel >= max(stop_distance+3., .7*speed) and
               desired_distance <= reserve and planned_accel >= -.8 and
               target == 0.)
    self.handover_confirm = self.handover_confirm+dt if opening else 0.
    handing_over = self.handover_confirm >= .2
    if not recovering:
      self.releasing = False
      self.cap = min(0., self.cap)
    elif self.cap < 0.:
      self.releasing = True
    if self.releasing:
      # Continue the cap through zero until it meets the requested acceleration,
      # avoiding an abrupt jump from the last negative cap to full acceleration.
      target = max(0., planned_accel)
    elif handing_over:
      target = min(0., planned_accel)
    self.reason = ('recover' if self.releasing else 'handover' if handing_over and self.cap < target
                   else 'reserve' if target < 0. else 'release' if self.cap < 0. else 'off')
    # Existing stronger MPC braking always wins in apply_lead_gap_cap.
    rate = .6 if target < self.cap else .9 if self.releasing else 1.2 if handing_over else .35
    self.cap += max(-rate*dt, min(rate*dt, target-self.cap))
    if self.releasing and self.cap >= target:
      self.cap, self.releasing, self.reason = 0., False, 'off'
    if abs(self.cap) < 1e-5:
      self.cap = 0.
    return self.cap if self.cap < 0. or self.releasing else None


def apply_lead_gap_cap(accel, velocity, jerk, cap, speed, action_time, previous_accel, dt):
  if cap is None or cap >= accel:
    return accel, velocity, jerk
  # Keep velocity PID/feedforward consistent with the added acceleration ceiling.
  velocity = min(velocity, max(0., speed+max(0., action_time)*cap))
  return cap, velocity, min(jerk, (cap-previous_accel)/dt)
