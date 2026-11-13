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
    self.approach_target = None
    self.recovery_confirm = 0.
    self.handover_confirm = 0.
    self.releasing = False
    self.reason = 'off'

  def _accel_cap(self, previous_accel, plan_dt):
    reserve_cap = self.cap if self.cap < 0. or self.releasing else None
    if self.approach_target is None or previous_accel is None or not math.isfinite(previous_accel):
      return reserve_cap
    # Start from the command already sent, including when it was positive.
    approach_cap = max(self.approach_target, previous_accel - .4 * plan_dt)
    return approach_cap if reserve_cap is None else min(reserve_cap, approach_cap)

  def update(self, lead, speed, follow, stop_distance, stamp, now, enabled,
             desired_distance=None, planned_accel=None, *, previous_accel=None, plan_dt=.05):
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
      return self._accel_cap(previous_accel, plan_dt)
    dt = stamp-self.stamp if self.stamp is not None else 0.
    continuous = (self.identity == identity and .01 <= dt <= .2 and
                  abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative_speed)*dt) <= 1.5)
    self.stamp, self.identity = stamp, identity
    self.distance, self.relative_speed = lead.dRel, lead.vRel
    if not continuous:
      # A new/cut-in target must establish its own history. MPC remains in charge.
      self.confirm, self.cap, self.recovery_confirm, self.handover_confirm = 0., 0., 0., 0.
      self.approach_target = None
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
    within_closing_range = lead.dRel < reserve + 2. * closing + 5.
    if closing > .6 and lead.aLeadK < .3 and within_closing_range:
      target = -min(.4, max(.12, .1*closing))
    if deficit > 1. and lead.vRel < .5:
      # Restore a moving time gap even when lead speed temporarily equals ours.
      target = min(target, -min(1.6, (closing+deficit/2.5)/1.4))
    # Match a slowing lead earlier, before the remaining gap demands hard braking.
    # Fade the extra request at crawl speed and leave the final stop to MPC.
    self.approach_target = None
    # A close, already-confirmed lead can start braking before closing speed builds.
    queue_approach = (speed <= 5.5 and .15 <= closing < .45 and
                      lead.aLeadK <= -.2 and lead.dRel <= reserve + 1.5)
    normal_approach = closing >= .45 and lead.aLeadK <= -.25 and within_closing_range
    if (lead.radar and lead.radarTrackId >= 0 and lead.modelProb >= .9 and abs(lead.dPath) <= 1. and
        2.5 < speed <= 40. / 3.6 and lead.vLead > .5 and (queue_approach or normal_approach)):
      approach_weight = min(1., (speed - 2.5) / 1.5, lead.vLead - .5)
      approach_decel = min(1., -.55 * lead.aLeadK + .30 * closing)
      self.approach_target = -approach_weight * approach_decel
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
    # Partial handover only relaxes this extra cap toward nonpositive MPC.
    # The full MPC distance objective remains mandatory for positive recovery.
    opening = (desired_distance is not None and planned_accel is not None and
               math.isfinite(desired_distance) and math.isfinite(planned_accel) and
               lead.vRel >= .5 and lead.aLeadK >= -.2 and
               lead.dRel >= max(stop_distance+3., .7*speed) and
               planned_accel >= -.8 and
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
    return self._accel_cap(previous_accel, plan_dt)


def apply_lead_gap_cap(accel, velocity, jerk, cap, speed, action_time, previous_accel, dt):
  if cap is None or cap >= accel:
    return accel, velocity, jerk
  # Keep velocity PID/feedforward consistent with the added acceleration ceiling.
  velocity = min(velocity, max(0., speed+max(0., action_time)*cap))
  return cap, velocity, min(jerk, (cap-previous_accel)/dt)


def brake_light_accel_cap(observation, lead, speed, now, previous_accel, dt):
  # Positive acceleration only: camera evidence never requests braking.
  if (not observation.braking or not lead.present or not lead.radar or
      lead.radarTrackId < 0 or lead.radarTrackId != observation.radarTrackId or
      not 0 <= now - observation.frameMonoTime * 1e-9 <= .25 or
      not all(math.isfinite(x) for x in (speed, lead.dRel, lead.vRel, lead.vLead, lead.aLeadK,
                                        lead.modelProb, lead.dPath, observation.distance, previous_accel, dt)) or
      not 1. <= speed <= 20. / 3.6 or not 3. <= lead.dRel <= 14. or
      abs(lead.dRel - observation.distance) > 1.5 or lead.modelProb < .9 or abs(lead.dPath) > 1. or
      lead.vLead <= .4 or lead.vRel > .2 or lead.aLeadK > .2 or
      (lead.vRel > -.15 and lead.aLeadK > -.15)):
    return None
  return max(0., previous_accel - .6 * dt)


class NormalStopGap:
  """Comfort stop assistance for a continuous radar lead."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.identity = self.stamp = None
    self.distance = self.relative = self.confirm = 0.
    self.armed = self.stopping = False
    self.target_gap = 2.5
    self.cap = None

  def update(self, lead, speed, planned, stamp, now, enabled):
    self.stopping = False
    if (not enabled or not all(math.isfinite(x) for x in
        (speed,planned,stamp,now,lead.dRel,lead.vRel,lead.vLead,lead.modelProb,lead.dPath)) or
        not 0.<=now-stamp<=.2 or not lead.present or not lead.radar or lead.radarTrackId<0 or
        lead.modelProb<.9 or abs(lead.dPath)>1. or not 0.<=speed<15. or
        not 1.<lead.dRel<80. or abs(lead.vLead-speed-lead.vRel)>1.5):
      self.reset()
      return None
    if self.stamp==stamp:
      self.stopping=bool(self.armed and self.cap is not None and speed<.1 and lead.vLead<.2 and lead.dRel <= max(1.8, self.target_gap - .3))
      return self.cap
    dt=stamp-self.stamp if self.stamp is not None else 0.
    continuous=(self.identity==lead.radarTrackId and .01<=dt<=.2 and
                abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative)*dt)<1.)
    if not continuous:
      self.reset()
    self.stamp,self.identity=stamp,lead.radarTrackId
    self.distance,self.relative=lead.dRel,lead.vRel
    if not continuous:
      return None
    self.confirm+=dt
    if self.confirm>=.5 and speed>=3. and lead.dRel>6.:
      self.armed=True
    if not self.armed or lead.vLead > .5:
      self.cap = None
      self.target_gap = 2.5
      return None
    # Use the extra space only if gentle braking can still reach a shorter gap.
    # Keep the chosen gap for this stop so slowing down cannot move the goal back.
    available = lead.dRel - .25 * speed
    if .25 < speed <= 2.5 and -2.5 <= planned < 0. and lead.vRel < -.15:
      # Do not demand extra braking solely to keep 2.5 m when the original
      # plan can stop within the allowed shorter gap. Bound this calculation;
      # genuinely stronger emergency braking still takes priority downstream.
      approach_decel = max(.9, min(1.8, -planned))
      comfortable_gap = available - speed * speed / (2. * approach_decel)
      if 1.8 <= comfortable_gap < self.target_gap:
        self.target_gap = 2.0 if comfortable_gap >= 2.0 else 1.8
    room = max(.3, available - self.target_gap)
    target = -min(2.5, speed * speed / (2. * room))
    if speed < .1 and lead.dRel <= max(1.8, self.target_gap - .3) and lead.vLead < .2:
      target=min(target,-.5)
      self.stopping=True
    if self.cap is None:
      self.cap=min(0.,planned)
    # Hand back unnecessary extra braking smoothly once the shorter target
    # fits the original plan; never weaken the original MPC command itself.
    release_rate = 1.5 if self.target_gap < 2.5 and target >= planned else .8
    rate = .8 if target < self.cap else release_rate
    self.cap+=max(-rate*dt,min(rate*dt,target-self.cap))
    return self.cap
