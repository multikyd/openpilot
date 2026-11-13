"""Soft distance targets for easing behind a moving lead and settling to a stop.

Only the MPC cost reference changes. Follow time, braking/danger constraints,
stop distance and the predicted lead/stop obstacles retain their original values.
"""
import math
import numpy as np


class StopApproach:
  def __init__(self):
    self.reset()

  def reset(self):
    self.identity = self.stamp = None
    self.distance = self.relative_speed = 0.
    self.confirm = self.easing_confirm = 0.
    self.brake_age = 10.
    self.bias = self.stop_bias = 0.
    self.approached_from_distance = False

  @staticmethod
  def same_lead(lead, other):
    return (lead.present and other.present and lead.radar and other.radar and
            lead.radarTrackId >= 0 and lead.radarTrackId == other.radarTrackId and
            math.isfinite(other.dRel) and math.isfinite(other.vRel) and
            abs(lead.dRel-other.dRel) < .5 and abs(lead.vRel-other.vRel) < .3)

  def update(self, lead, other, speed, accel, desired_distance, stop_distance, stamp, now, enabled):
    values = (speed, accel, desired_distance, stop_distance, stamp, now,
              lead.dRel, lead.vRel, lead.vLead, lead.aLeadK, lead.modelProb, lead.dPath)
    if (not enabled or not all(math.isfinite(x) for x in values) or not 0. <= now-stamp <= .2 or
        not lead.present or not lead.radar or lead.radarTrackId < 0 or lead.modelProb < .9 or
        abs(lead.dPath) > 1.2 or not 2. < lead.dRel < 100. or lead.vLead < -.1 or
        not 2. <= stop_distance <= 10. or abs(lead.vLead-speed-lead.vRel) > 1.5 or
        (other.present and not self.same_lead(lead, other) and (not math.isfinite(other.dRel) or other.dRel < lead.dRel+5.))):
      self.reset()
      return 0.
    if self.stamp == stamp:
      return -self.stop_bias if self.stop_bias > 0. else self.bias
    dt = stamp-self.stamp if self.stamp is not None else 0.
    continuous = (self.identity == lead.radarTrackId and .01 <= dt <= .2 and
                  abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative_speed)*dt) <= 1.5)
    self.identity, self.stamp = lead.radarTrackId, stamp
    self.distance, self.relative_speed = lead.dRel, lead.vRel
    if not continuous:
      self.confirm = self.easing_confirm = self.bias = self.stop_bias = 0.
      self.approached_from_distance = False
      self.brake_age = 10.
      return 0.
    self.confirm += dt
    if self.confirm >= .5 and speed >= 3. and lead.dRel > 6.:
      self.approached_from_distance = True

    # Settle toward a comfort target over the approach, without commanding a
    # stop at a fixed distance. Keep a small margin above 2 m in slow queues.
    if self.confirm >= .5 and .1 < speed <= 3. and -.1 <= lead.vLead <= .3 and lead.dRel <= 10.:
      stop_target = 2.5 if self.approached_from_distance else 2.1
      target_bias = max(0., stop_distance-stop_target)
      self.stop_bias = min(target_bias, self.stop_bias+1.0*dt)
      self.bias = self.easing_confirm = 0.
      return -self.stop_bias
    self.stop_bias = 0.
    self.brake_age = 0. if lead.aLeadK <= -.9 else self.brake_age+dt
    easing = (self.confirm >= 2. and self.brake_age <= 4. and 1.2 < speed < 30./3.6 and
              .5 < lead.vLead <= speed+.5 and -.9 < lead.aLeadK < .3 and
              -1.8 < lead.vRel < .5 and accel < -.1 and
              lead.dRel >= max(desired_distance+.5, stop_distance+speed*.9+1.))
    matched_low_speed = (self.confirm >= .5 and self.brake_age <= 4. and 1.2 < speed <= 3. and
                         lead.vLead > 1. and -.25 <= lead.vRel <= .35 and -1.2 < lead.aLeadK < .3 and
                         accel < -.1 and lead.dRel >= max(desired_distance-.35, 2.5+speed*.65))
    close_queue_easing = (self.confirm >= 2. and self.brake_age <= 4. and 1.2 < speed <= 4. and
                          lead.vLead > 1.5 and -.65 <= lead.vRel <= 1.5 and
                          -.8 < lead.aLeadK < .3 and accel < -.1 and
                          lead.dRel >= max(4.5, 2.0 + speed*.7, desired_distance-2.0))
    self.easing_confirm = self.easing_confirm+dt if easing or matched_low_speed or close_queue_easing else 0.
    target = min(.8, .15*speed) if self.easing_confirm >= .3 else 0.
    # Strong/renewed braking, closing or short room returns to the original goal.
    if not (matched_low_speed or close_queue_easing) and (lead.aLeadK <= -.9 or lead.vRel <= -1.8 or lead.dRel < desired_distance):
      self.bias = 0.
    else:
      rate = (.8 if matched_low_speed or close_queue_easing else .35) if target > self.bias else .8
      self.bias += max(-rate*dt, min(rate*dt, target-self.bias))
    if not 1.2 < speed < 30./3.6 or lead.vLead <= .5:
      self.bias = 0.
    return self.bias


def stop_approach_reference(bias, speed_plan, lead_speed_plan, lead_obstacle, other_obstacle, cruise_obstacle, stop_obstacle):
  """Normalize a bounded meter bias for the solver's distance-cost residual.

  Moving-lead bias fades near standstill. Stopped-lead bias applies only when the
  lead bounds the solution. Other lead and stop obstacles retain priority.
  This does not edit any parameter or constraint used by the solver.
  """
  arrays = [np.asarray(v, dtype=float) for v in
            (speed_plan, lead_speed_plan, lead_obstacle, other_obstacle, cruise_obstacle, stop_obstacle)]
  speed, lead_speed, lead_x, other_x, cruise_x, stop_x = arrays
  if not math.isfinite(bias) or bias == 0. or not all(np.all(np.isfinite(a)) for a in arrays):
    return np.zeros_like(speed)
  moving = np.clip((np.minimum(speed, lead_speed)-.5)/1.5, 0., 1.)
  selected_x = np.minimum(lead_x, cruise_x)
  clear = (other_x >= selected_x+2.) & (stop_x >= selected_x+5.)
  if bias < 0.:
    # A negative input represents a stopped-lead comfort target.
    stopped = np.clip((.5-lead_speed)/.2, 0., 1.)
    return bias*stopped*clear*(lead_x <= cruise_x)/(np.maximum(0., speed)+10.)
  return -min(.8, bias)*moving*clear/(np.maximum(0., speed)+10.)


class TerminalStopRelease:
  """Ease negative feedforward only in the final crawl behind a confirmed lead."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.identity = self.stamp = None
    self.distance = self.relative = self.confirm = 0.

  @staticmethod
  def same_target(lead, other):
    # Radar and vision may publish the same close car twice. A different radar
    # identity is always independent; a vision duplicate must overlap tightly.
    if StopApproach.same_lead(lead, other):
      return True
    values = (other.dRel, other.vRel, other.dPath)
    return (other.present and not other.radar and other.radarTrackId < 0 and
            all(math.isfinite(x) for x in values) and
            abs(other.dRel-lead.dRel) < .75 and
            abs(other.vRel-lead.vRel) < .35 and abs(other.dPath-lead.dPath) < .2)

  def update(self, lead, other, speed, command, previous, stamp, now, enabled, dt):
    values = (speed, command, previous, stamp, now, dt, lead.dRel, lead.vRel,
              lead.vLead, lead.aLeadK, lead.modelProb, lead.dPath)
    if (not enabled or not all(math.isfinite(x) for x in values) or
        not 0. < dt <= .05 or not 0. <= now-stamp <= .2 or
        not lead.present or not lead.radar or lead.radarTrackId < 0 or
        lead.modelProb < .95 or abs(lead.dPath) > .7 or not 0. <= speed < 15. or
        not 2. < lead.dRel < 60. or abs(lead.vLead-speed-lead.vRel) > 1. or
        (other.present and not self.same_target(lead, other) and
         (not math.isfinite(other.dRel) or other.dRel < lead.dRel+5.))):
      self.reset()
      return command
    if self.stamp != stamp:
      elapsed = stamp-self.stamp if self.stamp is not None else 0.
      continuous = (self.identity == lead.radarTrackId and .01 <= elapsed <= .2 and
                    abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative)*elapsed) < .5)
      self.confirm = self.confirm+elapsed if continuous else 0.
      self.identity, self.stamp = lead.radarTrackId, stamp
      self.distance, self.relative = lead.dRel, lead.vRel
    # A stronger inherited brake request can also outlast a stopped lead's
    # deceleration. Relax it only in the final crawl after longer confirmation;
    # the same stopping-room calculation below still has to pass.
    gentle = -2. < command < -.5
    settled_strong = (-2.8 < command <= -2. and speed <= .8 and self.confirm >= 1. and
                      abs(lead.vLead) < .3 and lead.aLeadK > -.8)
    if (self.confirm < .5 or not 0. <= speed <= 1.5 or abs(lead.vLead) >= .5 or
        not 2.5 <= lead.dRel <= 4.5 or not (gentle or settled_strong)):
      return command
    closing = max(speed, -lead.vRel, 0.)
    # Include actuator travel before evaluating room above a 2 m reserve.
    room = lead.dRel-2.-.3*closing
    required = closing**2/(2.*max(.01, room))
    if room <= 0. or required > .9:
      return command
    floor = -max(.5, 1.3*required)
    return max(command, min(floor, previous+1.5*dt))
