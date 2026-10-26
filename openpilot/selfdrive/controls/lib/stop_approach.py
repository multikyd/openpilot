"""Small, temporary comfort-distance bias while a stable lead's braking eases.

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
    self.bias = 0.

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
        abs(lead.dPath) > 1.2 or not 2. < lead.dRel < 100. or lead.vLead < 0. or
        not 2. <= stop_distance <= 10. or abs(lead.vLead-speed-lead.vRel) > 1.5 or
        (other.present and not self.same_lead(lead, other) and (not math.isfinite(other.dRel) or other.dRel < lead.dRel+5.))):
      self.reset()
      return 0.
    if self.stamp == stamp:
      return self.bias
    dt = stamp-self.stamp if self.stamp is not None else 0.
    continuous = (self.identity == lead.radarTrackId and .01 <= dt <= .2 and
                  abs(lead.dRel-self.distance-.5*(lead.vRel+self.relative_speed)*dt) <= 1.5)
    self.identity, self.stamp = lead.radarTrackId, stamp
    self.distance, self.relative_speed = lead.dRel, lead.vRel
    if not continuous:
      self.confirm = self.easing_confirm = self.bias = 0.
      self.brake_age = 10.
      return 0.
    self.confirm += dt
    self.brake_age = 0. if lead.aLeadK <= -.9 else self.brake_age+dt
    easing = (self.confirm >= 2. and self.brake_age <= 4. and 1.2 < speed < 30./3.6 and
              .5 < lead.vLead <= speed+.5 and -.9 < lead.aLeadK < .3 and
              -1.8 < lead.vRel < .5 and accel < -.1 and
              lead.dRel >= max(desired_distance+.5, stop_distance+speed*.9+1.))
    self.easing_confirm = self.easing_confirm+dt if easing else 0.
    target = min(.8, .15*speed) if self.easing_confirm >= .3 else 0.
    # Strong/renewed braking, closing or short room returns to the original goal.
    if lead.aLeadK <= -.9 or lead.vRel <= -1.8 or lead.dRel < desired_distance:
      self.bias = 0.
    else:
      rate = .35 if target > self.bias else .8
      self.bias += max(-rate*dt, min(rate*dt, target-self.bias))
    if not 1.2 < speed < 30./3.6 or lead.vLead <= .5:
      self.bias = 0.
    return self.bias


def stop_approach_reference(bias, speed_plan, lead_speed_plan, lead_obstacle, other_obstacle, cruise_obstacle, stop_obstacle):
  """Normalize a bounded meter bias for the solver's distance-cost residual.

  Fade on predicted near-standstill states. Other lead/stop obstacles have priority.
  This does not edit any parameter or constraint used by the solver.
  """
  arrays = [np.asarray(v, dtype=float) for v in
            (speed_plan, lead_speed_plan, lead_obstacle, other_obstacle, cruise_obstacle, stop_obstacle)]
  speed, lead_speed, lead_x, other_x, cruise_x, stop_x = arrays
  if not math.isfinite(bias) or bias <= 0. or not all(np.all(np.isfinite(a)) for a in arrays):
    return np.zeros_like(speed)
  moving = np.clip((np.minimum(speed, lead_speed)-.5)/1.5, 0., 1.)
  selected_x = np.minimum(lead_x, cruise_x)
  clear = (other_x >= selected_x+2.) & (stop_x >= selected_x+5.)
  return -min(.8, bias)*moving*clear/(np.maximum(0., speed)+10.)
