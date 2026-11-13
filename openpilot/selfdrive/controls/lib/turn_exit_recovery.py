"""Brief cruise recovery after a confirmed tight turn, before normal safety caps."""
import math


def _interp(x, xp, fp):
  for i in range(1, len(xp)):
    if x <= xp[i]:
      t = max(0., (x-xp[i-1])/(xp[i]-xp[i-1]))
      return fp[i-1]+t*(fp[i]-fp[i-1])
  return fp[-1]


class TurnExitRecovery:
  def __init__(self):
    self.last_time = None
    self.reset()

  def reset(self):
    self.turn_confirm = self.exit_confirm = 0.
    self.peak_angle = self.peak_curve = 0.
    self.last_turn_time = None
    self.until = None
    self.strength = 0.
    self.armed = False

  @staticmethod
  def clear_leads(leads, v_ego):
    # Both selected leads, including vision-only tracks, can veto recovery.
    for lead in leads:
      if not lead.present:
        continue
      if (not all(math.isfinite(v) for v in (lead.dRel, lead.vRel, lead.aLeadK)) or
          lead.dRel < max(25., 2.5*v_ego+8.) or lead.vRel < -.5 or lead.aLeadK < -.5):
        return False
    return True

  def update(self, *, now, enabled, v_ego, steering_angle, curvature, preview, far_preview,
             normal_max, target_speed, leads):
    previous_time, self.last_time = self.last_time, now
    dt = 0. if previous_time is None else now-previous_time
    values = (now, v_ego, steering_angle, curvature, preview, far_preview, normal_max, target_speed)
    if (not enabled or not all(math.isfinite(v) for v in values) or not 0. <= dt <= .2 or
        not 1. <= v_ego <= 60./3.6 or normal_max <= 0.):
      self.reset()
      return normal_max

    angle = abs(steering_angle)
    turning = (v_ego <= 12. and angle >= 60. and abs(preview)*v_ego**2 >= .4)
    if self.until is None:
      if turning:
        self.turn_confirm += dt
        self.last_turn_time = now
        self.peak_angle = max(self.peak_angle, angle)
        self.peak_curve = max(self.peak_curve, abs(preview))
        self.armed = self.armed or self.turn_confirm >= .3
      elif not self.armed:
        self.turn_confirm = 0.
        self.peak_angle = self.peak_curve = 0.
      if self.last_turn_time is not None and now-self.last_turn_time > 6.:
        self.reset()
        return normal_max
      opening = (self.armed and angle <= min(30., .4*self.peak_angle) and
                 abs(curvature)*v_ego**2 <= .8 and abs(preview)*v_ego**2 <= .8 and
                 abs(far_preview)*v_ego**2 <= 1.2 and abs(preview) <= .4*self.peak_curve and
                 target_speed-v_ego >= .8 and self.clear_leads(leads, v_ego))
      self.exit_confirm = self.exit_confirm+dt if opening else 0.
      if self.exit_confirm >= .3:
        self.until = now+6.

    if self.until is None:
      return normal_max
    # Re-entering a bend, approaching a lead or reaching the target removes assistance.
    if (now >= self.until or not self.clear_leads(leads, v_ego) or angle > 40. or target_speed-v_ego < .8):
      self.reset()
      return normal_max
    if (abs(curvature)*v_ego**2 > .8 or abs(preview)*v_ego**2 > .8 or abs(far_preview)*v_ego**2 > 1.2):
      # Pause on an uncertain preview; the original window keeps counting down.
      self.strength = self.exit_confirm = 0.
      return normal_max
    self.exit_confirm = min(.3, self.exit_confirm+dt)
    if self.exit_confirm < .3:
      return normal_max
    self.strength = min(1., self.strength+dt/.6, max(0., self.until-now))
    desired = _interp(v_ego*3.6, (10., 20., 40., 60.), (1.6, 1.5, 1.2, .9))
    return normal_max+self.strength*min(.45, max(0., desired-normal_max))
