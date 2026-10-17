import math


class CongestionFollow:
  """Recognize repeated short stop/go waves; never change standstill clearance."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.time = 0.
    self.stops = []
    self.low_time = self.move_time = self.flow_time = self.missing_time = 0.
    self.ready = True
    self.lead_id = None
    self.lead_time = 0.
    self.strength = 0.

  def update(self, lead, speed, normal_accel, t_follow, enabled, dt):
    if not enabled or not all(math.isfinite(x) for x in (speed, normal_accel, t_follow, dt)) or not 0. < dt <= .25:
      self.reset()
      return t_follow, normal_accel
    self.time += dt
    self.stops = [t for t in self.stops if self.time-t < 90.]
    reliable = (lead.present and lead.radar and lead.radarTrackId >= 0 and
                all(math.isfinite(x) for x in (lead.modelProb, lead.dRel, lead.dPath, lead.vLead)) and
                lead.modelProb >= .8 and 2. < lead.dRel < 45. and abs(lead.dPath) < 1.8 and lead.vLead >= 0.)
    if reliable:
      self.lead_time = self.lead_time+dt if self.lead_id == lead.radarTrackId else 0.
      self.lead_id = lead.radarTrackId
      self.missing_time = 0.
    else:
      self.lead_time = 0.
      self.lead_id = None
      self.missing_time += dt

    trusted = reliable and self.lead_time >= .8
    self.low_time = self.low_time+dt if trusted and speed < 1.2 and lead.vLead < 1.2 else 0.
    self.move_time = self.move_time+dt if trusted and speed > 1.5 and lead.vLead > 1.5 else 0.
    if self.move_time >= 2.:
      self.ready = True
    if self.ready and self.low_time >= 1.:
      self.stops.append(self.time)
      self.ready = False
    self.flow_time = self.flow_time+dt if speed > 8. or (trusted and lead.vLead > 9.) else 0.
    if self.flow_time >= 8. or self.missing_time >= 3.:
      self.stops.clear()
    target = float(len(self.stops) >= 2 and trusted and speed < 10. and lead.vLead < 10.)
    # Two-second entry, four-second release. Missing/cut-in leads receive no cap.
    self.strength += max(-dt/4., min(dt/2., target-self.strength))
    weight = self.strength * max(0., min(1., (10.-speed)/2.)) if trusted else 0.
    extra_follow = .25 * weight * max(0., min(1., speed/2.))
    accel_cap = .75 + .35 * max(0., min(1., speed/8.))
    return t_follow+extra_follow, min(normal_accel, normal_accel+weight*(accel_cap-normal_accel))
