"""Explainable traffic evidence for overtake candidates, not a learned model.

Missing detections are unknown space. This module never commands actuators.
All speeds are m/s; distances are sensor-reported longitudinal metres.
"""
from dataclasses import dataclass
import math
from statistics import median


@dataclass(frozen=True)
class TrafficAssessment:
  eligible: bool
  reason: str
  ahead_gap: float = 0.
  left_faster: bool = False
  right_faster: bool = False


def assess_traffic(*, fresh, limit_valid, road_limit, set_speed, lead, center, left, right):
  if (not fresh or not limit_valid or not math.isfinite(road_limit) or
      not 70./3.6 <= road_limit <= 140./3.6):
    return TrafficAssessment(False, "road_limit_unavailable")
  if (not math.isfinite(set_speed) or not lead.present or not lead.radar or lead.radarTrackId < 0 or
      not all(math.isfinite(v) for v in (lead.dRel, lead.vLead)) or lead.vLead < 0.):
    return TrafficAssessment(False, "lead_unreliable")
  # A small margin avoids classifying measurement noise at the limit as slow.
  if lead.vLead >= road_limit-3./3.6:
    return TrafficAssessment(False, "respect_road_limit")
  if lead.vLead > road_limit-5./3.6 or lead.vLead > set_speed-3./3.6:
    return TrafficAssessment(False, "speed_difference_small")

  def valid(vehicle):
    return (vehicle.present and vehicle.radar and vehicle.radarTrackId >= 0 and
            all(math.isfinite(v) for v in (vehicle.dRel, vehicle.vLead, vehicle.dPath)) and
            vehicle.vLead >= 0.)

  # Only a distinct, tightly aligned object beyond the original lead can
  # supply ahead-of-lead evidence. leadTwo alone may instead be a cut-in.
  ahead = [v for v in center if valid(v) and v.radarTrackId != lead.radarTrackId and
           abs(v.dPath) <= .8 and v.dRel > lead.dRel+5.]
  next_car = min(ahead, key=lambda v: v.dRel, default=None)
  gap = next_car.dRel-lead.dRel if next_car is not None else 0.
  def faster(vehicles):
    speeds = [v.vLead for v in vehicles if valid(v) and v.radarTrackId != lead.radarTrackId and
              abs(v.dRel) <= 100. and 1.8 <= abs(v.dPath) <= 5.]
    return bool(speeds) and median(speeds) >= lead.vLead+8./3.6
  left_fast, right_fast = faster(left), faster(right)
  if next_car is not None and gap <= max(25., 2.*lead.vLead) and next_car.vLead <= lead.vLead+5./3.6:
    return TrafficAssessment(False, "following_traffic", gap, left_fast, right_fast)
  if next_car is not None and gap >= max(45., 3.*lead.vLead):
    return TrafficAssessment(True, "observed_large_ahead_gap", gap, left_fast, right_fast)
  if left_fast and right_fast:
    return TrafficAssessment(True, "both_adjacent_lanes_faster", gap, left_fast, right_fast)
  return TrafficAssessment(False, "insufficient_traffic_evidence", gap, left_fast, right_fast)


class OvertakeAdviceConfirmation:
  """Confirm sustained evidence for the same lead; invalidate immediately on loss."""
  def __init__(self):
    self.previous = None
    self.lead_id = -1
    self.duration = 0.

  def update(self, now, eligible, lead_id):
    if not math.isfinite(now):
      self.__init__()
      return False
    dt = 0. if self.previous is None else now-self.previous
    self.previous = now
    if not eligible or lead_id < 0 or not 0. <= dt <= .2:
      self.lead_id, self.duration = -1, 0.
      return False
    if self.lead_id != lead_id:
      self.lead_id, self.duration = lead_id, 0.
    else:
      self.duration += dt
    return self.duration >= 3.
