"""Presentation state for the C4 mici lead/brake side panel; no control output."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LeadBrakeState:
  state: str
  distance: float
  relative_speed: float
  motion: int  # -1 closing, +1 opening, 0 nearly constant separation

  @property
  def status_text(self):
    return {"ON": "ON?", "OFF": "OFF?", "UNKNOWN": "WAIT"}[self.state]

  @property
  def motion_text(self):
    return {-1: "CLOSING", 1: "AWAY", 0: "STEADY"}[self.motion]


def lead_brake_state(lead, fresh, observation=None):
  if not fresh or not lead.present:
    return None
  distance, relative = float(lead.dRel), float(lead.vRel)
  if not math.isfinite(distance) or not math.isfinite(relative) or distance < 0:
    return None
  state = observation.state if observation is not None else "UNKNOWN"
  if state not in ("ON", "OFF", "UNKNOWN"):
    state = "UNKNOWN"
  motion = -1 if relative < -.3 else 1 if relative > .3 else 0
  return LeadBrakeState(state, distance, relative, motion)
