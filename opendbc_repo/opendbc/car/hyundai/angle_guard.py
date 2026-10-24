"""Conservative Ioniq 5 PE angle-command guard; thresholds are not EPS specs."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GuardOutput:
  angle: float
  active: bool
  torque_scale: float
  state: int  # 0 normal, 1 limiting/takeover warning, 2 temporarily unavailable
  reason: str


class AngleSteeringGuard:
  # Margin below the repeated ~198 degree command fault on the recorded vehicle.
  WARN_ANGLE = 170.
  LIMITED_ANGLE = 180.
  HARD_ANGLE = 190.
  TRACKING_ERROR = 50.
  LIMITED_ERROR = 45.
  MAX_RISK_TIME = .4
  RELEASE_ANGLE = 150.
  RELEASE_TIME = .5
  RESUME_TIME = .5

  def __init__(self):
    self.last_time = None
    self.last_output = None
    self.risk_time = 0.
    self.safe_time = 0.
    self.resume_time = 0.
    self.held = False
    self.was_active = False
    self.reason = ''

  def update(self, now_ns, requested, candidate, actual, lat_active, eps_fault):
    now = float(now_ns) / 1e9
    dt = .01 if self.last_time is None else max(0., min(.05, now-self.last_time))
    if self.last_time is not None and (now < self.last_time or now-self.last_time > .2):
      self.resume_time = 0.
      self.was_active = False
      self.risk_time = self.safe_time = 0.
    self.last_time = now
    valid = all(math.isfinite(v) for v in (requested, candidate, actual, now))
    if not valid:
      self.held, self.reason = True, 'invalid_input'
    elif eps_fault:
      self.held, self.reason = True, 'eps_fault'
    elif lat_active and (abs(candidate) >= self.HARD_ANGLE or abs(actual) >= self.HARD_ANGLE):
      self.held, self.reason = True, 'command_envelope'

    if self.held:
      safe = (valid and not eps_fault and abs(actual) < self.RELEASE_ANGLE
              and abs(requested) < self.RELEASE_ANGLE and abs(candidate-actual) < 20.)
      self.safe_time = self.safe_time + dt if safe else 0.
      if self.safe_time < self.RELEASE_TIME:
        self.was_active = False
        self.resume_time = self.risk_time = 0.
        self.last_output = actual if math.isfinite(actual) else 0.
        return GuardOutput(self.last_output, False, 0., 2, self.reason)
      self.held = False
      self.safe_time = 0.
      self.reason = ''

    if not lat_active:
      self.was_active = False
      self.resume_time = self.risk_time = 0.
      self.last_output = actual
      return GuardOutput(actual, False, 0., 0, '')

    error = candidate-actual
    warning = abs(candidate) >= self.WARN_ANGLE
    # A 20-degree lag near 170 degrees can be ordinary actuator tracking delay.
    # Reducing torque there increases the lag and can falsely latch a handover
    # before the wheel reaches the requested angle (test35, route 53 at 40:31).
    # Keep the warning separate from severe tracking-error protection; the
    # absolute command/measured-angle envelope above still applies.
    risk = abs(candidate) >= 100. and abs(error) >= self.TRACKING_ERROR
    self.risk_time = self.risk_time + dt if risk else 0.
    if self.risk_time >= self.MAX_RISK_TIME:
      self.held, self.reason = True, 'persistent_angle_or_tracking_error'
      self.was_active = False
      self.resume_time = 0.
      self.last_output = actual
      return GuardOutput(actual, False, 0., 2, self.reason)

    angle = candidate
    if risk:
      target = max(-self.LIMITED_ANGLE, min(self.LIMITED_ANGLE, candidate))
      target = max(actual-self.LIMITED_ERROR, min(actual+self.LIMITED_ERROR, target))
      previous = actual if self.last_output is None else self.last_output
      step = 100.*dt
      angle = max(previous-step, min(previous+step, target))
    if not self.was_active:
      angle = actual  # First reactivation follows the wheel; never a stale target.
    self.resume_time = min(self.RESUME_TIME, self.resume_time + dt)
    scale = self.resume_time / self.RESUME_TIME
    if risk:
      scale *= max(0., 1. - self.risk_time/self.MAX_RISK_TIME)
    self.last_output, self.was_active = angle, True
    return GuardOutput(angle, True, scale, 1 if warning or risk else 0,
                       'approaching_limit' if warning else 'tracking_error' if risk else '')
