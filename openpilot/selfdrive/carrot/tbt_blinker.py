"""Short-lived SPAS2 requests for the opt-in TBT driving experiment."""
import math
from uuid import uuid4

from opendbc.car.hyundai.blinker_test import CANCEL, TBT_REQUEST, TBT_MAX_SECONDS, TBT_WAIT_MAX_SECONDS, moving_speed_ok, read_json, write_json

TURN_START_DISTANCE = 70.0
TURN_MAX_START_DISTANCE = 90.0
TURN_FALLBACK_START_DISTANCE = TURN_START_DISTANCE
TURN_APPROACH_SECONDS = 4.0
TURN_FINISH_SECONDS = 5.0


class TbtBlinker:
  def __init__(self, request=TBT_REQUEST, cancel=CANCEL):
    self.request, self.cancel = request, cancel
    self.turn_direction = None
    self.turn_id = None
    self.previous_distance = None
    self.request_id = None
    self.last_request_id = None
    self.start = 0.
    self.request_distance_limit = 0.
    self.stationary_wait = False
    self.steered_into_turn = False
    self.finish_until = None
    self.centered_since = None
    self.last_center_sample = None
    self.cancelled = False

  def stop(self, now):
    if self.request_id is not None:
      write_json(self.request, {"id": self.request_id, "action": "off", "start": now,
                                "expires": now, "deadline": now})
      self.request_id = None
      self.stationary_wait = False
      self.steered_into_turn = False
      self.finish_until = None
      self.centered_since = None
      self.last_center_sample = None

  def update(self, now, code, distance, age, cs, enabled, *, held=False, next_road_width=0, lane_width=0.):
    valid_turn = (code in (12, 13, 16, 19) and math.isfinite(distance) and 0 <= age <= 2.)
    direction = "left" if code in (12, 16) else "right"
    if valid_turn:
      next_turn = (direction != self.turn_direction or
                   (self.previous_distance is not None and
                    (distance - self.previous_distance >= 45 or
                     (self.previous_distance <= 15 and distance >= 45))))
      # Navigation can advance at the turn point while the car is still turning.
      passed_turn_point = distance < -5 or (next_turn and distance >= 45.)
      if (self.finish_until is None and self.request_id is not None and self.steered_into_turn and
          self.previous_distance is not None and -5. <= self.previous_distance <= 12. and passed_turn_point):
        self.finish_until = now + TURN_FINISH_SECONDS
      if self.finish_until is not None and now < self.finish_until:
        direction, distance = self.turn_direction, 0.
      else:
        if self.finish_until is not None:
          self.stop(now)
          self.cancelled = True
        if next_turn:
          self.stop(now)
          self.turn_direction = direction
          self.turn_id = uuid4().hex
          self.last_request_id = None
          self.cancelled = False
        self.previous_distance = distance
        if distance < -5:
          self.cancelled = True

    if self.last_request_id is not None and read_json(self.cancel).get("id") == self.last_request_id:
      self.cancelled = True

    if (not enabled or not valid_turn or cs is None or self.cancelled or
        not (cs.canValid and cs.blinkerStalkSupported and cs.blinkerStalkValid) or
        str(cs.gearShifter) != "drive"):
      self.stop(now)
      return

    moving_threshold = TURN_FALLBACK_START_DISTANCE
    if math.isfinite(lane_width) and 2. <= lane_width <= 5.:
      width_fraction = min(1., max(0., (lane_width - 2.8) / (3.5 - 2.8)))
      moving_threshold = TURN_START_DISTANCE + width_fraction * (TURN_MAX_START_DISTANCE - TURN_START_DISTANCE)
    if cs.leftBlinkerStalk or cs.rightBlinkerStalk or (cs.leftBlinker and cs.rightBlinker):
      if self.request_id is not None or 0 <= distance <= moving_threshold:
        self.cancelled = True
      self.stop(now)
      return

    speed = cs.vEgo
    waiting = cs.standstill and abs(speed) < .05 and held and not cs.gasPressed
    moving = moving_speed_ok(cs)
    if not math.isfinite(speed) or not (waiting or moving):
      self.stop(now)
      return
    self.stationary_wait = self.stationary_wait or waiting

    stopped_threshold = 20. + min(10., max(0., 2. * (next_road_width - 5.)))
    # Reserve time for the turn inside the moving request deadline.
    threshold = stopped_threshold if waiting else min(moving_threshold, abs(speed) * TURN_APPROACH_SECONDS)
    if self.request_id is None:
      if not 0 <= distance <= threshold or cs.leftBlinker or cs.rightBlinker:
        return
      self.request_id = self.last_request_id = uuid4().hex
      self.start = now
      # Keep the initial window when lane lines fade near the junction.
      self.request_distance_limit = moving_threshold + 3.
    elif distance > self.request_distance_limit:
      self.stop(now)
      return

    angle = cs.steeringAngleDeg
    if self.finish_until is not None and not math.isfinite(angle):
      self.cancelled = True
      self.stop(now)
      return
    # Only recognize a turn near the navigation point and while moving.
    if moving and math.isfinite(angle):
      direction_angle = angle if direction == "left" else -angle
      if distance <= 12. and direction_angle >= 30.:
        self.steered_into_turn = True
      if self.steered_into_turn and distance <= 8. and abs(angle) <= 8.:
        if self.last_center_sample is None or not 0 <= now - self.last_center_sample <= .25:
          self.centered_since = now
        self.last_center_sample = now
        if now - self.centered_since >= .4:
          self.cancelled = True
          self.stop(now)
          return
      else:
        self.last_center_sample = None
    else:
      self.last_center_sample = None

    deadline = self.start + (TBT_WAIT_MAX_SECONDS if self.stationary_wait else TBT_MAX_SECONDS)
    if now >= deadline:
      self.cancelled = True
      self.stop(now)
      return
    write_json(self.request, {"id": self.request_id, "voice_id": self.turn_id,
                              "action": direction, "stationary": waiting,
                              "start": self.start, "expires": min(now + .35, deadline),
                              "deadline": deadline})
