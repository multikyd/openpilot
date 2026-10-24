"""Shared geometry for the 536x240 C4 mici HUD, in logical UI pixels."""
from dataclasses import dataclass
import math

HEADER_TOP = 4
LEFT_CENTER = 46  # driver x=16 + width=60 / 2; also TPMS, gap and warning center
SPEED_HEADER_HEIGHT = 70


def left_car_box(x, y):
  return (x+LEFT_CENTER-20, y+77, 40, 70)


def lower_notice_box(x, y, height):
  return (x+LEFT_CENTER-41, y+150, 82, min(90, max(0, height-154)))


def gap_bar_boxes(x, y):
  # Four fixed slots near the hood, inside the 40x70 car, never above its roof.
  return tuple((x+LEFT_CENTER-7, y+100+i*4, 14, 2) for i in range(4))


def signal_box(x, y, width):
  # Offset box ends at video center + 40. Keep an 8px gap and a rounded-corner inset.
  if width < 440:
    return None
  available = width/2-40-8-20
  w = min(190., available)
  return (x+width-20-w, y+HEADER_TOP, w, 74*w/232)


@dataclass(frozen=True)
class RoadNotice:
  kind: str
  speed: float
  distance: float


def select_road_notice(kind, original_limit, target_limit, distance, bump_target, bump_distance):
  def valid(speed, dist):
    return math.isfinite(speed) and math.isfinite(dist) and speed > 0 and dist >= 0
  candidates = []
  if kind == 22:
    if valid(target_limit, distance) and distance > 0:
      candidates.append(RoadNotice('BUMP', target_limit, distance))
  elif valid(original_limit, distance) and original_limit > 20 and distance > 0:
    candidates.append(RoadNotice('CAM', original_limit, distance))
  if valid(bump_target, bump_distance):
    candidates.append(RoadNotice('BUMP', bump_target, bump_distance))
  return min(candidates, key=lambda n: (n.distance, n.kind != 'BUMP')) if candidates else None
