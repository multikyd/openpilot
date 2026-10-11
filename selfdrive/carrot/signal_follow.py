"""Bounded following assistance; a signal never becomes a stop/start command."""
import math


def directional_signal(data, now, navigation, gps):
  movements = {}
  for direction in ('straight', 'left', 'right', 'uturn'):
    state = str(data.get(direction, '')).upper()
    try:
      remain = float(data.get(direction + '_remain_time', 0))
    except (TypeError, ValueError):
      remain = 0.0
    movements[direction] = {'state': state, 'remain': remain}
  return {'ts': now, 'lat': data.get('lat'), 'lon': data.get('lon'),
          'distance': data.get('distance'), 'movements': movements,
          'navigation': navigation, 'gps': gps}


def _position(lat, lon):
  return math.isfinite(lat) and math.isfinite(lon) and -90 < lat < 90 and -180 < lon < 180 and abs(lat) > 1 and abs(lon) > 1


def _offset(lat, lon, target_lat, target_lon):
  north = math.radians(target_lat - lat) * 6371000.0
  east = math.radians(target_lon - lon) * 6371000.0 * math.cos(math.radians(lat))
  return north, east


def red_approach(signal, now, v_ego):
  """Only fresh straight-through route signals corroborated by vehicle GPS."""
  try:
    age = now - float(signal['ts'])
    if not 0 <= age <= 2.0:
      return False, 'signal_age'
    nav, gps = signal['navigation'], signal['gps']
    if not gps['valid'] or not 0 <= float(gps['age']) + age <= 2.0 or not 0 <= float(nav['age']) + age <= 2.0:
      return False, 'position_age'
    accuracy = float(gps['accuracy'])
    if not math.isfinite(accuracy) or not 0 < accuracy <= 15.0:
      return False, 'gps_accuracy'
    lat, lon, slat, slon = map(float, (nav['lat'], nav['lon'], signal['lat'], signal['lon']))
    glat, glon = float(gps['lat']), float(gps['lon'])
    if not _position(lat, lon) or not _position(slat, slon) or not _position(glat, glon):
      return False, 'position'
    if math.hypot(*_offset(lat, lon, glat, glon)) > 30.0:
      return False, 'simulation_or_position_mismatch'
    nav_speed, gps_speed = float(nav['speed']) / 3.6, float(gps['speed'])
    if not math.isfinite(nav_speed) or not math.isfinite(gps_speed) or abs(nav_speed - v_ego) > 3.0 or abs(gps_speed - v_ego) > 3.0:
      return False, 'speed_mismatch'
    heading = float(gps['bearing'])
    nav_heading = float(nav['bearing'])
    if not math.isfinite(heading) or not math.isfinite(nav_heading) or abs((heading - nav_heading + 180) % 360 - 180) > 20:
      return False, 'heading'
    north, east = _offset(glat, glon, slat, slon)
    distance = math.hypot(north, east)
    angle = math.radians(heading)
    ahead = north * math.cos(angle) + east * math.sin(angle)
    lateral = abs(east * math.cos(angle) - north * math.sin(angle))
    # Leave close-to-stop behavior to normal lead following.
    if not 20 < distance < min(250.0, max(50.0, v_ego * 12.0)) or ahead <= 0 or lateral > 15:
      return False, 'signal_geometry'
    reported = float(signal['distance'])
    if not math.isfinite(reported) or reported <= 0 or abs(reported - distance) > max(25.0, distance * .25):
      return False, 'distance_mismatch'
    turn, turn_distance = int(nav['turn']), float(nav['turn_distance'])
    # Version 1 supports straight approaches only. Unknown and near turns are excluded.
    if turn not in (0, 11, 202) and not (turn in (12, 16, 13, 19, 14) and turn_distance > distance + 40):
      return False, 'route_direction'
    straight = signal['movements']['straight']
    remaining = float(straight['remain']) - age
    if straight['state'] != 'RED_LIGHT_ON' or not 2.0 < remaining <= 300:
      return False, 'straight_not_red'
    return True, 'red_ahead'
  except (KeyError, TypeError, ValueError, OverflowError):
    return False, 'invalid_data'


class SignalFollowAssist:
  def __init__(self, enabled=True):
    self.enabled = enabled
    self.strength = 0.0
    self.gap_extra = 0.0
    self.confirm_time = 0.0
    self.last_distance = None
    self.reason = 'inactive'

  def update(self, signal, now, dt, v_ego, lead, active, override=False):
    if not self.enabled or not active or override or not 2.0 < v_ego < 22.2:
      self.strength = self.gap_extra = self.confirm_time = 0.0
      self.last_distance = None
      self.reason = 'inactive'
      return
    valid, self.reason = red_approach(signal, now, v_ego)
    try:
      lead_valid = lead.status and (lead.radar or lead.modelProb > .8) and all(
        math.isfinite(float(x)) for x in (lead.dRel, lead.vRel, lead.aLeadK, lead.vLead)) and 0 < lead.dRel < max(20.0, v_ego * 3.0)
      if lead_valid and self.last_distance is not None and abs(lead.dRel - self.last_distance) > max(3.0, v_ego * .3):
        lead_valid = False
      self.last_distance = float(lead.dRel) if lead.status else None
    except (AttributeError, TypeError, ValueError):
      lead_valid = False
      self.last_distance = None
    self.confirm_time = min(1.0, self.confirm_time + dt) if valid and lead_valid else 0.0
    if valid and not lead_valid:
      self.reason = 'lead_unconfirmed'
    target = 1.0 if self.confirm_time >= .5 else 0.0
    self.strength += max(-.4 * dt, min(.5 * dt, target - self.strength))
    slowing = lead_valid and (lead.aLeadK < -.4 or lead.vRel < -.5 or lead.vLead < 2.0)
    # Extra distance is v_ego * gap_extra, vanishing near standstill.
    fade = max(0.0, min(1.0, (v_ego - 2.0) / 2.0))
    gap_target = .25 * self.strength * fade if valid and slowing else 0.0
    self.gap_extra += max(-.1 * dt, min(.05 * dt, gap_target - self.gap_extra))
    self.reason = 'active' if target else self.reason

  def cruise_accel(self, normal_max):
    return min(normal_max, normal_max * (1.0 - self.strength) + min(normal_max, .3) * self.strength)
