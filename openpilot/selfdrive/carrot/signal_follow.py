"""Bounded following assistance; a signal never becomes a stop/start command."""
import math


def normalized_position(lat, lon):
  try:
    lat, lon = float(lat), float(lon)
    valid = lambda a, b: math.isfinite(a) and math.isfinite(b) and -90 < a < 90 and -180 < b < 180
    if valid(lat, lon):
      return lat, lon
    if valid(lon, lat):
      return lon, lat
  except (TypeError, ValueError, OverflowError):
    pass
  return None, None


def signal_from_sinf(data):
  # Legacy sinf red is straight/general; it does not establish a left/right red.
  location = data.get('location', {})
  location = location if isinstance(location, dict) else {}
  lat, lon = normalized_position(location.get('latitude'), location.get('longitude'))
  out = dict(lat=lat, lon=lon, distance=data.get('distance'))
  for direction, key in (('straight', 'green'), ('left', 'left'), ('right', 'right'), ('uturn', 'uturn')):
    on = bool(data.get(key + 'LightOn'))
    state = 'GREEN_LIGHT_ON' if on else 'UNAVAILABLE'
    remain = data.get(key + 'LightRemainTime', 0)
    if direction == 'straight' and data.get('redLightOn') and not on:
      state, remain = 'RED_LIGHT_ON', data.get('redLightRemainTime', 0)
    out[direction], out[direction + '_remain_time'] = state, remain
  return out


def directional_signal(data, now, navigation, gps):
  movements = {}
  for direction in ('straight', 'left', 'right', 'uturn'):
    state = str(data.get(direction, '')).upper()
    try:
      remain = float(data.get(direction + '_remain_time', 0))
    except (TypeError, ValueError):
      remain = 0.0
    movements[direction] = {'state': state, 'remain': remain}
  lat, lon = normalized_position(data.get('lat'), data.get('lon'))
  return {'ts': now, 'lat': lat, 'lon': lon,
          'distance': data.get('distance'), 'movements': movements,
          'navigation': navigation, 'gps': gps}


def usable_signal(signal):
  """Reject corrupt/empty feeds without refreshing the last valid timestamp."""
  if not isinstance(signal, dict):
    return False
  try:
    distance = float(signal.get('distance'))
    lat, lon = normalized_position(signal.get('lat'), signal.get('lon'))
    return (math.isfinite(distance) and 0 <= distance <= 5000 and lat is not None and
            _position(lat, lon) and any(
              m.get('state') in ('RED_LIGHT_ON', 'GREEN_LIGHT_ON', 'YELLOW_LIGHT_ON')
              for m in signal.get('movements', {}).values()))
  except (ValueError, TypeError, OverflowError, AttributeError):
    return False


def _position(lat, lon):
  return math.isfinite(lat) and math.isfinite(lon) and -90 < lat < 90 and -180 < lon < 180 and abs(lat) > 1 and abs(lon) > 1


def _offset(lat, lon, target_lat, target_lon):
  north = math.radians(target_lat - lat) * 6371000.0
  east = math.radians(target_lon - lon) * 6371000.0 * math.cos(math.radians(lat))
  return north, east


def lane_guidance(data):
  """Keep bounded, explicit route movements with their own point and distance."""
  items = data.get('aheadLaneInfoData', [])
  items = list(items[:32]) if isinstance(items, list) else []
  items.append(data)
  result = []
  directions = {4: 'straight', 11: 'straight', 202: 'straight',
                12: 'left', 16: 'left', 1000: 'left',
                13: 'right', 19: 'right', 1001: 'right', 14: 'uturn'}
  for item in items:
    if not isinstance(item, dict):
      continue
    try:
      lat, lon = normalized_position(item.get('vpLanePointLat'), item.get('vpLanePointLon'))
      distance = float(item.get('nLaneDist', item.get('nlaneDist')))
      turn = int(item.get('nLaneTurnCode', item.get('nlaneTurnCode')))
      if lat is None or abs(lat) < 1 or abs(lon) < 1 or not math.isfinite(distance) or not 0 <= distance <= 5000:
        continue
      direction = directions.get(turn)
      if direction is None:
        continue
      point = dict(lat=lat, lon=lon, distance=distance, movement=direction)
      if point not in result:
        result.append(point)
    except (ValueError, TypeError, OverflowError):
      continue
  return sorted(result, key=lambda p: p['distance'])[:32]


def matched_lane_movement(nav, distance, lat, lon):
  """Match this signal, not merely the next maneuver on the whole route."""
  points = nav.get('lane_points', [])
  if not isinstance(points, list):
    return None
  matches = []
  valid = []
  for point in points[:32]:
    try:
      plat, plon = float(point['lat']), float(point['lon'])
      pdist = float(point['distance'])
      direction = point['movement']
      if not _position(plat, plon) or not math.isfinite(pdist) or not 0 <= pdist <= 5000 or direction not in ('straight', 'left', 'right', 'uturn'):
        continue
      valid.append((pdist, direction))
      separation = math.hypot(*_offset(lat, lon, plat, plon))
      if separation <= 30 and abs(pdist-distance) <= max(30., distance*.25):
        matches.append((separation, direction))
    except (KeyError, ValueError, TypeError, OverflowError):
      continue
  if not matches:
    return None
  matches.sort()
  direction = matches[0][1]
  # A nearby competing point or a turn before this signal makes attribution unsafe.
  if any(d != direction and sep <= matches[0][0]+10 for sep, d in matches):
    return 'other'
  if any(d in ('left', 'right', 'uturn') and pdist < distance-40 for pdist, d in valid):
    return 'other'
  return direction


def red_approach(signal, now, v_ego):
  """Only fresh straight-through route signals corroborated by vehicle GPS."""
  try:
    age = now - float(signal['ts'])
    if not 0 <= age <= 3.0:
      return False, 'signal_age'
    nav, gps = signal['navigation'], signal['gps']
    context_age = now - float(signal.get('context_ts', signal['ts']))
    if not gps['valid'] or not 0 <= float(gps['age']) + context_age <= 3.0 or not 0 <= float(nav['age']) + context_age <= 3.0:
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
    reported_now = max(0.0, reported - max(0.0, gps_speed) * age)
    if not math.isfinite(reported) or reported <= 0 or abs(reported_now - distance) > max(25.0, distance * .25):
      return False, 'distance_mismatch'
    turn, turn_distance = int(nav['turn']), float(nav['turn_distance'])
    movement = matched_lane_movement(nav, distance, slat, slon)
    if movement == 'other':
      return False, 'route_direction'
    if movement is None:
      movement = 'straight'
      near_turn = 0 < turn_distance <= distance + 40
      if near_turn and turn in (12, 16, 1000):
        movement = 'left'
      elif near_turn and turn in (13, 19, 1001):
        movement = 'right'
      elif near_turn and turn == 14:
        movement = 'uturn'
      elif turn not in (-1, 0, 11, 202, 12, 16, 1000, 13, 19, 1001, 14):
        return False, 'route_direction'
    light = signal['movements'][movement]
    remaining = float(light['remain']) - age
    if light['state'] != 'RED_LIGHT_ON' or not 2.0 < remaining <= 300:
      return False, movement + '_not_red'
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
      lead_valid = lead.present and (lead.radar or lead.modelProb > .8) and all(
        math.isfinite(float(x)) for x in (lead.dRel, lead.vRel, lead.aLeadK, lead.vLead)) and 0 < lead.dRel < max(20.0, v_ego * 3.0)
      if lead_valid and self.last_distance is not None and abs(lead.dRel - self.last_distance) > max(3.0, v_ego * .3):
        lead_valid = False
      self.last_distance = float(lead.dRel) if lead.present else None
    except (AttributeError, TypeError, ValueError):
      lead_valid = False
      self.last_distance = None
    self.confirm_time = min(1.0, self.confirm_time + dt) if valid else 0.0
    # A verified red signal can suppress acceleration without a lead.
    target = 1.0 if self.confirm_time >= .5 else 0.0
    self.strength += max(-.4 * dt, min(.5 * dt, target - self.strength))
    slowing = lead_valid and (lead.aLeadK < -.4 or lead.vRel < -.5 or lead.vLead < 2.0)
    # Extra distance is v_ego * gap_extra, vanishing near standstill.
    fade = max(0.0, min(1.0, (v_ego - 2.0) / 2.0))
    gap_target = .25 * self.strength * fade if valid and slowing else 0.0
    self.gap_extra += max(-.1 * dt, min(.05 * dt, gap_target - self.gap_extra))
    self.reason = ('active_follow' if slowing else 'active_coast') if target else self.reason

  def cruise_accel(self, normal_max):
    return min(normal_max, normal_max * (1.0 - self.strength) + min(normal_max, 0.0) * self.strength)
