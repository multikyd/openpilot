"""Normalize navigation signal messages at the app boundary."""
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
  out = dict(lat=lat, lon=lon, distance=data.get('distance'),
             _source_timestamp_ms=data.get('_source_timestamp_ms'))
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
