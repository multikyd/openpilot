"""Local, directional bump memory. Thresholds are provisional, not height estimates."""
import math
from collections import deque

MAX_RECORDS = 256
SMOOTH_PEAK = 1.5  # m/s^2, gravity-projected, high-pass acceleration
ROUGH_PEAK = 3.0
UNKNOWN_PEAK = 3.5


def offset(lat, lon, heading, meters):
  angle = math.radians(heading)
  return (lat + math.cos(angle) * meters / 111320,
          lon + math.sin(angle) * meters / (111320 * max(.1, math.cos(math.radians(lat)))))


def relative(lat, lon, heading, record):
  north = (record['lat'] - lat) * 111320
  east = (record['lon'] - lon) * 111320 * math.cos(math.radians(lat))
  angle = math.radians(heading)
  return north * math.cos(angle) + east * math.sin(angle), east * math.cos(angle) - north * math.sin(angle)


def direction_matches(a, b):
  return abs((a - b + 180) % 360 - 180) <= 25


class BumpMemory:
  def __init__(self, data=None):
    self.records = []
    if isinstance(data, dict) and data.get('version') == 1 and isinstance(data.get('records'), list):
      for r in data.get('records', [])[-MAX_RECORDS:]:
        try:
          vals = [float(r[k]) for k in ('lat', 'lon', 'heading', 'target')]
          if not all(math.isfinite(v) for v in vals) or not (-90 < vals[0] < 90 and -180 < vals[1] < 180):
            continue
          clean = dict(r)
          clean.update(zip(('lat', 'lon', 'heading', 'target'), vals))
          clean['target'] = max(10, min(50, clean['target']))
          clean['passes'] = max(0, int(r.get('passes', 0)))
          clean['smooth'] = max(0, min(2, int(r.get('smooth', 0))))
          clean['confirmed'] = bool(r.get('confirmed', False))
          # Recover legacy impact provenance when it is supported by the saved peak.
          peak = float(r.get('last_peak', 0))
          clean['impact_learned'] = bool(r.get('impact_learned', False)) or (math.isfinite(peak) and peak >= ROUGH_PEAK)
          cap = float(r.get('manual_cap', 50))
          if not math.isfinite(cap):
            continue
          clean['manual_cap'] = max(10, min(50, cap))
          self.records.append(clean)
        except (KeyError, TypeError, ValueError, OverflowError):
          continue
    self.gravity = None
    self.sensor_start = self.sensor_last = None
    self.samples = deque(maxlen=1600)
    self.position = None
    self.pending = None
    self.cooldowns = {}
    self.notice = ''
    self.notice_until = 0
    self.sound_seq = 0
    self.sound_last = -100
    self.dirty = False
    self.manual_until = 0
    self.base = 30
    self.raise_limit = 10
    self.last_peak = self.last_pass_speed = 0.0

  def export(self):
    return {'version': 1, 'records': self.records}

  def sensor(self, stamp, vector):
    if len(vector) != 3 or not all(math.isfinite(x) for x in vector):
      return
    norm = math.sqrt(sum(x*x for x in vector))
    if not (1 < norm < 40) or (self.sensor_last is not None and stamp <= self.sensor_last):
      return
    dt = .01 if self.sensor_last is None else stamp - self.sensor_last
    if self.gravity is None or dt > .3:
      self.gravity = list(vector)
      self.sensor_start = stamp
    self.sensor_last = stamp
    gnorm = math.sqrt(sum(x*x for x in self.gravity))
    vertical = sum((v-g)*g for v, g in zip(vector, self.gravity)) / max(gnorm, 1)
    alpha = min(dt / 3, 1)
    self.gravity = [g + alpha*(v-g) for v, g in zip(vector, self.gravity)]
    if stamp - self.sensor_start >= 2:
      self.samples.append((stamp, abs(vertical)))

  def gps(self, stamp, lat, lon, heading, accuracy):
    if (all(math.isfinite(v) for v in (stamp, lat, lon, heading, accuracy)) and
        -90 < lat < 90 and -180 < lon < 180 and (lat != 0 or lon != 0) and 0 < accuracy <= 8):
      self.position = (stamp, lat, lon, heading)
    else:
      self.position = None

  def _find(self, lat, lon, heading):
    closest = None
    best = 12
    for r in self.records:
      if direction_matches(heading, r['heading']):
        along, cross = relative(lat, lon, heading, r)
        dist = math.hypot(along, cross)
        if dist < best:
          best, closest = dist, r
    return closest

  def _record(self, lat, lon, heading, known):
    r = self._find(lat, lon, heading)
    if r is None:
      r = dict(lat=lat, lon=lon, heading=heading, target=self.base, confirmed=known,
               passes=0, smooth=0, manual_cap=50, impact_learned=not known)
      self.records.append(r)
      if len(self.records) > MAX_RECORDS:
        self.records.pop(0)
      self.dirty = True
    elif known and not r['confirmed']:
      r['confirmed'] = True
      self.dirty = True
    return r

  def _notify(self, now, text):
    self.notice, self.notice_until = text, now + 5

  def finish_pass(self, record, peak, above, samples, speed, manual, eligible, now):
    # Missing/undersampled IMU must never count as a smooth pass.
    if samples < 15 or speed < 10 or not all(math.isfinite(v) for v in (peak, speed)):
      return
    record['passes'] += 1
    if record['passes'] >= 2:
      record['confirmed'] = True
    old = record['target']
    if manual and speed < old - 1:
      record['manual_cap'] = min(record['manual_cap'], max(10, round(speed)))
      record['target'] = min(old, record['manual_cap'])
    # A sustained hard impact reduces the target even after a driver's slowdown.
    if peak >= ROUGH_PEAK and above >= 2:
      record['impact_learned'] = True
      record['target'] = max(10, min(record['target'] - 5, math.floor((speed - 5)/5)*5))
      record['smooth'] = 0
      self._notify(now, f"큰 충격 · 다음 {record['target']:.0f}")
    elif peak < SMOOTH_PEAK and eligible and not manual and abs(speed-old) <= 2:
      record['smooth'] += 1
      if record['smooth'] >= 3:
        record['target'] = min(old + 5, self.base + self.raise_limit, record['manual_cap'], 50)
        record['smooth'] = 0
      self._notify(now, f"부드러움 · 다음 {record['target']:.0f}")
    else:
      record['smooth'] = 0
      self._notify(now, f"목표 유지 {record['target']:.0f}")
    record['last_peak'] = round(peak, 3)
    record['last_speed'] = round(speed, 1)
    record['last_manual'] = bool(manual)
    record['last_time'] = now  # diagnostic monotonic only; never persisted as a cooldown
    self.dirty = True
    self.last_peak, self.last_pass_speed = peak, speed

  def tick(self, now, speed, nav_distance=None, manual=False, eligible=False):
    if manual:
      self.manual_until = now + 12
    if self.position is None or not (0 <= now-self.position[0] <= 1) or not math.isfinite(speed) or speed < 3:
      self.pending = None
      return None
    stamp, lat, lon, heading = self.position
    # Align latest GPS reception with current vehicle motion (bounded to one second).
    lat, lon = offset(lat, lon, heading, speed/3.6 * (now-stamp))
    fresh_sensor = self.sensor_last is not None and 0 <= now-self.sensor_last < .3
    known = nav_distance is not None and math.isfinite(nav_distance) and 0 < nav_distance < 250
    if known:
      blat, blon = offset(lat, lon, heading, nav_distance)
      r = self._find(blat, blon, heading)
      if r is None and nav_distance < 100:
        self._record(blat, blon, heading, True)
    nearest, nearest_d = None, 251
    for r in self.records:
      if not direction_matches(heading, r['heading']):
        continue
      d, cross = relative(lat, lon, heading, r)
      if abs(cross) <= 6 and -12 <= d < nearest_d and now-self.cooldowns.get(id(r), -100) > 60:
        nearest, nearest_d = r, d
    # Unknown impacts are candidates only; two independent passes confirm them.
    recent = [(t, a) for t, a in self.samples if now-.6 <= t <= now]
    if self.pending is None and nearest is None and fresh_sensor and speed >= 10:
      peaks = [(t, a) for t, a in recent if a >= UNKNOWN_PEAK]
      if len(peaks) >= 2:
        impact_t, _ = max(peaks, key=lambda x: x[1])
        ilat, ilon = offset(lat, lon, heading, -speed/3.6*(now-impact_t))
        r = self._record(ilat, ilon, heading, False)
        if now-self.cooldowns.get(id(r), -100) > 60:
          self.finish_pass(r, max(a for _, a in recent), len(peaks), len(recent), speed,
                           now < self.manual_until, False, now)
          self.cooldowns[id(r)] = now
          self._notify(now, '노면 충격 저장')
    if self.pending is None and nearest is not None and 0 < nearest_d <= 35:
      self.pending = dict(record=nearest, start=now, manual=now < self.manual_until,
                          eligible=eligible, crossing=None, speed=speed, valid=True)
    if self.pending is not None:
      p = self.pending
      d, cross = relative(lat, lon, heading, p['record'])
      p['manual'] |= now < self.manual_until
      p['eligible'] &= eligible
      p['valid'] &= fresh_sensor
      if abs(cross) > 8 or not direction_matches(heading, p['record']['heading']) or now-p['start'] > 25:
        self.pending = None
      else:
        if d <= 0 and p['crossing'] is None:
          p['crossing'], p['speed'] = now, speed
        if p['crossing'] is not None and now-p['crossing'] >= 1.2 and d < -5:
          window = [(t, a) for t, a in self.samples if p['crossing']-.8 <= t <= p['crossing']+1.2]
          # Reject long holes, low rate and truncated recordings; never infer smoothness from silence.
          coverage = (len(window) >= 15 and window[-1][0]-window[0][0] >= 1.5 and
                      max((b[0]-a[0] for a, b in zip(window, window[1:])), default=1) <= .1)
          if p['valid'] and coverage:
            self.finish_pass(p['record'], max(a for _, a in window), sum(a >= ROUGH_PEAK for _, a in window),
                             len(window), p['speed'], p['manual'], p['eligible'], now)
          self.cooldowns[id(p['record'])] = now
          self.pending = None
    if nearest is not None and nearest['confirmed'] and nearest_d >= -8:
      target = min(nearest['target'], self.base + self.raise_limit, nearest['manual_cap'])
      notify = nearest.get('impact_learned', False) or abs(target - self.base) >= 0.5
      if notify and 0 < nearest_d < 120 and now-self.sound_last >= 15 and getattr(self, '_announced', None) != id(nearest):
        self.sound_seq += 1
        self.sound_last = now
        # One sound per encounter, independently of passing/learning success.
        self._announced = id(nearest)
      return dict(target=target, distance=nearest_d)
    self._announced = None
    return None
