"""Driver-confirmed stopped vehicle positions, not inferred traffic-light locations."""
import json
import math
import statistics

from openpilot.selfdrive.carrot.signal_follow import _offset, normalized_position


def geometry(lat, lon, heading, target):
  north, east = _offset(lat, lon, target['lat'], target['lon'])
  angle = math.radians(heading)
  return north*math.cos(angle)+east*math.sin(angle), east*math.cos(angle)-north*math.sin(angle)


class ManualStopLineMemory:
  def __init__(self, stored=None):
    self.points = []
    try:
      data = json.loads(stored) if isinstance(stored, (bytes, str)) else stored
      if data and data.get('version') != 1:
        data = {}
      for point in (data or {}).get('points', [])[-64:]:
        if point.get('movement') not in ('straight', 'left'):
          continue
        lat, lon = normalized_position(point.get('lat'), point.get('lon'))
        samples = [s for s in point.get('samples', [])[-12:] if
                   all(isinstance(s.get(k), (int, float)) and math.isfinite(s[k])
                       for k in ('lat', 'lon', 'wall', 'accuracy')) and 0 < s['accuracy'] <= 2.5 and
                   normalized_position(s['lat'], s['lon']) == (s['lat'], s['lon'])]
        heading = float(point['heading'])
        if lat is not None and abs(lat)>1 and abs(lon)>1 and math.isfinite(heading) and samples:
          self.points.append(dict(lat=lat, lon=lon, heading=heading % 360,
                                  movement=point['movement'], samples=samples))
    except (ValueError, TypeError, KeyError, AttributeError):
      self.points = []
    self.dirty = False
    self.reset()

  def reset(self):
    self.last_time = None
    self.manual_until = -1.
    self.samples = []
    self.last_gps = None
    self.bookmark = None
    self.confirm_mark = None
    self.identity = None

  def export(self):
    return dict(version=1, points=self.points[-64:])

  def observe(self, now, wall, gps, speed, heading, movement, red, no_lead,
              brake, gas, steering, drive, bookmark_stamp=None, enabled=True):
    try:
      lat, lon = normalized_position(gps['lat'], gps['lon'])
      valid = (enabled and gps['valid'] and 0 <= gps['age'] < .8 and
               lat is not None and abs(lat)>1 and abs(lon)>1 and
               all(math.isfinite(v) for v in (now, wall, speed, heading, steering, gps['accuracy'], gps['stamp'])) and
               wall>1600000000 and 0<gps['accuracy']<=2.5 and red and no_lead and drive and
               movement in ('straight','left') and abs(steering)<20 and -.01<=speed<15.)
    except (TypeError, KeyError, ValueError):
      valid = False
    if not valid:
      self.reset()
      return False
    if self.last_time is not None and not 0<now-self.last_time<=.8:
      self.reset()
    self.last_time = now
    # Deliberate brake intervention, or pedal-controlled repositioning, is required.
    if brake or gas:
      self.manual_until = now+60.
    identity = movement
    if self.identity != identity:
      self.samples = []
      self.confirm_mark = None
      self.identity = identity
    if speed>=.15:
      # Any creep invalidates the old position and an earlier confirmation.
      self.samples = []
      self.confirm_mark = None
      if bookmark_stamp is not None:self.bookmark = bookmark_stamp
      self.last_gps = gps['stamp']
      return False
    if gps['stamp'] != self.last_gps:
      if self.samples and math.hypot(*_offset(lat,lon,self.samples[-1][1],self.samples[-1][2]))>1.5:
        self.samples = []
        self.confirm_mark = None
      self.samples.append((now,lat,lon,heading,gps['accuracy']))
      self.samples = [s for s in self.samples if now-s[0]<=3.]
      self.last_gps = gps['stamp']
    fresh_confirmation = (bookmark_stamp is not None and bookmark_stamp != self.bookmark and
                          0<=now-bookmark_stamp<=.8)
    if bookmark_stamp is not None:self.bookmark = bookmark_stamp
    if not fresh_confirmation or now>self.manual_until or len(self.samples)<8 or now-self.samples[0][0]<1.5:
      return False
    # A single swipe remains an ordinary issue bookmark.
    if self.confirm_mark is None or not 0<bookmark_stamp-self.confirm_mark<=4.:
      self.confirm_mark=bookmark_stamp
      return False
    self.confirm_mark=None
    plat = statistics.median(s[1] for s in self.samples)
    plon = statistics.median(s[2] for s in self.samples)
    if max(math.hypot(*_offset(plat,plon,s[1],s[2])) for s in self.samples)>1.:
      return False
    # Known passed points are rejected, even if the driver bookmarks them.
    for point in self.points:
      ahead,side = geometry(plat,plon,heading,point)
      if point['movement']==movement and abs((heading-point['heading']+180)%360-180)<12 and abs(side)<1.5 and -30<ahead< -2:
        return False
    point = next((p for p in self.points if p['movement']==movement and
                  abs((heading-p['heading']+180)%360-180)<12 and
                  math.hypot(*_offset(plat,plon,p['lat'],p['lon']))<2.5),None)
    if point is None:
      point=dict(lat=plat,lon=plon,heading=heading,movement=movement,samples=[])
      self.points.append(point);self.points=self.points[-64:]
    # Repeated swipes while waiting do not count as independent visits.
    if point['samples'] and wall-point['samples'][-1]['wall']<600:
      return False
    point['samples'].append(dict(lat=plat,lon=plon,wall=wall,accuracy=max(s[4] for s in self.samples)))
    point['samples']=[s for s in point['samples'][-12:] if 0<=wall-s['wall']<=30*86400]
    point['lat']=statistics.median(s['lat'] for s in point['samples'])
    point['lon']=statistics.median(s['lon'] for s in point['samples'])
    self.dirty=True
    self.manual_until=-1.
    return True

  def target(self, now, wall, gps, speed, heading, movement, red, no_lead, enabled=True):
    try:
      if (not enabled or not red or not no_lead or not gps['valid'] or not 0<=gps['age']<.8 or
          not 0<gps['accuracy']<=2.5 or not 0.5<=speed<=8. or
          not all(math.isfinite(v) for v in (now,wall,speed,heading,gps['lat'],gps['lon']))):
        return None
      matches=[]
      for point in self.points:
        samples=[s for s in point['samples'] if 0<=wall-s['wall']<=30*86400]
        if (point['movement']!=movement or len(samples)<3 or len({int(s['wall']//86400) for s in samples})<2 or
            abs((heading-point['heading']+180)%360-180)>12):
          continue
        scatter=max(math.hypot(*_offset(point['lat'],point['lon'],s['lat'],s['lon'])) for s in samples)
        ahead,side=geometry(gps['lat'],gps['lon'],heading,point)
        if scatter<=1. and 4.<ahead<40. and abs(side)<1.2:
          matches.append(dict(ts=now,distance=ahead,lateral=side,samples=len(samples),scatter=scatter,
                              accuracy=gps['accuracy'],movement=movement))
      # Multiple plausible locations cannot establish a unique stop target.
      return matches[0] if len(matches)==1 else None
    except (TypeError, ValueError, KeyError):
      return None


class ManualStopLineCorrection:
  """Only shorten an existing model stop; never create a stop or extend it."""
  def __init__(self):
    self.offset=0.

  def update(self, stop, target, now, speed, clearance, traffic_adjust, active, dt):
    try:
      valid = (active and isinstance(target,dict) and 0<=now-target['ts']<=.5 and
               target['samples']>=3 and target['scatter']<=1. and 0<target['accuracy']<=2.5 and
               abs(target['lateral'])<1.2 and 4.<target['distance']<40. and .5<=speed<=8. and
               all(math.isfinite(v) for v in (stop,now,speed,clearance,traffic_adjust,dt,*
                   [target[k] for k in ('ts','distance','lateral','scatter','accuracy')])) and
               0<dt<=.2 and 2.<=clearance<=10. and 0<stop<100.)
    except (KeyError,TypeError,ValueError):
      valid=False
    if not valid:
      if not active:
        self.offset=0.
      # Losing GPS must not lengthen a stop already shortened in this episode.
      return max(0.,stop+self.offset) if active and math.isfinite(stop) else stop
    # The MPC obstacle includes standstill clearance and the traffic adjustment.
    obstacle=target['distance']+clearance-traffic_adjust
    desired=max(-1.,min(self.offset,obstacle-stop))
    self.offset+=max(-.2*dt,min(.2*dt,desired-self.offset))
    return max(0.,stop+self.offset)
