"""Display-only signal observations. Learned estimates never enter vehicle control."""
import datetime
import json
import math
import statistics

from openpilot.selfdrive.carrot.signal_follow import normalized_position, _offset

UNKNOWN, RED, GREEN, YELLOW = 0, 1, 2, 3
DIRECTIONS = ('straight', 'left')


def number(value, default=0.):
  try:
    value = float(value)
    return value if math.isfinite(value) else default
  except (TypeError, ValueError, OverflowError):
    return default


def slot(wall):
  local = datetime.datetime.fromtimestamp(wall)
  return f'{int(local.weekday() >= 5)}:{local.hour * 2 + local.minute // 30}'


def blank():
  return dict(source='', distance=0., movement='straight', straightState=0, leftState=0,
              straightRemain=0., leftRemain=0., estimateLow=0., estimateHigh=0.,
              confidence=0., samples=0, operatingMode='', predictionState=0)


def movement(nav, distance):
  turn = int(number(nav.get('turn'), -1))
  turn_distance=number(nav.get('turn_distance'), -1)
  if 0 <= turn_distance and abs(turn_distance-distance)<=40:
    if turn in (12, 16, 1000): return 'left'
    if turn in (13, 19, 1001, 14): return 'other'
  if turn in (12,16,1000,13,19,1001,14) and 0<=turn_distance<distance-40:
    return 'other'
  return 'straight' if turn in (-1, 0, 11, 202, 12, 16, 1000, 13, 19, 1001, 14) else 'other'


def fit_cycle(records, wall):
  """Reject non-periodic/ambiguous schedules rather than invent a countdown."""
  records = [r for r in records if 0 <= wall-r['t'] <= 21*86400 and r['kind'] == 'phase']
  reds = [r for r in records if r['state'] == RED]
  greens = [r for r in records if r['state'] == GREEN]
  if min(len(reds), len(greens)) < 3 or len({int(r['t']//86400) for r in records}) < 3:
    return None
  minimum = max(r['remain'] for r in reds) + max(r['remain'] for r in greens)
  candidates = []
  for half in range(max(80, math.ceil(minimum*2)), 721):
    period = half/2
    anchors, error = {}, 0.
    for state, values in ((RED, reds), (GREEN, greens)):
      origin = values[-1]['end']
      residuals = [((r['end']-origin+period/2) % period)-period/2 for r in values]
      center = statistics.median(residuals)
      error = max(error, max(abs(v-center) for v in residuals))
      anchors[state] = origin+center
    green_time = (anchors[GREEN]-anchors[RED]) % period
    if error <= 2.5 and 5 < green_time < period-5:
      candidates.append((error, period, anchors, green_time))
  if not candidates: return None
  candidates.sort(key=lambda c:c[0])
  best = candidates[0]
  if any(abs(c[1]-best[1]) > .1 and c[0] < best[0]+1.5 for c in candidates[1:]):
    return None
  return best


class SignalMemory:
  def __init__(self, stored=None, learn=True):
    self.nodes = []
    try:
      data = json.loads(stored) if isinstance(stored, (str, bytes)) else stored
      if isinstance(data, dict) and data.get('version') == 1:
        for n in data.get('nodes', [])[-128:]:
          if not isinstance(n, dict): continue
          lat, lon = normalized_position(n.get('lat'), n.get('lon'))
          if lat is None or abs(lat) < 1 or abs(lon) < 1: continue
          records = []
          for r in n.get('records', [])[-192:]:
            if (isinstance(r, dict) and r.get('kind') in ('phase', 'wait', 'mode') and
                r.get('direction') in DIRECTIONS and r.get('state') in (0,1,2,3) and isinstance(r.get('slot'), str) and
                all(math.isfinite(number(r.get(k), math.nan)) for k in ('t', 'end', 'remain'))):
              records.append(r)
          self.nodes.append(dict(lat=lat, lon=lon, heading=number(n.get('heading')) % 360,
                                 records=records, confirmed=bool(n.get('confirmed'))))
    except (ValueError, TypeError, OverflowError):
      self.nodes = []
    self.learn = learn
    self.dirty = False
    self.heading = None
    self.stopped = None
    self.last_vision = None
    self.vision_pending = None
    self.fit_cache = {}

  def export(self):
    return dict(version=1, nodes=self.nodes[-128:])

  def _node(self, lat, lon, heading, create=False, confirmed=False):
    closest, best = None, 35.
    for n in self.nodes:
      distance = math.hypot(*_offset(lat, lon, n['lat'], n['lon']))
      if distance < best and abs((n['heading']-heading+180) % 360-180) < 25:
        closest, best = n, distance
    if closest is None and create:
      closest = dict(lat=lat, lon=lon, heading=heading, confirmed=confirmed, records=[])
      if self.learn:
        self.nodes.append(closest);self.nodes = self.nodes[-128:];self.dirty = True
    if closest is not None and confirmed and self.learn and not closest['confirmed']:
      closest.update(lat=lat,lon=lon,confirmed=True);self.fit_cache.clear();self.dirty = True
    return closest

  def _record(self, node, direction, wall, kind, state=0, remain=0., mode=''):
    if not self.learn or direction not in DIRECTIONS: return
    records = node['records'];end = wall+remain
    if any(r['kind']==kind and r['direction']==direction and
           (abs(r['end']-end)<3 if kind=='phase' else abs(r['t']-wall)<30)
           for r in records[-12:]): return
    if kind=='phase':
      same=[r for r in records if r['direction']==direction and r['slot']==slot(wall)]
      old_fit=fit_cycle(same,wall)
      if old_fit:
        _,period,anchors,_=old_fit
        mismatch=abs((end-anchors[state]+period/2)%period-period/2)
        if mismatch>8:
          records=[r for r in records if not (r['kind']=='phase' and r['direction']==direction and r['slot']==slot(wall))]
    records.append(dict(kind=kind, direction=direction, slot=slot(wall), t=wall,
                        end=end, state=state, remain=remain, mode=mode))
    if kind=='mode':
      records=[r for r in records if not (r['kind']=='phase' and r['direction']==direction and r['slot']==slot(wall))]
    node['records'] = [r for r in records[-192:] if wall-r['t'] < 21*86400]
    if kind=='phase' and not node['confirmed']:
      phases=[r for r in node['records'] if r['kind']=='phase']
      if len(phases)>=6 and len({int(r['t']//86400) for r in phases})>=3:
        node['confirmed']=True
    self.fit_cache.clear();self.dirty = True

  def _estimate(self, node, direction, wall, elapsed):
    records = [r for r in node['records'] if r['direction']==direction and r['slot']==slot(wall)]
    mode_change=max((r['t'] for r in node['records'] if r['kind']=='mode'),default=0)
    records=[r for r in records if r['t']>mode_change]
    key = (node['lat'], node['lon'], direction, slot(wall), int(wall//3600))
    if key not in self.fit_cache:
      self.fit_cache[key] = fit_cycle(records, wall) if node['confirmed'] else None
      if len(self.fit_cache)>256:self.fit_cache={key:self.fit_cache[key]}
    model = self.fit_cache[key]
    if model:
      error, period, anchors, green_time = model
      age = wall-max(r['t'] for r in records if r['kind']=='phase')
      uncertainty = max(2., error+1.) + age/3600*.15
      phase = (wall-anchors[RED]) % period
      remaining, state = (green_time-phase, GREEN) if phase < green_time else (period-phase, RED)
      if 0 <= age <= 7*86400 and uncertainty < 12 and remaining > uncertainty:
        return remaining-uncertainty, remaining+uncertainty, state, .65, 'NORMAL'
    waits = [r['remain'] for r in records if r['kind']=='wait' and 0 <= wall-r['t'] < 21*86400]
    days = {int(r['t']//86400) for r in records if r['kind']=='wait'}
    if elapsed is not None and len(waits)>=5 and len(days)>=3:
      waits.sort();low=waits[len(waits)//5];high=waits[min(len(waits)-1, len(waits)*4//5)]
      if high-low <= 20 and elapsed < high:
        return max(0.,low-elapsed), high-elapsed, UNKNOWN, .3, 'WAIT'
    return None

  def update(self, signal, now, wall, gps, nav, vision, speed, lead_start=False):
    out = blank()
    if not all(math.isfinite(v) for v in (now,wall,speed)) or wall<1600000000 or speed<0:
      self.stopped=None;return out
    if not gps.get('valid') or not 0 <= number(gps.get('age'), 99) < 3:
      self.stopped=None;self.last_vision=None;return out
    lat, lon = normalized_position(gps.get('lat'), gps.get('lon'))
    if lat is None or abs(lat)<1 or abs(lon)<1 or not 0<number(gps.get('accuracy'),99)<=15:
      self.stopped=None;return out
    if speed > 2 or self.heading is None:
      bearing=number(gps.get('bearing'),math.nan)
      if not math.isfinite(bearing):return out
      self.heading=bearing % 360
    heading=self.heading;angle=math.radians(heading)
    def geometry(n):
      north,east=_offset(lat,lon,n['lat'],n['lon'])
      return north*math.cos(angle)+east*math.sin(angle), abs(east*math.cos(angle)-north*math.sin(angle))
    # Route context must agree with physical GPS; simulated navigation is excluded.
    nlat,nlon=normalized_position(nav.get('lat'),nav.get('lon'))
    nav_ok=(0<=number(nav.get('age'),99)<3 and nlat is not None and
            math.hypot(*_offset(lat,lon,nlat,nlon))<30 and
            abs(number(nav.get('speed'))/3.6-speed)<4 and
            (speed<2 or abs((number(nav.get('bearing'),heading)-heading+180)%360-180)<25))
    node=None;distance=0.;current={}
    signal=signal if isinstance(signal,dict) else {}
    age=now-number(signal.get('ts'),-100)
    slat,slon=normalized_position(signal.get('lat'),signal.get('lon'))
    if nav_ok and 0<=age<=3 and slat is not None and abs(slat)>1 and abs(slon)>1:
      ahead,side=geometry(dict(lat=slat,lon=slon))
      distance=math.hypot(*_offset(lat,lon,slat,slon))
      reported=number(signal.get('distance'),-100)-max(0.,speed)*age
      if -15<ahead<650 and side<35 and abs(reported-distance)<max(30.,distance*.3):
        node=self._node(slat,slon,heading,True,True)
        for direction in DIRECTIONS:
          light=signal.get('movements',{}).get(direction,{})
          state={'RED_LIGHT_ON':RED,'GREEN_LIGHT_ON':GREEN,'YELLOW_LIGHT_ON':YELLOW}.get(light.get('state'),0)
          remaining=number(light.get('remain'))-age
          current[direction]=(state,remaining if 0<remaining<=300 else 0.)
          if node is not None and state in (RED,GREEN) and 0<remaining<=300:
            self._record(node,direction,wall,'phase',state,remaining)
        out['source']='TMAP';out['confidence']=1.;out['operatingMode']='NORMAL'
    if node is None:
      # Match an already observed intersection ahead, separated by approach heading.
      candidates=[]
      for n in self.nodes:
        ahead,side=geometry(n)
        if -10<ahead<500 and side<25 and abs((n['heading']-heading+180)%360-180)<25:
          candidates.append((max(0.,ahead),n))
      if candidates:distance,node=min(candidates,key=lambda x:x[0])
    visual_age=now-number(vision.get('ts'),-100)
    visual_ok=0<=visual_age<1 and number(vision.get('confidence'))>=.7
    # TBT is a location candidate, never proof of a red light or a timed cycle.
    if (node is None and nav_ok and 0<=number(nav.get('turn_distance'),-1)<=80 and
        movement(nav, number(nav.get('turn_distance'))) in DIRECTIONS and (visual_ok or speed<.3)):
      distance=number(nav['turn_distance'])
      nlat=lat+math.degrees(distance*math.cos(angle)/6371000)
      nlon=lon+math.degrees(distance*math.sin(angle)/(6371000*math.cos(math.radians(lat))))
      node=self._node(nlat,nlon,heading,True,False)
    if node is None:
      self.stopped=None;return out
    direction=movement(nav,distance) if nav_ok else 'other'
    out['distance']=max(0.,distance);out['movement']=direction
    if visual_ok and not current:
      color=vision.get('color','')
      mode={'Flashing Yellow':'FLASH','Flashing Red':'FLASH','Off':'OFF'}.get(color,'NORMAL')
      if mode!='NORMAL':
        out.update(source='VISION',operatingMode=mode)
        self._record(node,direction,wall,'mode',mode=mode)
        self.last_vision=None;return out
      state={'Red Light':RED,'Green Light':GREEN,'Yellow Light':YELLOW,'Left turn':GREEN}.get(color,0)
      observed_direction='left' if color=='Left turn' else 'straight'
      if state:
        pending_key=(id(node),observed_direction,state)
        if self.vision_pending is None or self.vision_pending[0]!=pending_key:
          self.vision_pending=(pending_key,now)
        if now-self.vision_pending[1]<.25:
          return out
        current[observed_direction]=(state,0.);out.update(source='VISION',confidence=.7,operatingMode='NORMAL')
        # Actual color transitions, not ego/lead movement, can anchor phase learning.
        previous=self.last_vision
        if (previous and previous[0] is node and previous[1]==observed_direction and
            0<=now-previous[3]<1 and previous[2] in (RED,GREEN) and state in (RED,GREEN) and previous[2]!=state):
          self._record(node,observed_direction,wall,'phase',previous[2],0.)
        self.last_vision=(node,observed_direction,state,now)
    elif not visual_ok:self.last_vision=None;self.vision_pending=None
    for d,(state,remain) in current.items():out[d+'State']=state;out[d+'Remain']=remain
    # Queue stops remain weak evidence. They never train red/green phase boundaries.
    identity=(id(node),direction)
    if speed<.3 and distance<85 and direction in DIRECTIONS:
      if self.stopped is None or self.stopped[0]!=identity:self.stopped=(identity,now,wall,node,direction)
    elif self.stopped is not None:
      old=self.stopped
      if speed>1.5:
        duration=now-old[1]
        if 4<=duration<=240:self._record(old[3],old[4],old[2],'wait',remain=duration,mode='lead' if lead_start else 'ego')
        self.stopped=None
      elif identity!=old[0]:self.stopped=None
    out['samples']=min(65535,len([r for r in node['records'] if r['direction']==direction and r['slot']==slot(wall)]))
    if not out['source']:
      out['source']='LEARN'
      elapsed=now-self.stopped[1] if self.stopped and self.stopped[0]==identity else None
      prediction=self._estimate(node,direction,wall,elapsed) if direction in DIRECTIONS else None
      if prediction:
        low,high,state,confidence,mode=prediction
        out.update(source='EST',estimateLow=low,estimateHigh=high,predictionState=state,
                   confidence=confidence,operatingMode=mode)
    return out
