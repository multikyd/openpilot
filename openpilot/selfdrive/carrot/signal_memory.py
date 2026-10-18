"""Display-only signal observations. Learned estimates never enter vehicle control."""
import datetime
import json
import math
import statistics

from openpilot.selfdrive.carrot.signal_follow import normalized_position, _offset, matched_lane_movement

UNKNOWN, RED, GREEN, YELLOW = 0, 1, 2, 3
DIRECTIONS = ('straight', 'left', 'right', 'observed')
LAMP_DIRECTIONS = ('straight', 'left', 'right')


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
              confidence=0., samples=0, operatingMode='', predictionState=0,
              rightState=0, rightRemain=0., locationKind='')


def movement(nav, distance, lat=None, lon=None):
  if lat is not None and lon is not None:
    matched = matched_lane_movement(nav, distance, lat, lon)
    if matched is not None:
      return 'other' if matched == 'uturn' else matched
  turn = int(number(nav.get('turn'), -1))
  turn_distance=number(nav.get('turn_distance'), -1)
  if 0 <= turn_distance and abs(turn_distance-distance)<=40:
    if turn in (12, 16, 1000): return 'left'
    if turn in (13, 19, 1001): return 'right'
    if turn == 14: return 'other'
  if turn in (12,16,1000,13,19,1001,14) and 0<=turn_distance<distance-40:
    return 'other'
  return 'straight' if turn in (-1, 0, 11, 202, 12, 16, 1000, 13, 19, 1001, 14) else 'other'


def fit_cycle(records, wall):
  """Reject non-periodic/ambiguous schedules rather than invent a countdown."""
  sightings=[r for r in records if r['kind']=='seen' and r['state'] in (RED,GREEN) and 0<=wall-r['t']<=21*86400]
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
    consistent=True
    for sight in sightings:
      phase=(sight['t']-anchors[RED])%period
      predicted=GREEN if phase<green_time else RED
      boundary=min(phase,period-phase,abs(phase-green_time))
      if predicted!=sight['state'] and boundary>3:
        consistent=False;break
    if consistent and error <= 2.5 and 5 < green_time < period-5:
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
            if (isinstance(r, dict) and r.get('kind') in ('phase', 'wait', 'mode', 'seen', 'turn', 'model', 'lead') and
                r.get('direction') in DIRECTIONS and r.get('state') in (0,1,2,3) and isinstance(r.get('slot'), str) and
                all(math.isfinite(number(r.get(k), math.nan)) for k in ('t', 'end', 'remain'))):
              records.append(r)
          self.nodes.append(dict(lat=lat, lon=lon, heading=number(n.get('heading')) % 360,
                                 records=records, confirmed=bool(n.get('confirmed')),
                                 location_kind=n.get('location_kind','signal' if n.get('confirmed') else 'candidate')))
    except (ValueError, TypeError, OverflowError):
      self.nodes = []
    self.learn = learn
    self.dirty = False
    self.heading = None
    self.stopped = None
    self.last_vision = None
    self.vision_pending = None
    self.fit_cache = {}
    self.discovery = None
    self.detection_track = None
    self.unlocated_node = None
    self.turn_hint = None
    self.pending_departure = None
    self.model_stop_since = None

  def export(self):
    return dict(version=1, nodes=self.nodes[-128:])

  def _reset_tracking(self):
    self.stopped=self.discovery=self.detection_track=self.last_vision=self.vision_pending=self.pending_departure=self.turn_hint=self.model_stop_since=None

  def _node(self, lat, lon, heading, create=False, confirmed=False):
    closest, best = None, 35.
    for n in self.nodes:
      distance = math.hypot(*_offset(lat, lon, n['lat'], n['lon']))
      if distance < best and abs((n['heading']-heading+180) % 360-180) < 25:
        closest, best = n, distance
    if closest is None and create:
      closest = dict(lat=lat, lon=lon, heading=heading, confirmed=confirmed, records=[],
                     location_kind='signal' if confirmed else 'candidate')
      if self.learn:
        self.nodes.append(closest);self.nodes = self.nodes[-128:];self.dirty = True
    if closest is not None and confirmed and self.learn and (not closest['confirmed'] or closest.get('location_kind')!='signal'):
      closest.update(lat=lat,lon=lon,confirmed=True,location_kind='signal');self.fit_cache.clear();self.dirty = True
    return closest

  def _record(self, node, direction, wall, kind, state=0, remain=0., mode=''):
    if not self.learn or direction not in DIRECTIONS or node.get('location_kind')=='unknown': return
    records = node['records'];end = wall+remain
    if any(r['kind']==kind and r['direction']==direction and
           (kind not in ('seen','model') or r['state']==state) and (kind!='mode' or r.get('mode')==mode) and
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

  def _finish_departure(self, now, wall, lat, lon, heading, steering):
    pending=self.pending_departure
    if pending is None:return
    old,started,plat,plon,initial_heading,intent,max_steer=pending
    max_steer=max(max_steer,abs(steering))
    self.pending_departure=(old,started,plat,plon,initial_heading,intent,max_steer)
    distance=math.hypot(*_offset(plat,plon,lat,lon))
    delta=(heading-initial_heading+180)%360-180
    turn='right' if intent=='right' and delta>30 and max_steer>25 else 'left' if intent=='left' and delta< -30 and max_steer>25 else None
    if distance>=12 and turn:
      self._record(old[3],turn,wall,'turn',mode=turn)
      self.pending_departure=None
    elif distance>=35 and now-started>=6 and abs(delta)<15:
      # A brief lane change with nearly unchanged heading is not an intersection turn.
      direction='observed' if intent else old[4]
      if direction!='right' and old[5]>=4:
        self._record(old[3],direction,old[2],'wait',remain=old[5],mode='lead_move' if len(old)>6 and old[6] else 'ego_move')
      self.pending_departure=None
    elif now-started>20:
      self.pending_departure=None

  def update(self, signal, now, wall, gps, nav, vision, speed, lead_start=False,
             left_blinker=False, right_blinker=False, steering_angle=0., model_hint=0, lead_present=False):
    out = blank()
    if not all(math.isfinite(v) for v in (now,wall,speed)) or wall<1600000000 or speed < -.01:
      self._reset_tracking();return out
    # The speed filter can undershoot zero while stopped (up to 1 cm/s).
    speed = max(0., speed)
    if not gps.get('valid') or not 0 <= number(gps.get('age'), 99) < 3:
      self._reset_tracking();return out
    lat, lon = normalized_position(gps.get('lat'), gps.get('lon'))
    if lat is None or abs(lat)<1 or abs(lon)<1 or not 0<number(gps.get('accuracy'),99)<=15:
      self._reset_tracking();return out
    if speed > 2 or self.heading is None:
      bearing=number(gps.get('bearing'),math.nan)
      if not math.isfinite(bearing):return out
      self.heading=bearing % 360
    heading=self.heading;angle=math.radians(heading)
    steering=number(steering_angle)
    if left_blinker != right_blinker:self.turn_hint=('left' if left_blinker else 'right',now)
    intent=self.turn_hint[0] if self.turn_hint and 0<=now-self.turn_hint[1]<10 else None
    motion_heading=number(gps.get('bearing'),heading) if speed>1 else heading
    self._finish_departure(now,wall,lat,lon,motion_heading,steering)
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
      # Display uses routed TMAP distance, which can differ from GPS straight-line
      # distance through curves or a signal's mapped reference point. Keep the
      # freshness, physical approach and navigation checks; reject implausible ranges.
      range_ok=(0<=reported<=650 and max(0.,distance-40.)<=reported<=max(100.,2.*distance+40.))
      if -15<ahead<650 and side<35 and range_ok:
        distance=reported
        node=self._node(slat,slon,heading,True,True)
        for direction in LAMP_DIRECTIONS:
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
    visual_ok=(0<=visual_age<1 and number(vision.get('confidence'))>=.7 and
               vision.get('color') in ('Red Light','Green Light','Yellow Light','Left turn','Right turn',
                                       'Flashing Yellow','Flashing Red','Off'))
    track=(number(vision.get('x')),number(vision.get('y')))
    if visual_ok:
      previous_track=self.detection_track
      if (previous_track is None or now-previous_track[1]>1.2 or
          max(abs(track[0]-previous_track[3][0]),abs(track[1]-previous_track[3][1]))>=.2):
        self.detection_track=(now,now,number(vision.get('ts'),now),track)
      else:self.detection_track=(previous_track[0],now,previous_track[2],track)
    else:self.detection_track=None
    # TBT is a location candidate, never proof of a red light or a timed cycle.
    if (node is None and nav_ok and 0<=number(nav.get('turn_distance'),-1)<=80 and
        movement(nav, number(nav.get('turn_distance'))) in DIRECTIONS and (visual_ok or speed<.3)):
      distance=number(nav['turn_distance'])
      nlat=lat+math.degrees(distance*math.cos(angle)/6371000)
      nlon=lon+math.degrees(distance*math.sin(angle)/(6371000*math.cos(math.radians(lat))))
      node=self._node(nlat,nlon,heading,True,False)
    # Without a map target, register the observer's stopped position, NOT the
    # physical light location. Fresh sustained detections are required.
    if node is None and visual_ok and speed<.3:
      position=self.discovery
      moved=math.hypot(*_offset(lat,lon,position[1],position[2])) if position else 999
      same_track=position and max(abs(track[0]-position[4][0]),abs(track[1]-position[4][1]))<.2
      if not position or moved>12 or not same_track:
        self.discovery=(now,lat,lon,number(vision.get('ts'),now),track)
      elif (now-position[0]>=.3 and self.detection_track and now-self.detection_track[0]>=2 and
            number(vision.get('ts'),now)-self.detection_track[2]>=1):
        node=self._node(lat,lon,heading,True,False)
        node['location_kind']='stop'
        distance=0.
    elif speed>=.3 or not visual_ok:self.discovery=None
    if model_hint==1 and speed<.3:
      if self.model_stop_since is None:self.model_stop_since=(now,lat,lon)
      if math.hypot(*_offset(lat,lon,*self.model_stop_since[1:]))>12:
        self.model_stop_since=(now,lat,lon)
      if node is None and now-self.model_stop_since[0]>=3:
        node=self._node(lat,lon,heading,True,False)
        node['location_kind']='queue' if lead_present else 'stop'
        distance=0.
    else:self.model_stop_since=None
    if node is None and visual_ok:
      if self.unlocated_node is None:
        self.unlocated_node=dict(lat=lat,lon=lon,heading=heading,confirmed=False,records=[],location_kind='unknown')
      node=self.unlocated_node;distance=0.
    if node is None:
      self.stopped=None;return out
    lane_direction=matched_lane_movement(nav,distance,node['lat'],node['lon']) if nav_ok else None
    route_known=nav_ok and (int(number(nav.get('turn'),-1)) not in (-1,0) or lane_direction is not None)
    direction=movement(nav,distance,node['lat'],node['lon']) if route_known else intent if intent else 'observed'
    out['distance']=max(0.,distance);out['movement']=direction
    out['locationKind']=node.get('location_kind','candidate')
    if model_hint in (1,2):
      self._record(node,direction,wall,'model',state=int(model_hint))
    if visual_ok and not current:
      color=vision.get('color','')
      mode={'Flashing Yellow':'FLASH','Flashing Red':'FLASH','Off':'OFF'}.get(color,'NORMAL')
      if mode!='NORMAL':
        out.update(source='VISION',operatingMode=mode)
        self._record(node,direction,wall,'mode',mode=mode)
        self.last_vision=None;return out
      state={'Red Light':RED,'Green Light':GREEN,'Yellow Light':YELLOW,'Left turn':GREEN}.get(color,0)
      observed_direction='left' if color=='Left turn' else 'right' if color=='Right turn' else 'straight'
      if color=='Right turn':state=GREEN
      if state:
        pending_key=(id(node),observed_direction,state)
        if self.vision_pending is None or self.vision_pending[0]!=pending_key:
          self.vision_pending=(pending_key,now)
        if now-self.vision_pending[1]<.25:
          return out
        current[observed_direction]=(state,0.);out.update(source='VISION',confidence=.7,operatingMode='NORMAL')
        learning_direction='observed' if observed_direction=='straight' and not route_known else observed_direction
        observed_wall=wall-max(0.,now-self.vision_pending[1])
        self._record(node,learning_direction,observed_wall,'seen',state)
        # Actual color transitions, not ego/lead movement, can anchor phase learning.
        previous=self.last_vision
        if (previous and previous[0] is node and previous[1]==observed_direction and
            0<=now-previous[3]<1 and previous[2] in (RED,GREEN) and state in (RED,GREEN) and previous[2]!=state):
          self._record(node,learning_direction,observed_wall,'phase',previous[2],0.)
        self.last_vision=(node,observed_direction,state,now)
    elif not visual_ok:self.last_vision=None;self.vision_pending=None
    for d,(state,remain) in current.items():out[d+'State']=state;out[d+'Remain']=remain
    # Queue stops remain weak evidence. They never train red/green phase boundaries.
    identity=(id(node),direction)
    if self.stopped and lead_start and lead_present:
      self._record(node,direction,wall,'lead',mode='moving_lead')
    if speed<.3 and distance<85 and direction in DIRECTIONS:
      if self.stopped is None or self.stopped[0]!=identity:self.stopped=(identity,now,wall,node,direction)
    elif self.stopped is not None:
      old=self.stopped
      if speed>1.5:
        duration=now-old[1]
        if .2<=duration<=240:
          saved=(*old[:5],duration,bool(lead_start))
          self.pending_departure=(saved,now,lat,lon,heading,intent,abs(steering))
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
      elif model_hint in (1,2):
        out.update(source='MODEL',operatingMode='MODEL_STOP' if model_hint==1 else 'MODEL_MOVE')
    return out
