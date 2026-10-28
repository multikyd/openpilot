"""Experimental leadOne-gated lamp observation. Never consumed by vehicle control.

No trained vehicle/lamp detector: projection is a search region, not a vehicle box.
Causal per-target observers require a brightness/colour transition; startup is UNKNOWN.
Centre-lamp changes can confirm ON independently of a side-lamp brightness rise.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Observation:
  state: str = "UNKNOWN"
  reason: str = "SEARCH"
  lamps: tuple = ()  # crop-local (x, y, width, height)
  gains: tuple = ()


@dataclass(frozen=True)
class CopiedNV12:
  data: bytes
  stride: int
  uv_offset: int


def copy_nv12_roi(frame, roi):
  """Detach a chroma-aligned crop before a worker touches shared camera memory."""
  x,y,w,h = roi
  x0,y0 = x//2*2,y//2*2
  x1,y1 = (x+w+1)//2*2,(y+h+1)//2*2
  if not (0 <= x0 < x1 <= frame.width and 0 <= y0 < y1 <= frame.height):
    raise ValueError('NV12 crop outside frame')
  width,height = x1-x0,y1-y0
  raw = np.frombuffer(frame.data,dtype=np.uint8)
  luma = np.ndarray((height,width),np.uint8,buffer=raw,offset=y0*frame.stride+x0,strides=(frame.stride,1))
  chroma = np.ndarray((height//2,width),np.uint8,buffer=raw,
                      offset=frame.uv_offset+(y0//2)*frame.stride+x0,strides=(frame.stride,1))
  copied = CopiedNV12(luma.tobytes()+chroma.tobytes(),width,width*height)
  return copied,(x-x0,y-y0,w,h)


def project_search(matrix, distance, lateral, ground_z, shape):
  """Project a 2.2 m x 1.9 m lead search plane, using the UI's calibrated convention."""
  if not all(math.isfinite(v) for v in (distance, lateral, ground_z)) or not 3 < distance < 65:
    return None
  points = np.array([[distance, -lateral + dy, ground_z + dz] for dy, dz in
                     ((-1.1, -1.9), (1.1, -1.9), (-1.1, .05), (1.1, .05))])
  q = np.asarray(matrix) @ points.T
  if not np.isfinite(q).all() or (q[2] <= .1).any():
    return None
  xy = q[:2] / q[2]
  x0, y0 = np.floor(xy.min(axis=1)).astype(int)
  x1, y1 = np.ceil(xy.max(axis=1)).astype(int)
  h, w = shape[:2]
  # The lead search plane includes road below the lamps. At short distances that
  # ground plane can cross the bottom edge while the lamps remain fully visible.
  # Permit limited bottom clipping; never shift a clipped side onto another car.
  if x0 < 0 or y0 < 0 or x1 >= w or y0 >= h or x1-x0 < 42 or y1-y0 < 32:
    return None
  if y1 > h and (h-y0)/(y1-y0) < .75:
    return None
  y1 = min(y1, h)
  return (int(x0), int(y0), int(x1-x0), int(y1-y0))


def nv12_crop(frame, roi, max_size=192):
  """Copy only a bounded NV12 ROI. Never pass shared VisionIPC memory to a worker."""
  x, y, w, h = roi
  step = max(1, math.ceil(max(w, h) / max_size))
  xx = np.arange(x, x+w, step)
  yy = np.arange(y, y+h, step)
  raw = np.frombuffer(frame.data, dtype=np.uint8)
  yv = raw[yy[:, None]*frame.stride + xx].astype(np.float32)
  uv = frame.uv_offset + (yy[:, None]//2)*frame.stride + (xx//2)*2
  u = raw[uv].astype(np.float32)-128
  v = raw[uv+1].astype(np.float32)-128
  # Match the existing UI NV12 shader's full-range conversion.
  rgb = np.stack((yv+1.402*v, yv-.344*u-.714*v, yv+1.772*u), axis=2)
  return np.clip(rgb, 0, 255).astype(np.uint8), step


def _dilate(mask):
  padded = np.pad(mask, 1)
  return np.logical_or.reduce([padded[y:y+mask.shape[0], x:x+mask.shape[1]] for y in range(3) for x in range(3)])


def red_components(rgb):
  r, g, b = rgb.astype(np.float32).transpose(2, 0, 1)
  red = (r >= 90) & (r > g*1.6) & (r > b*1.4) & (r-g > 45)
  # A white/yellow saturated centre belongs to a lamp only near red evidence.
  white = (r >= 160) & (g >= 100) & (b >= 70) & (r >= g*.9) & (g >= b*.85)
  mask = red | (white & _dilate(_dilate(red)))
  # Close one-pixel LED gaps without accepting an entire red backdrop.
  padded = np.pad(_dilate(mask), 1, constant_values=True)
  mask = np.logical_and.reduce([padded[y:y+mask.shape[0], x:x+mask.shape[1]] for y in range(3) for x in range(3)])
  h, w = mask.shape
  if mask.mean() > .65:
    return []
  # Run-length connected components: bounded ROI, no Python per-pixel flood fill.
  parents, bounds, cores, counts, previous = [], [], [], [], []
  def find(i):
    while parents[i] != i:
      parents[i] = parents[parents[i]]
      i = parents[i]
    return i
  for y, row in enumerate(mask):
    transitions = np.flatnonzero(np.diff(np.pad(row.astype(np.int8), 1)))
    current = []
    cursor = 0
    for start, stop in zip(transitions[::2], transitions[1::2]):
      start, stop = int(start), int(stop)-1
      i = len(parents)
      if i >= 4096:
        return []
      parents.append(i); bounds.append([start,y,stop,y]); counts.append(stop-start+1)
      bright = np.flatnonzero(r[y,start:stop+1] >= 170)
      cores.append([start+int(bright[0]),y,start+int(bright[-1]),y] if len(bright) >= 1 else None)
      while cursor < len(previous) and previous[cursor][1] < start-1:
        cursor += 1
      j = cursor
      while j < len(previous) and previous[j][0] <= stop+1:
        root = find(previous[j][2]); own = find(i)
        if root != own:
          parents[root] = own
          bounds[own] = [min(bounds[own][0],bounds[root][0]),min(bounds[own][1],bounds[root][1]),
                         max(bounds[own][2],bounds[root][2]),max(bounds[own][3],bounds[root][3])]
          counts[own] += counts[root]
          if cores[root] is not None:
            if cores[own] is None:
              cores[own] = cores[root][:]
            else:
              cores[own] = [min(cores[own][0],cores[root][0]),min(cores[own][1],cores[root][1]),
                            max(cores[own][2],cores[root][2]),max(cores[own][3],cores[root][3])]
        j += 1
      current.append((start,stop,i))
    previous = current
  out = []
  for i in range(len(parents)):
    if parents[i] == i and 2 <= counts[i] <= h*w*.12:
      core = cores[i]
      # A few saturated pixels must not collapse a long LED strip to a dot.
      use_core = core is not None and core[2]-core[0]+1 >= .5*(bounds[i][2]-bounds[i][0]+1)
      x,y,x1,y1 = core if use_core else bounds[i]
      out.append((x,y,x1-x+1,y1-y+1))
  return sorted(out,key=lambda a:a[2]*a[3],reverse=True)[:24] if len(out) <= 128 else []


def connected_lamp_ends(box, shape):
  """Split only a thin, centred rear light bar; never a broad red surface."""
  h, w = shape[:2]
  x, y, bw, bh = box
  if (.55*w < bw < .9*w and 1 <= bh < .10*h and bw > 6*bh and
      abs(x+bw/2-w/2) < .12*w and .20*h < y+bh/2 < .80*h):
    end = max(3, int(bw*.20))
    return ((x,y,end,bh), (x+bw-end,y,end,bh))
  return ()


def locate_lamps(rgb, previous_geometry=None, components=None):
  """Red-region pair heuristic within lead search ROI; ambiguous pairs are rejected."""
  h, w = rgb.shape[:2]
  components = red_components(rgb) if components is None else components
  # Keep the original components for centre-lamp observations; virtual ends
  # only supply left/right geometry for a connected horizontal LED bar.
  bars = [(box, connected_lamp_ends(box, rgb.shape)) for box in components]
  components = [box for box, ends in bars if not ends]+[end for _, ends in bars for end in ends]
  center = lambda a: (a[0]+a[2]/2, a[1]+a[3]/2)
  pairs = []
  for a in components:
    ax, ay = center(a)
    for b in components:
      bx, by = center(b)
      if not (.40*w < bx-ax < .94*w and abs(ay-by) < .12*h and
              abs((ax+bx)/2-w/2) < .16*w and .15*h < (ay+by)/2 < .86*h):
        continue
      if min(a[2]*a[3],b[2]*b[3]) < max(3,h*w*.0004) or max(a[2]*a[3], b[2]*b[3]) > 7*min(a[2]*a[3], b[2]*b[3]):
        continue
      cost = (abs(ay-by)/h + abs((ax+bx)/2-w/2)/w + .25*(ay+by)/2/h
              - .04*math.log1p(min(a[2]*a[3], b[2]*b[3])))
      separation = bx-ax
      upper = [c for c in components if c not in (a,b) and max(4,.12*min(a[2]*a[3],b[2]*b[3])) <= c[2]*c[3] <= 4*max(a[2]*a[3],b[2]*b[3]) and
               abs(center(c)[0]-(ax+bx)/2) < separation*.17 and
               .06*h < min(ay,by)-center(c)[1] < min(.45*h,.55*separation)]
      if upper:
        cost -= .22
      if previous_geometry is not None:
        current = np.array([[ax/w, ay/h], [bx/w, by/h]])
        cost += 1.5*float(np.max(np.abs(current-previous_geometry[:2])))
      pairs.append((cost, a, b))
  if not pairs:
    return ()
  pairs.sort(key=lambda item: item[0])
  if len(pairs) > 1 and pairs[1][0]-pairs[0][0] < .04:
    return ()
  _, left, right = pairs[0]
  lx, ly = center(left); rx, ry = center(right)
  middle = [a for a in components if a not in (left, right) and a[2] >= 3 and
            max(3, .12*min(left[2]*left[3], right[2]*right[3])) <= a[2]*a[3] <= 4*max(left[2]*left[3],right[2]*right[3]) and
            abs(center(a)[0]-(lx+rx)/2) < (rx-lx)*.17 and
            .06*h < min(ly, ry)-center(a)[1] < min(.45*h,.55*(rx-lx))]
  if previous_geometry is not None and len(previous_geometry) == 3:
    middle.sort(key=lambda a: np.linalg.norm(np.array(center(a))/[w, h]-previous_geometry[2]))
  else:
    middle.sort(key=lambda a: abs(center(a)[0]-(lx+rx)/2))
  return (left, right, middle[0]) if middle else (left, right)


def lamp_scores(rgb, lamps):
  left, right = lamps[:2]
  h, w = rgb.shape[:2]
  x0 = left[0]+left[2]; x1 = right[0]
  y0 = int(min(left[1], right[1])+.10*h)
  body = rgb[y0:min(h, y0+max(5, int(.25*h))), x0:x1, 0]
  if body.size < 20 or np.median(body) < 25:
    return None
  ref = float(np.median(body))
  return lamp_intensities(rgb, lamps)/ref


def lamp_intensities(rgb, lamps):
  h, w = rgb.shape[:2]
  values = []
  for x, y, bw, bh in lamps:
    red = rgb[max(0, y-1):min(h, y+bh+1), max(0, x-1):min(w, x+bw+1), 0].ravel()
    values.append(float(np.sort(red)[-max(2, len(red)//5):].mean()))
  return np.array(values)


class VehicleOutline:
  """Lamp-seeded image-edge refinement inside the projected lead search region.

  This is a local contour estimate, not a trained object detector. Unresolved
  edges produce no vehicle rectangle instead of promoting the search ROI.
  """
  def __init__(self):
    self.geometry = None
    self.time = None
    self.confirm = 0

  def update(self, rgb, lamps, now):
    if self.time is None or not 0 < now-self.time <= .35:
      self.geometry = None
      self.confirm = 0
    self.time = now
    if len(lamps) < 2:
      self.confirm = 0
      return None
    h,w = rgb.shape[:2]
    left,right = lamps[:2]
    lx,rx = left[0]+left[2]/2,right[0]+right[2]/2
    ly,ry = left[1]+left[3]/2,right[1]+right[3]/2
    if not (.4*w < rx-lx < .94*w and abs(ly-ry) < .12*h):
      self.confirm = 0
      return None
    gray = rgb.astype(np.float32).mean(axis=2)
    gx = np.abs(np.diff(gray,axis=1))
    gy = np.abs(np.diff(gray,axis=0))
    rows = slice(max(1,int(.1*h)),min(h,int(max(ly,ry)+.20*h)))
    def peak(values, low, high):
      low,high = max(1,int(low)),min(len(values)-1,int(high))
      if high-low < 3:
        return None
      segment = values[low:high]
      index = int(np.argmax(segment))+low
      # A coherent edge must stand out from its local texture/noise.
      return index if values[index] > max(3.,1.35*float(np.median(segment))) else None
    columns = gx[rows].mean(axis=0)
    x0 = peak(columns,lx-.18*w,left[0]+1)
    x1 = peak(columns,right[0]+right[2],rx+.18*w)
    if x0 is None or x1 is None or not .5*w < x1-x0 < .98*w:
      self.confirm = 0
      return None
    horizontal = gy[:,x0+int(.1*(x1-x0)):x1-int(.1*(x1-x0))].mean(axis=1)
    y0 = peak(horizontal,.02*h,min(ly,ry)-.14*h)
    y1 = peak(horizontal,max(max(ly,ry)+.08*h,.8*h),.98*h)
    if y0 is None or y1 is None or not .4*h < y1-y0 < .98*h:
      self.confirm = 0
      return None
    geometry = np.array([x0/w,y0/h,x1/w,y1/h])
    if self.geometry is None or np.max(np.abs(geometry-self.geometry)) > .16:
      self.geometry = geometry
      self.confirm = 1
      return None
    self.geometry += .35*(geometry-self.geometry)
    self.confirm += 1
    if self.confirm < 3:
      return None
    x0,y0,x1,y1 = self.geometry*[w,h,w,h]
    x0 = min(x0,float(left[0]))
    x1 = max(x1,float(right[0]+right[2]))
    return (float(x0),float(y0),float(x1-x0),float(y1-y0))



class CenterLampTransitions:
  """Causal upper-centre lamp onset within the current lead's search region.

  The owner resets this observer on lead/camera changes. A red component alone
  cannot confirm ON: require lower lamp evidence, an exposure-compensated rise
  and three matching observations. Absence never establishes brake OFF.
  """
  # Conservative thresholds checked against the test9-test35 front-camera scan.
  # The scan supports a strong per-target red/chroma transition, but its
  # candidates are not independent labels, so keep the onset thresholds strict.
  CHROMA_RISE = 45.
  RAW_RISE = 20.
  MIN_RATIO = .6
  CONFIRM_FRAMES = 3
  MAX_CANDIDATE_MISSES = 1
  REFERENCE_GRACE = .4

  def __init__(self):
    self.history = deque(maxlen=3)  # Three bounded ROI copies, never full frames.
    self.geometry = None
    self.reference = None
    self.count = 0
    self.candidate_misses = 0
    self.last_seen = None
    self.last_time = None
    self.release_until = -math.inf

  def clear_candidate(self):
    self.geometry = self.reference = None
    self.count = 0
    self.candidate_misses = 0
    self.last_seen = None

  @staticmethod
  def stats(rgb, normalized, background):
    h, w = rgb.shape[:2]
    x, y, bw, bh = normalized
    x0, y0 = max(0, int(x*w)), max(0, int(y*h))
    x1 = min(w, max(x0+1, int((x+bw)*w)))
    y1 = min(h, max(y0+1, int((y+bh)*h)))
    # Include one neighbouring pixel so a thin LED moving across the sampling
    # grid cannot manufacture an OFF reference or a release.
    patch = rgb[max(0,y0-1):min(h,y1+1), max(0,x0-1):min(w,x1+1)].astype(float)
    if not patch.size:
      return None
    excess = patch[:, :, 0]-np.maximum(patch[:, :, 1], patch[:, :, 2])
    raw = float(np.percentile(patch[:, :, 0], 80))
    chroma = float(np.percentile(excess, 80))
    return np.array([raw, chroma, max(25., background)])

  def update(self, rgb, now, components):
    if self.last_time is not None and (now <= self.last_time or now-self.last_time > .4):
      self.__init__()
    self.last_time = now
    if self.last_seen is not None and now-self.last_seen > self.REFERENCE_GRACE:
      self.clear_candidate()
    h, w = rgb.shape[:2]
    background = float(np.median(rgb[:, :, 0]))
    # A known centre lamp returning to its unlit reference vetoes a side-only
    # brightness latch. Do not invent OFF from an occlusion or moved crop.
    if self.count >= self.CONFIRM_FRAMES and self.reference is not None:
      current = self.stats(rgb, self.geometry, background)
      exposure = current[2]/self.reference[2]
      delta = current-self.reference*exposure
      matched = any(abs((b[0]+b[2]/2)/w-(self.geometry[0]+self.geometry[2]/2)) < .08 and
                    abs((b[1]+b[3]/2)/h-(self.geometry[1]+self.geometry[3]/2)) < .06 and
                    .04*w <= b[2] < .35*w and b[3] < .13*h for b in components)
      if not matched and .5 <= exposure <= 2. and delta[0] < self.RAW_RISE*.25 and delta[1] < self.CHROMA_RISE*.25:
        self.release_until = max(self.release_until, self.last_seen+.8)
    candidates = []
    for box in components:
      x, y, bw, bh = box
      cx, cy = x+bw/2, y+bh/2
      if not (.30*w < cx < .70*w and .01*h < cy < .50*h and
              3 <= bw < .35*w and bh < .13*h and bw >= 1.2*bh):
        continue
      # Current lower side evidence ties this upper strip to the lead's rear.
      lower = [b for b in components if b[2]*b[3] >= 3 and .05*h < b[1]+b[3]/2-cy < .55*h]
      left = any(b[0]+b[2]/2 < cx-.12*w for b in lower)
      right = any(b[0]+b[2]/2 > cx+.12*w for b in lower)
      connected = any(connected_lamp_ends(b, rgb.shape) and b[0] < cx-.12*w and b[0]+b[2] > cx+.12*w for b in lower)
      left, right = left or connected, right or connected
      norm = np.array([x/w, y/h, bw/w, bh/h])
      matched = self.geometry is not None and np.max(np.abs(norm[:2]-self.geometry[:2])) < .08
      # A different component cannot replace a still-confirmed lamp during
      # the grace period; reacquire only after the old reference expires.
      if self.count >= self.CONFIRM_FRAMES and not matched:
        continue
      confirmed = matched and self.count >= self.CONFIRM_FRAMES
      # Entry and maintenance use different floors. Downsampling a confirmed
      # thin LED strip must not erase its OFF reference for one dimmer frame.
      if bw < max(3, (.04 if confirmed else .07)*w):
        continue
      if not (left and right) and not confirmed:
        continue
      current = self.stats(rgb, norm, background)
      if current is None or current[0] < (110 if confirmed else 130) or current[1] < (35 if confirmed else 45):
        continue
      reference = self.reference if matched else None
      if reference is None:
        if len(self.history) < 3:
          continue
        reference = np.median([self.stats(p, norm, bg) for p, bg in self.history], axis=0)
      exposure = current[2]/reference[2]
      if not .5 <= exposure <= 2.:
        continue  # A major exposure jump cannot establish a lamp onset.
      delta = current-reference*exposure
      ratio = delta[1]/max(25., reference[1]*exposure)
      threshold = .5 if confirmed else 1.
      if (delta[1] >= self.CHROMA_RISE*threshold and delta[0] >= self.RAW_RISE*threshold and
          ratio >= self.MIN_RATIO*threshold):
        candidates.append((float(delta[1]), box, norm, reference, matched))
    result = None
    candidates.sort(key=lambda candidate: candidate[0], reverse=True)
    unambiguous = bool(candidates) and (len(candidates) == 1 or candidates[0][0] > 1.4*candidates[1][0])
    if unambiguous:
      _, box, norm, reference, matched = candidates[0]
      self.count = min(self.CONFIRM_FRAMES, self.count + 1) if matched else 1
      self.candidate_misses = 0
      self.geometry, self.reference = norm, reference
      self.last_seen = now
      if self.count >= self.CONFIRM_FRAMES:
        result = box
    elif self.count < self.CONFIRM_FRAMES and self.count > 0 and self.candidate_misses < self.MAX_CANDIDATE_MISSES:
      # A single dim/occluded sample inside the 5 Hz camera stream should not
      # discard an otherwise consistent onset. The box is still hidden on this
      # frame; this only preserves the causal confirmation streak.
      self.candidate_misses += 1
    elif self.count < self.CONFIRM_FRAMES:
      self.clear_candidate()
    # With missing/ambiguous evidence, never return an old box or ON. An already
    # confirmed reference alone survives until REFERENCE_GRACE expires above.
    self.history.append((rgb.copy(), background))
    return result


class AdaptiveLamps:
  def __init__(self):
    self.reset()

  def reset(self):
    self.center = CenterLampTransitions()
    self.release_veto = False
    self._reset_relative()

  def _reset_relative(self):
    self.history = deque(maxlen=40)
    self.raw_history = deque(maxlen=40)
    self.prev_geometry = None
    self.last_time = None
    self.last_valid_time = None
    self.confirmed_on = False
    self.count = 0
    self.off_count = 0
    self.baseline = None
    self.reference_confirmed = False

  def update(self, rgb, now):
    if self.center.last_time is not None and not 0 < now-self.center.last_time <= .4:
      self.release_veto = False
    # Reuse the bounded component extraction for both observers.
    components = red_components(rgb)
    relative = self._update_relative(rgb, now, components)
    middle = self.center.update(rgb, now, components)
    if middle is not None:
      self.release_veto = False
      lamps = relative.lamps[:2]+(middle,) if len(relative.lamps) >= 2 else (middle,)
      return Observation("ON", "CENTER_LAMP_RISE", lamps, relative.gains)
    if now <= self.center.release_until:
      self.release_veto = True
    if relative.state == "OFF":
      self.release_veto = False
    if self.release_veto:
      # Let the existing side observer finish its coordinated fall/reference
      # confirmation, but never display its old ON over a centre-lamp release.
      return Observation("UNKNOWN", "CENTER_LAMP_RELEASE", relative.lamps, relative.gains)
    return relative

  def _update_relative(self, rgb, now, components):
    if self.last_time is not None and (now <= self.last_time or now-self.last_time > .6):
      self._reset_relative()
    self.last_time = now
    if self.last_valid_time is not None and now-self.last_valid_time > .4:
      self._reset_relative()
      self.last_time = now
    lamps = locate_lamps(rgb, self.prev_geometry, components)
    if not lamps:
      # Hide ON immediately, but preserve the same target's reference over a brief
      # missed observation. No old lamp locations are returned or drawn.
      self.confirmed_on = False
      self.count = self.off_count = 0
      return Observation(reason="NO_LAMP_PAIR")
    if any(tuple(lamps[:2]) == connected_lamp_ends(box, rgb.shape) for box in components):
      # A connected bar is also a normal tail light. Its endpoints are useful
      # geometry, but must not create a side-brightness ON/OFF reference.
      self._reset_relative()
      return Observation(reason="CONNECTED_LAMP_REFERENCE", lamps=lamps)
    h, w = rgb.shape[:2]
    geometry = np.array([((x+bw/2)/w, (y+bh/2)/h) for x, y, bw, bh in lamps])
    if self.prev_geometry is not None and np.max(np.abs(geometry[:2]-self.prev_geometry[:2])) > .12:
      self._reset_relative()
    self.prev_geometry = geometry
    self.last_time = self.last_valid_time = now
    scores = lamp_scores(rgb, lamps)
    if scores is None:
      self._reset_relative()
      return Observation(reason="LOW_VISIBILITY", lamps=lamps)
    # Compare side lamps even if center lamp is not visible at all times.
    sides = np.array([scores[0], scores[1], scores[2] if len(scores) > 2 else 0.])
    raw = lamp_intensities(rgb, lamps)
    raw = np.array([raw[0], raw[1], raw[2] if len(raw) > 2 else 0.])
    if len(self.history) < 4:
      self.history.append(sides)
      self.raw_history.append(raw)
      self.baseline = np.percentile(np.array(self.history), 15, axis=0)
      return Observation(reason="LEARNING", lamps=lamps)
    gains = (sides-self.baseline)/np.maximum(.15, self.baseline)
    rise = bool(np.min(gains[:2]) > .45 or
                (gains[2] > .5 and np.min(gains[:2]) > .08 and np.mean(gains[:2]) > .18))
    # A rolling reference must not turn a slow change in size/illumination into
    # a new braking event. Require an onset relative to recent matched frames.
    recent = np.median(np.array(self.history)[-3:], axis=0)
    edge = (sides-recent)/np.maximum(.15, recent)
    onset = bool(np.min(edge[:2]) > .12 or
                 (edge[2] > .30 and np.min(edge[:2]) > .03))
    raw_recent = np.median(np.array(self.raw_history)[-3:], axis=0)
    raw_edge = (raw-raw_recent)/np.maximum(10., raw_recent)
    # A darker body/background alone must not turn unchanged lamps into ON.
    emitted_rise = bool(np.min(raw_edge[:2]) > .06 or
                        (raw_edge[2] > .12 and np.min(raw_edge[:2]) > -.02))
    rise = rise and onset and emitted_rise
    self.count = self.count+1 if rise else 0
    if self.count >= 3:
      self.confirmed_on = True
    if self.confirmed_on:
      dropped = bool(np.max((sides-self.baseline)/np.maximum(.15, self.baseline)) < .20)
      self.off_count = self.off_count+1 if dropped else 0
      if self.off_count >= 3:
        self.confirmed_on = False
        self.reference_confirmed = True
        self.history.clear()
        self.raw_history.clear()
      else:
        still_lit = bool(np.min(gains[:2]) > .22 or (gains[2] > .30 and np.min(gains[:2]) > .03))
        return Observation("ON" if still_lit else "UNKNOWN",
                           "BRIGHTNESS_RISE" if still_lit else "RELEASE_PENDING", lamps,
                           tuple(float(g) for g in gains))
    # A coordinated fall from an initially illuminated target can confirm a lower reference.
    high = np.max(self.history, axis=0) if self.history else sides
    if np.max(sides[:2]/np.maximum(high[:2], .15)) < .72:
      self.off_count += 1
      if self.off_count >= 3:
        self.reference_confirmed = True
    elif not self.reference_confirmed:
      self.off_count = 0
    # Update only outside confirmed ON and rising candidates; no future samples are used.
    if not rise:
      self.history.append(sides)
      self.raw_history.append(raw)
      low = np.percentile(np.array(self.history), 15, axis=0)
      self.baseline = low
    state = "OFF" if self.reference_confirmed and np.max(np.abs(gains)) < .25 else "UNKNOWN"
    return Observation(state, "REFERENCE_OK" if state == "OFF" else "LEARNING", lamps, tuple(float(g) for g in gains))
