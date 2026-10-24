"""Experimental leadOne-gated lamp observation. Never consumed by vehicle control.

No trained vehicle/lamp detector: projection is a search region, not a vehicle box.
A causal per-target baseline needs an observed brightness transition; startup is UNKNOWN.
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


def red_components(rgb):
  r, g, b = rgb.astype(np.float32).transpose(2, 0, 1)
  mask = (r > 45) & (r > g*1.65) & (r > b*1.45)
  h, w = mask.shape
  if mask.sum() > h*w*.22:
    return []  # red background/glare: do not invent a target
  seen = np.zeros_like(mask)
  out = []
  for y, x in zip(*np.nonzero(mask)):
    if seen[y, x]:
      continue
    stack = [(int(x), int(y))]
    seen[y, x] = True
    xs, ys = [], []
    while stack:
      a, c = stack.pop()
      xs.append(a); ys.append(c)
      for u, v in ((a-1, c), (a+1, c), (a, c-1), (a, c+1)):
        if 0 <= u < w and 0 <= v < h and mask[v, u] and not seen[v, u]:
          seen[v, u] = True
          stack.append((u, v))
    if 2 <= len(xs) <= h*w*.09:
      out.append((min(xs), min(ys), max(xs)-min(xs)+1, max(ys)-min(ys)+1))
  # Bound pair matching even in a highly fragmented red/noisy background.
  return sorted(out, key=lambda a: a[2]*a[3], reverse=True)[:24] if len(out) <= 48 else []


def locate_lamps(rgb, previous_geometry=None):
  """Red-region pair heuristic within lead search ROI; ambiguous pairs are rejected."""
  h, w = rgb.shape[:2]
  components = red_components(rgb)
  center = lambda a: (a[0]+a[2]/2, a[1]+a[3]/2)
  pairs = []
  for a in components:
    ax, ay = center(a)
    for b in components:
      bx, by = center(b)
      if not (.40*w < bx-ax < .94*w and abs(ay-by) < .12*h and
              abs((ax+bx)/2-w/2) < .16*w and .15*h < (ay+by)/2 < .86*h):
        continue
      if max(a[2]*a[3], b[2]*b[3]) > 7*min(a[2]*a[3], b[2]*b[3]):
        continue
      cost = (abs(ay-by)/h + abs((ax+bx)/2-w/2)/w + .12*(ay+by)/2/h
              - .04*math.log1p(min(a[2]*a[3], b[2]*b[3])))
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
            a[2]*a[3] >= max(3, .12*min(left[2]*left[3], right[2]*right[3])) and
            abs(center(a)[0]-(lx+rx)/2) < (rx-lx)*.17 and
            .06*h < min(ly, ry)-center(a)[1] < .55*h]
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



class AdaptiveLamps:
  def __init__(self):
    self.reset()

  def reset(self):
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
    if self.last_time is not None and (now <= self.last_time or now-self.last_time > .6):
      self.reset()
    self.last_time = now
    if self.last_valid_time is not None and now-self.last_valid_time > .4:
      self.reset()
      self.last_time = now
    lamps = locate_lamps(rgb, self.prev_geometry)
    if not lamps:
      # Hide ON immediately, but preserve the same target's reference over a brief
      # missed observation. No old lamp locations are returned or drawn.
      self.confirmed_on = False
      self.count = self.off_count = 0
      return Observation(reason="NO_LAMP_PAIR")
    h, w = rgb.shape[:2]
    geometry = np.array([((x+bw/2)/w, (y+bh/2)/h) for x, y, bw, bh in lamps])
    if self.prev_geometry is not None and np.max(np.abs(geometry[:2]-self.prev_geometry[:2])) > .12:
      self.reset()
    self.prev_geometry = geometry
    self.last_time = self.last_valid_time = now
    scores = lamp_scores(rgb, lamps)
    if scores is None:
      self.reset()
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
