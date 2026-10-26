"""Optional whole-vehicle detector; no control/planning outputs.
ON/OFF is a bounded display hint; fast lamp onset/release evidence takes priority.
"""
import importlib.util
import multiprocessing as mp
import os
from pathlib import Path
from queue import Empty, Full
import time
from threading import Thread
import numpy as np
from openpilot.selfdrive.ui.onroad.brake_lights import Observation, copy_nv12_roi

MODEL = Path(__file__).resolve().parent / 'models' / 'brake_light_v4.onnx'


def intersection(a, b):
  return max(0., min(a[0]+a[2], b[0]+b[2])-max(a[0], b[0])) * max(0., min(a[1]+a[3], b[1]+b[3])-max(a[1], b[1]))


def decode_predictions(output, width, height, confidence=.65):
  raw = np.asarray(output)
  if raw.shape != (1, 6, 8400):
    raise ValueError('Unexpected brake model output')
  rows = raw[0].T
  ids = np.argmax(rows[:, 4:], axis=1)
  scores = rows[np.arange(len(rows)), 4+ids]
  keep = np.flatnonzero(np.isfinite(rows).all(axis=1) & (scores >= confidence))
  candidates = []
  for i in keep[np.argsort(scores[keep])[::-1]][:100]:
    cx, cy, w, h = rows[i, :4]
    if w <= 0 or h <= 0:
      continue
    x1, y1 = max(0., float(cx-w/2)*width/640), max(0., float(cy-h/2)*height/640)
    x2, y2 = min(float(width), float(cx+w/2)*width/640), min(float(height), float(cy+h/2)*height/640)
    box = (x1, y1, x2-x1, y2-y1)
    if box[2] < 10 or box[3] < 10:
      continue
    if any(intersection(box, c['box']) / max(1., box[2]*box[3]+c['box'][2]*c['box'][3]-intersection(box, c['box'])) > .45 for c in candidates):
      continue
    candidates.append(dict(box=box, confidence=float(scores[i]), state='ON' if ids[i] == 1 else 'OFF'))
    if len(candidates) == 20:
      break
  return candidates


def select_lead_box(candidates, roi):
  x, y, w, h = roi
  matches = []
  for c in candidates:
    a, b, bw, bh = c['box']
    if (c['confidence'] < .65 or not np.isfinite((a,b,bw,bh)).all() or bw <= 0 or bh <= 0 or
        not .5 < bw/w < 1.6 or not .4 < bh/h < 1.6 or not .45 < bw/bh < 3.5 or
        abs(a+bw/2-x-w/2) > w*.3 or abs(b+bh/2-y-h/2) > h*.35):
      continue
    overlap = intersection(c['box'], roi)
    iou = overlap / max(1., w*h+bw*bh-overlap)
    if iou < .25 or overlap/(bw*bh) < .6:
      continue
    matches.append((iou*c['confidence'], c))
  matches.sort(key=lambda item: item[0], reverse=True)
  if not matches or (len(matches) > 1 and matches[0][0] < matches[1][0]*1.2):
    return None
  return matches[0][1]


def fuse_observation(observation, hint):
  observation = observation or Observation()
  if hint is None:
    return observation
  # A fresh centre/brightness rise remains authoritative over slow model OFF.
  if observation.state == 'ON':
    return observation
  if observation.reason == 'RELEASE_PENDING':
    return observation
  if observation.state == 'OFF' and hint['state'] == 'ON':
    return Observation('UNKNOWN', 'MODEL_LAMP_CONFLICT', observation.lamps, observation.gains)
  return Observation(hint['state'], 'MODEL_'+hint['state'], observation.lamps, observation.gains)


def nv12_input(frame, width, height):
  # Stretch full-range NV12 to the model's 640x640 RGB input.
  xx = np.clip((np.arange(640)+.5)*width/640-.5, 0, width-1).astype(int)
  yy = np.clip((np.arange(640)+.5)*height/640-.5, 0, height-1).astype(int)
  raw = np.frombuffer(frame.data, np.uint8)
  luma = raw[yy[:, None]*frame.stride+xx].astype(np.float32)
  uv = frame.uv_offset+(yy[:, None]//2)*frame.stride+(xx//2)*2
  u, v = raw[uv].astype(np.float32)-128, raw[uv+1].astype(np.float32)-128
  rgb = np.stack((luma+1.402*v, luma-.344*u-.714*v, luma+1.772*u), axis=0)
  return (np.clip(rgb, 0, 255)/255)[None].astype(np.float32)


def _background_priority():
  # UI runs SCHED_FIFO on C4. Reset the launching thread BEFORE spawning, so
  # child imports and Queue feeder threads never inherit UI realtime priority.
  if hasattr(os, 'sched_setscheduler'):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  if hasattr(os, 'sched_setaffinity'):
    try:
      os.sched_setaffinity(0, {0})
    except OSError:
      pass
  if hasattr(os, 'nice'):
    os.nice(10)


def _worker(inbox, outbox, path):
  try:
    _background_priority()
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.log_severity_level = 3
    session = ort.InferenceSession(path, options, providers=['CPUExecutionProvider'])
    inp = session.get_inputs()[0]
    if inp.shape != [1,3,640,640] or session.get_outputs()[0].shape != [1,6,8400]:
      raise ValueError('Unexpected brake model interface')
    outbox.put(('ready',))
    while True:
      job = inbox.get()
      if job is None:
        return
      if job == ('ping',):
        continue
      epoch, captured, roi, width, height, copied = job
      start = time.monotonic()
      output = session.run(None, {inp.name: nv12_input(copied, width, height)})[0]
      chosen = select_lead_box(decode_predictions(output, width, height), roi)
      outbox.put(('result', epoch, captured, roi, chosen, time.monotonic()-start))
  except Exception as exc:
    try:
      outbox.put(('error', type(exc).__name__))
    except (OSError, ValueError):
      pass


class BrakeVehicleModel:
  def __init__(self):
    self.enabled = (os.environ.get('LEAD_BRAKE_MODEL', '1') != '0' and MODEL.is_file() and
                    importlib.util.find_spec('onnxruntime') is not None)
    self.process = None
    self.launching = False
    self.launch_error = False
    self.inbox = self.outbox = None
    self.ready = self.busy = False
    self.started = self.submitted = 0.
    self.last_submit = -100.
    self.interval = 1.
    self.result = None
    self.status = 'idle' if self.enabled else 'unavailable'
    self.report = None
    self.low_fps_since = None

  def clear(self):
    self.result = None

  def close(self):
    self.launching = False
    if self.process is not None:
      if self.process.is_alive():
        self.process.terminate()
      self.process.join(timeout=.01)
      self.process = None
    for q in (self.inbox, self.outbox):
      if q is not None:
        q.cancel_join_thread()
        q.close()
    self.inbox = self.outbox = None
    self.ready = self.busy = False

  def disable(self, reason):
    self.enabled = False
    self.status = reason
    self.result = None
    self.close()

  def _launch(self):
    try:
      _background_priority()
      ctx = mp.get_context('spawn')
      inbox, outbox = ctx.Queue(maxsize=1), ctx.Queue(maxsize=2)
      # Start the serialization feeder from this non-realtime thread.
      inbox.put_nowait(('ping',))
      process = ctx.Process(target=_worker, args=(inbox,outbox,str(MODEL)), daemon=True)
      process.start()
      if not self.enabled or not self.launching:
        process.terminate()
        process.join(timeout=.01)
        for q in (inbox,outbox):
          q.cancel_join_thread()
          q.close()
        return
      self.inbox, self.outbox, self.process = inbox, outbox, process
      self.launching = False
    except Exception:
      self.launch_error = True
      self.launching = False

  def tick(self, frame, roi, epoch, now, fps=None):
    if not self.enabled:
      return None
    if isinstance(fps, (int, float)) and 0 < fps < 30:
      if self.low_fps_since is None:
        self.low_fps_since = now
      elif now-self.low_fps_since >= 3.:
        self.disable('ui_fps_low')
        return None
    else:
      self.low_fps_since = None
    try:
      if self.launch_error:
        self.disable('worker_setup_error')
        return None
      if self.process is None:
        if not self.launching:
          self.launching = True
          self.started = now
          self.status = 'loading'
          Thread(target=self._launch, name='brake_model_launch', daemon=True).start()
        elif now-self.started > 20.:
          self.disable('worker_start_timeout')
        return None
      for _ in range(2):
        try:
          message = self.outbox.get_nowait()
        except Empty:
          break
        if message[0] == 'ready':
          self.ready = True
          self.status = 'ready'
        elif message[0] == 'error':
          self.disable('load_or_run_error:'+message[1])
          return None
        elif message[0] == 'result':
          _, generation, captured, found_roi, chosen, elapsed = message
          self.busy = False
          self.interval = max(1., elapsed*5)  # At most ~20% of one CPU core.
          self.report = dict(duration_ms=round(elapsed*1000,1), state=chosen['state'] if chosen else 'UNKNOWN',
                             confidence=chosen['confidence'] if chosen else 0., matched=chosen is not None)
          if elapsed > 1.:
            self.disable('inference_too_slow')
            return None
          if generation == epoch and 0 <= now-captured < 1.:
            self.result = (generation, captured, found_roi, chosen)
      if (not self.process.is_alive() or (not self.ready and now-self.started > 20.) or
          (self.busy and now-self.submitted > 2.)):
        self.disable('worker_stalled_or_exited')
        return None
      if self.ready and not self.busy and now-self.last_submit >= self.interval:
        copied, _ = copy_nv12_roi(frame, (0,0,frame.width,frame.height))
        try:
          self.inbox.put_nowait((epoch,now,roi,frame.width,frame.height,copied))
          self.busy = True
          self.submitted = self.last_submit = now
        except Full:
          pass
      return self.current_box(roi, epoch, now)
    except Exception:
      self.disable('worker_setup_error')
      return None

  def current_box(self, roi, epoch, now):
    if self.result is None:
      return None
    generation, captured, previous, chosen = self.result
    if chosen is None or generation != epoch or not 0 <= now-captured <= 2.:
      return None
    x,y,w,h = roi
    px,py,pw,ph = previous
    if (not .7 < w/pw < 1.4 or not .7 < h/ph < 1.4 or
        abs(x-px) > w*.25 or abs(y-py) > h*.25):
      return None
    a,b,bw,bh = chosen['box']
    # Projection carries only the outline between detections, never lamp state.
    return (x+(a-px)*w/pw, y+(b-py)*h/ph, bw*w/pw, bh*h/ph)

  def current_state(self, roi, epoch, now):
    if self.current_box(roi, epoch, now) is None:
      return None
    _, captured, _, chosen = self.result
    if now-captured > min(1.5, self.interval+.25) or chosen['confidence'] < .8:
      return None
    return dict(state=chosen['state'], confidence=chosen['confidence'], age=now-captured)
