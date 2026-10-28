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
from openpilot.common.brake_model_runtime import runtime_directory, use_runtime, mark_broken
from openpilot.selfdrive.ui.onroad.brake_lights import Observation, copy_nv12_roi

LEGACY_MODEL = Path(__file__).resolve().parent / 'models' / 'brake_light_v4.onnx'
LIGHT_MODEL = LEGACY_MODEL.with_name('brake_light_v4_320.onnx')
QUANT_MODEL = LEGACY_MODEL.with_name('brake_light_v4_320_int8.onnx')
MODEL = LIGHT_MODEL if LIGHT_MODEL.is_file() else LEGACY_MODEL


def intersection(a, b):
  return max(0., min(a[0]+a[2], b[0]+b[2])-max(a[0], b[0])) * max(0., min(a[1]+a[3], b[1]+b[3])-max(a[1], b[1]))


def decode_predictions(output, width, height, confidence=.65, input_size=640):
  raw = np.asarray(output)
  if input_size not in (320, 640) or raw.shape != (1, 6, sum((input_size//s)**2 for s in (8,16,32))):
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
    x1, y1 = max(0., float(cx-w/2)*width/input_size), max(0., float(cy-h/2)*height/input_size)
    x2, y2 = min(float(width), float(cx+w/2)*width/input_size), min(float(height), float(cy+h/2)*height/input_size)
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
  if observation.reason in ('RELEASE_PENDING', 'CENTER_LAMP_RELEASE'):
    return observation
  if observation.state == 'OFF' and hint['state'] == 'ON':
    return Observation('UNKNOWN', 'MODEL_LAMP_CONFLICT', observation.lamps, observation.gains)
  return Observation(hint['state'], 'MODEL_'+hint['state'], observation.lamps, observation.gains)


def nv12_input(frame, width, height, input_size=640):
  # Keep the original full-frame NV12 conversion; only network input size varies.
  if input_size not in (320, 640):
    raise ValueError('Unexpected brake model input size')
  xx = np.clip((np.arange(input_size)+.5)*width/input_size-.5, 0, width-1).astype(int)
  yy = np.clip((np.arange(input_size)+.5)*height/input_size-.5, 0, height-1).astype(int)
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
      # Share the existing background CPU pool instead of pinning to one core.
      os.sched_setaffinity(0, set(range(min(4, os.cpu_count() or 1))))
    except OSError:
      pass
  if hasattr(os, 'nice'):
    os.nice(max(0, 10-os.nice(0)))


def _worker(inbox, outbox, path):
  try:
    _background_priority()
    runtime = use_runtime()
    try:
      import onnxruntime as ort
    except Exception:
      mark_broken(runtime)
      raise
    paths = [Path(path)]
    if (paths[0].name == LIGHT_MODEL.name and QUANT_MODEL.is_file() and
        os.getenv('LEAD_BRAKE_VARIANT', 'auto') == 'auto'):
      paths.append(QUANT_MODEL)
    best, benchmarks = None, []
    # Benchmark two background threads only when single-thread latency cannot
    # comfortably meet the existing freshness budget. Never use realtime CPUs.
    for candidate in paths:
      thread_counts = [1]
      for threads in thread_counts:
        try:
          options = ort.SessionOptions()
          options.intra_op_num_threads = threads
          options.inter_op_num_threads = 1
          options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
          options.log_severity_level = 3
          options.add_session_config_entry('session.intra_op.allow_spinning', '0')
          options.add_session_config_entry('session.inter_op.allow_spinning', '0')
          load_start = time.monotonic()
          candidate_session = ort.InferenceSession(str(candidate), options, providers=['CPUExecutionProvider'])
          inp = candidate_session.get_inputs()[0]
          size = inp.shape[2]
          if (size not in (320, 640) or inp.shape != [1,3,size,size] or
              candidate_session.get_outputs()[0].shape != [1,6,sum((size//s)**2 for s in (8,16,32))]):
            raise ValueError('Unexpected brake model interface')
          load_ms = (time.monotonic()-load_start)*1000
          outbox.put(('phase', 'warmup'))
          dummy = np.zeros((1,3,size,size), dtype=np.float32)
          samples, cpu_samples = [], []
          for _ in range(2):
            started, cpu_started = time.monotonic(), time.process_time()
            candidate_session.run(None, {inp.name: dummy})
            samples.append(time.monotonic()-started)
            cpu_samples.append(time.process_time()-cpu_started)
          benchmarks.append(dict(model=candidate.name, threads=threads, load_ms=round(load_ms,1),
                                 cold_ms=round(samples[0]*1000,1), steady_ms=round(samples[1]*1000,1),
                                 cpu_ms=round(cpu_samples[1]*1000,1)))
          if threads == 1 and size == 320 and samples[1] > .7 and (os.cpu_count() or 1) >= 2:
            thread_counts.append(2)
          # Prefer FP32/one thread on a near tie.
          if best is None or samples[1] < best[0]*.95:
            best = (samples[1], candidate_session, inp, size, candidate.name, threads)
          candidate_session = None  # do not retain a rejected session/thread pool
        except Exception as exc:
          benchmarks.append(dict(model=candidate.name, threads=threads, error=type(exc).__name__))
    if best is None:
      raise RuntimeError('No usable brake model candidate')
    elapsed, session, inp, size, model_name, threads = best
    candidate_session = None  # release the non-selected session before live inference
    outbox.put(('ready', elapsed, dict(model=model_name, input_size=size, threads=threads, benchmarks=benchmarks)))
    while True:
      job = inbox.get()
      if job is None:
        return
      if job == ('ping',):
        continue
      epoch, captured, roi, width, height, copied = job
      start, cpu_start = time.monotonic(), time.process_time()
      output = session.run(None, {inp.name: nv12_input(copied, width, height, size)})[0]
      chosen = select_lead_box(decode_predictions(output, width, height, input_size=size), roi)
      outbox.put(('result', epoch, captured, roi, chosen, time.monotonic()-start,
                  dict(cpu_ms=round((time.process_time()-cpu_start)*1000,1))))
  except Exception as exc:
    try:
      outbox.put(('error', type(exc).__name__))
    except (OSError, ValueError):
      pass


class BrakeVehicleModel:
  def __init__(self):
    variant = os.getenv('LEAD_BRAKE_VARIANT', 'auto')
    self.model_path = LEGACY_MODEL if variant == 'legacy' else QUANT_MODEL if variant == 'int8' and QUANT_MODEL.is_file() else MODEL
    self.input_size = 640 if self.model_path == LEGACY_MODEL else 320
    self.enabled = (os.environ.get('LEAD_BRAKE_MODEL', '1') != '0' and self.model_path.is_file() and
                    (runtime_directory() is not None or importlib.util.find_spec('onnxruntime') is not None))
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
    self.retry_count = 0
    self.retry_at = 0.
    self.phase = 'idle'
    self.phase_started = 0.
    self.launch_generation = 0

  def clear(self):
    self.result = None

  def close(self):
    self.launch_generation += 1
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

  def _fail(self, reason, now, elapsed=None):
    # Distinguish a dead process from a live-but-slow worker before cleanup.
    self.report = dict(self.report or {}, failure=reason, phase=self.phase,
                       elapsed_ms=round(elapsed*1000, 1) if elapsed is not None else None,
                       exit_code=getattr(self.process, 'exitcode', None),
                       worker_pid=getattr(self.process, 'pid', None), retry_count=self.retry_count)
    self.result = None
    self.close()
    if self.retry_count < 2:
      self.retry_count += 1
      self.retry_at = now+30.*self.retry_count
      self.status = 'retry_wait:'+reason
      self.report.update(retry_count=self.retry_count, retry_delay_s=30.*self.retry_count)
      self.launch_error = False
    else:
      self.enabled = False
      self.status = 'disabled:'+reason

  def _launch(self, generation=None):
    generation = self.launch_generation if generation is None else generation
    try:
      _background_priority()
      ctx = mp.get_context('spawn')
      inbox, outbox = ctx.Queue(maxsize=1), ctx.Queue(maxsize=4)
      # Start the serialization feeder from this non-realtime thread.
      inbox.put_nowait(('ping',))
      process = ctx.Process(target=_worker, args=(inbox,outbox,str(self.model_path)), daemon=True)
      process.start()
      if not self.enabled or not self.launching or generation != self.launch_generation:
        process.terminate()
        process.join(timeout=.01)
        for q in (inbox,outbox):
          q.cancel_join_thread()
          q.close()
        return
      self.inbox, self.outbox, self.process = inbox, outbox, process
      self.launching = False
    except Exception as exc:
      if generation == self.launch_generation:
        self.launch_error = type(exc).__name__
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
    if now < self.retry_at:
      return None
    try:
      if self.launch_error:
        self._fail('worker_setup_error:'+str(self.launch_error), now)
        return None
      if self.process is None:
        if not self.launching:
          self.launching = True
          self.started = now
          self.phase = self.status = 'loading'
          self.phase_started = now
          self.last_submit = -100.
          Thread(target=self._launch, args=(self.launch_generation,), name='brake_model_launch', daemon=True).start()
        elif now-self.started > 20.:
          self._fail('worker_start_timeout', now, now-self.started)
        return None
      for _ in range(4):
        try:
          message = self.outbox.get_nowait()
        except Empty:
          break
        if message[0] == 'phase':
          self.phase = self.status = message[1]
          self.phase_started = now
        elif message[0] == 'ready':
          self.ready = True
          self.phase = self.status = 'ready'
          metadata = message[2] if len(message)>2 else {}
          self.input_size = metadata.get('input_size', self.input_size)
          self.report = dict(metadata, warmup_ms=round(message[1]*1000,1) if len(message)>1 else None,
                             retry_count=self.retry_count)
        elif message[0] == 'error':
          self._fail('load_or_run_error:'+message[1], now)
          return None
        elif message[0] == 'result':
          _, generation, captured, found_roi, chosen, elapsed = message[:6]
          self.busy = False
          self.report = dict(self.report or {}, duration_ms=round(elapsed*1000,1), state=chosen['state'] if chosen else 'UNKNOWN',
                             confidence=chosen['confidence'] if chosen else 0., matched=chosen is not None)
          if len(message)>6:
            self.report.update(message[6])
          cpu_seconds = float(self.report.get('cpu_ms', elapsed*1000))/1000
          # Budget total CPU across threads, not only elapsed wall time.
          self.interval = max(1., elapsed*5, cpu_seconds*5)
          if elapsed > 1.:
            self._fail('inference_too_slow', now, elapsed)
            return None
          if generation == epoch and 0 <= now-captured < 1.:
            self.result = (generation, captured, found_roi, chosen)
      if not self.process.is_alive():
        self._fail('worker_exit', now)
        return None
      if not self.ready and now-self.started > 60.:
        self._fail('warmup_timeout' if self.phase == 'warmup' else 'model_load_timeout', now, now-self.started)
        return None
      if self.busy and now-self.submitted > 2.:
        self._fail('inference_timeout', now, now-self.submitted)
        return None
      if self.ready and not self.busy and now-self.last_submit >= self.interval:
        copied, _ = copy_nv12_roi(frame, (0,0,frame.width,frame.height))
        try:
          self.inbox.put_nowait((epoch,now,roi,frame.width,frame.height,copied))
          self.busy = True
          self.phase = 'inference'
          self.submitted = self.last_submit = now
        except Full:
          pass
      return self.current_box(roi, epoch, now)
    except Exception as exc:
      self._fail('worker_setup_error:'+type(exc).__name__, now)
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
    state_threshold = .9 if self.input_size == 320 else .8
    if now-captured > min(1.5, self.interval+.25) or chosen['confidence'] < state_threshold:
      return None
    return dict(state=chosen['state'], confidence=chosen['confidence'], age=now-captured)
