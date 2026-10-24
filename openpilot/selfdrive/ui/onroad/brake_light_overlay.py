"""Display-only brake-lamp experiment for the C4 road view.

CPU work runs at <=10 Hz on one bounded copied ROI, with no work queue backlog.
No state is published to planning/control. cloudlog events enter normal rlogs.
"""
import os
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import pyray as rl
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA
from openpilot.selfdrive.ui.onroad.brake_lights import AdaptiveLamps, nv12_crop, project_search


class BrakeLightOverlay:
  def __init__(self):
    self.enabled = os.environ.get("LEAD_BRAKE_LIGHTS_UI", "1") != "0"
    self.pool = None  # lazily allocated; missing feature never prevents UI startup
    self.pending = None
    self.epoch = 0
    self.adaptive = AdaptiveLamps()
    self.previous = None
    self.display = None
    self.last_submit = -1.
    self.last_frame = None
    self.last_camera_frame = None
    self.last_camera_time = 0.
    self.last_log = None
    self.last_event = -30.
    self.retry_after = 0.
    self.context = None
    self.label_cache = None
    self.pending_sample = None

  def clear(self):
    if self.previous is not None or self.display is not None:
      self.epoch += 1
      self.adaptive = AdaptiveLamps()
      cloudlog.event("lead_brake_lights_ui", state="HIDDEN", reason="TARGET_OR_DATA_LOST", display_only=True)
      self.last_log = ("HIDDEN", "TARGET_OR_DATA_LOST")
    self.previous = None
    self.display = None
    self.last_frame = None
    self.label_cache = None
    self.pending_sample = None

  def label_sample(self, now):
    sample = self.label_cache
    if (not self.enabled or self.previous is None or sample is None or
        not 0 <= now-sample['mono'] <= .35 or not 0 <= now-sample['source_sof'] <= .4):
      return None
    return sample

  def latest_observation(self, now=None):
    now = time.monotonic() if now is None else now
    if (not self.enabled or self.previous is None or self.display is None or
        not 0 <= now-self.display[0] <= .35):
      return None
    return self.display[3]

  def close(self):
    self.clear()
    if self.pool is not None:
      self.pool.shutdown(wait=False, cancel_futures=True)
      self.pool = None

  def render(self, view, rect, sm, config, calibration, stream):
    if not self.enabled:
      return
    now = time.monotonic()
    if now < self.retry_after:
      return
    try:
      self._render(view, rect, sm, config, calibration, stream, now)
    except Exception:
      # Optional diagnostics must not take down the driving UI.
      self.clear()
      self.pending = None
      self.retry_after = now+30.
      cloudlog.exception("lead_brake_lights_ui_error")

  def _render(self, view, rect, sm, config, calibration, stream, now):
    frame = view.frame
    camera_key = (stream, frame.frame_id) if frame is not None else None
    if camera_key != self.last_camera_frame:
      self.last_camera_frame, self.last_camera_time = camera_key, now
    fresh = all(sm.valid[s] and sm.alive[s] and now-sm.recv_time[s] < .35
                for s in ("modelV2", "radarState"))
    calibrated = sm.valid['extrinsicsCalibration'] and getattr(sm['extrinsicsCalibration'].calStatus, 'raw', sm['extrinsicsCalibration'].calStatus) == 1
    lead = sm['radarState'].leadOne
    if (frame is None or not fresh or not calibrated or not lead.present or
        now-self.last_camera_time > .3 or abs(int(sm['modelV2'].frameId)-int(frame.frame_id)) > 8):
      self.clear()
      return
    # VisionIPC source timestamp also rejects a stale frame newly received after a stall.
    stamp = view.client.timestamp_sof / 1e9
    if stamp <= 0 or abs(now-stamp) > .4:
      self.clear()
      return
    if (frame.width, frame.height) != (config.width, config.height):
      self.clear()
      return
    model = sm['modelV2']
    if len(model.position.x) < 2 or len(model.position.z) != len(model.position.x):
      self.clear()
      return
    distance = float(lead.dRel)+RADAR_TO_CAMERA
    height = sm['extrinsicsCalibration'].height
    if not height:
      self.clear()
      return
    ground = float(np.interp(distance, model.position.x, model.position.z))+height[0]
    roi = project_search(config.intrinsics @ calibration, distance, float(lead.yRel), ground,
                         (frame.height, frame.width))
    if roi is None:
      self.clear()
      return
    current = (now, distance, float(lead.yRel), float(lead.vRel), int(lead.radarTrackId), stream)
    context = (stream, tuple(np.round(calibration.ravel(), 3)))
    if self.previous is not None:
      t, d, y, vr, track, old_stream = self.previous
      changed = (now-t > .6 or old_stream != stream or
                 (track != current[4] and (track >= 0 or current[4] >= 0)) or
                 abs(distance-(d+vr*(now-t))) > max(1.5, d*.06) or abs(current[2]-y) > .6 or
                 context != self.context)
      if changed:
        self.clear()
    self.context = context
    self.previous = current
    if self.pending is not None and self.pending.done():
      epoch, captured, result_roi, step, observation, frame_id, duration = self.pending.result()
      self.pending = None
      if epoch == self.epoch and now-captured < .35:
        self.display = (captured, result_roi, step, observation)
        if self.pending_sample is not None and self.pending_sample['frame_id'] == frame_id:
          self.label_cache = dict(self.pending_sample, prediction=observation.state,
                                  prediction_mono=captured, lamps=observation.lamps, reason=observation.reason)
        event = (observation.state, observation.reason)
        if event != self.last_log or now-self.last_event > 5.:
          state_changed = self.last_log is None or observation.state != self.last_log[0]
          if state_changed or now-self.last_event >= .5:
            cloudlog.event("lead_brake_lights_ui", state=observation.state, reason=observation.reason,
                           lamps=len(observation.lamps), gains=list(observation.gains), frame_id=frame_id,
                           d_rel=float(lead.dRel), v_rel=float(lead.vRel), processing_ms=round(duration, 2),
                           display_only=True)
            self.last_log, self.last_event = event, now
    if self.pending is None and now-self.last_submit >= .1-1e-6 and self.last_frame != camera_key:
      rgb, step = nv12_crop(frame, roi)
      state, epoch, frame_id = self.adaptive, self.epoch, int(frame.frame_id)
      prediction = self.latest_observation(now)
      self.pending_sample = dict(rgb=rgb, mono=now, capture_wall=time.time(), frame_id=frame_id,
                              source_sof=stamp, stream=int(stream), roi=list(roi), step=step,
                              radar_track_id=int(lead.radarTrackId), d_rel=float(lead.dRel),
                              y_rel=float(lead.yRel), v_rel=float(lead.vRel),
                              prediction=prediction.state if prediction else 'UNKNOWN',
                              prediction_mono=self.display[0] if prediction else None)
      def process():
        begin = time.monotonic()
        result = state.update(rgb, now)
        return epoch, now, roi, step, result, frame_id, (time.monotonic()-begin)*1000
      if self.pool is None:
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="lead_lamps")
      self.pending = self.pool.submit(process)
      self.last_submit, self.last_frame = now, camera_key
    transform = view._calc_frame_matrix(rect)
    sx, sy = rect.width*transform[0, 0]/frame.width, rect.height*transform[1, 1]/frame.height
    ox = rect.x+(rect.width-frame.width*sx)/2+transform[0, 2]*rect.width/2
    oy = rect.y+(rect.height-frame.height*sy)/2+transform[1, 2]*rect.height/2
    def screen(box):
      x, y, w, h = box
      return rl.Rectangle(float(ox+x*sx), float(oy+y*sy), float(w*sx), float(h*sy))
    # This is a radar-projected search/track area, not a trained vehicle boundary.
    region = screen(roi)
    rl.draw_rectangle_lines_ex(region, 2.5, rl.Color(65,220,245,240))
    if self.display is None or now-self.display[0] > .35:
      return
    captured, found_roi, step, observation = self.display
    # Keep older detections from sliding onto another target after a sudden position change.
    if abs(found_roi[0]-roi[0]) > roi[2]*.18 or abs(found_roi[1]-roi[1]) > roi[3]*.18:
      self.display = None
      return
    x, y, w, h = found_roi
    color = rl.Color(255, 165, 50, 235) if observation.state == 'ON' else rl.Color(85, 205, 235, 210)
    for a, b, bw, bh in observation.lamps:
      rl.draw_rectangle_lines_ex(screen((x+a*step, y+b*step, bw*step, bh*step)), 2., color)
