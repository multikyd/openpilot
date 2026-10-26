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
from openpilot.selfdrive.ui.onroad.brake_vehicle_model import BrakeVehicleModel, fuse_observation
from openpilot.selfdrive.ui.onroad.brake_lights import AdaptiveLamps, VehicleOutline, copy_nv12_roi, nv12_crop, project_search


class BrakeLightOverlay:
  def __init__(self):
    self.enabled = os.environ.get("LEAD_BRAKE_LIGHTS_UI", "1") != "0"
    self.pool = None  # lazily allocated; missing feature never prevents UI startup
    self.pending = None
    self.epoch = 0
    self.adaptive = AdaptiveLamps()
    self.vehicle_outline = VehicleOutline()
    self.show_search = os.environ.get("LEAD_BRAKE_SEARCH_UI", "0") == "1"
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
    self.vehicle_model = BrakeVehicleModel()
    self.last_vehicle_status = None
    self.last_vehicle_log = -10.
    self.model_roi = None

  def clear(self):
    self.vehicle_model.clear()
    self.model_roi = None
    if self.previous is not None or self.display is not None:
      self.epoch += 1
      self.adaptive = AdaptiveLamps()
      self.vehicle_outline = VehicleOutline()
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
    if not self.enabled or self.previous is None:
      return None
    observation = self.display[3] if self.display is not None and 0 <= now-self.display[0] <= .35 else None
    hint = self.vehicle_model.current_state(self.model_roi, self.epoch, now) if self.model_roi is not None else None
    return fuse_observation(observation, hint) if observation is not None or hint is not None else None

  def close(self):
    self.clear()
    self.vehicle_model.close()
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
    self.model_roi = roi
    model_body = self.vehicle_model.tick(frame, roi, self.epoch, now, fps=rl.get_fps())
    if self.vehicle_model.status != self.last_vehicle_status or (self.vehicle_model.report is not None and now-self.last_vehicle_log >= 5.):
      cloudlog.event("lead_vehicle_model_ui", status=self.vehicle_model.status,
                     result=self.vehicle_model.report, frame_id=int(frame.frame_id),
                     radar_track_id=int(lead.radarTrackId), display_only=True)
      self.last_vehicle_status, self.last_vehicle_log = self.vehicle_model.status, now
    if self.pending is not None and self.pending.done():
      epoch, captured, result_roi, result_step, observation, frame_id, duration, rgb, body = self.pending.result()
      self.pending = None
      if epoch == self.epoch and now-captured < .35:
        self.display = (captured, result_roi, result_step, observation, body)
        shown = fuse_observation(observation, self.vehicle_model.current_state(roi, self.epoch, now))
        if self.pending_sample is not None and self.pending_sample['frame_id'] == frame_id:
          self.label_cache = dict(self.pending_sample, rgb=rgb, step=result_step, resampling='point', prediction=shown.state, legacy_prediction=observation.state,
                                  prediction_mono=captured, lamps=observation.lamps, reason=shown.reason)
        event = (shown.state, shown.reason)
        if event != self.last_log or now-self.last_event > 5.:
          state_changed = self.last_log is None or shown.state != self.last_log[0]
          if state_changed or now-self.last_event >= .5:
            cloudlog.event("lead_brake_lights_ui", state=shown.state, reason=shown.reason, legacy_state=observation.state,
                           lamps=len(observation.lamps), gains=list(observation.gains), frame_id=frame_id,
                           d_rel=float(lead.dRel), v_rel=float(lead.vRel), processing_ms=round(duration, 2),
                           roi=list(result_roi), step=result_step, lamp_boxes=list(observation.lamps),
                           vehicle_box=body,
                           display_only=True)
            self.last_log, self.last_event = event, now
    if self.pending is None and now-self.last_submit >= .1-1e-6 and self.last_frame != camera_key:
      copied, local_roi = copy_nv12_roi(frame, roi)
      state, epoch, frame_id = self.adaptive, self.epoch, int(frame.frame_id)
      prediction = self.latest_observation(now)
      self.pending_sample = dict(mono=now, capture_wall=time.time(), frame_id=frame_id,
                              source_sof=stamp, stream=int(stream), roi=list(roi),
                              radar_track_id=int(lead.radarTrackId), d_rel=float(lead.dRel),
                              y_rel=float(lead.yRel), v_rel=float(lead.vRel),
                              prediction=prediction.state if prediction else 'UNKNOWN',
                              prediction_mono=self.display[0] if prediction else None)
      def process(state=state, outline=self.vehicle_outline, copied=copied, crop=local_roi,
                  epoch=epoch, captured=now, result_roi=roi, frame_id=frame_id):
        begin = time.monotonic()
        rgb, result_step = nv12_crop(copied, crop)
        result = state.update(rgb, captured)
        body = outline.update(rgb, result.lamps, captured)
        return epoch, captured, result_roi, result_step, result, frame_id, (time.monotonic()-begin)*1000, rgb, body
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
    # A projected search region must not masquerade as a detected vehicle edge.
    if self.show_search:
      rl.draw_rectangle_lines_ex(screen(roi), 1., rl.Color(65,220,245,90))
    shown = self.latest_observation(now)
    display_state = shown.state if shown is not None else 'UNKNOWN'
    # Cyan means detected vehicle, not a confirmed brake-OFF classification.
    # Only positive brake evidence changes the outline to orange.
    color = rl.Color(255,145,0,255) if display_state == 'ON' else rl.Color(0,220,255,255)
    def draw_vehicle(box):
      bounds = screen(box)
      rl.draw_rectangle_lines_ex(bounds, 8., rl.Color(0,0,0,230))
      rl.draw_rectangle_lines_ex(bounds, 5., color)
    if model_body is not None:
      draw_vehicle(model_body)
    if self.display is None or now-self.display[0] > .35:
      return
    captured, found_roi, display_step, observation = self.display[:4]
    # Keep older detections from sliding onto another target after a sudden position change.
    if abs(found_roi[0]-roi[0]) > roi[2]*.18 or abs(found_roi[1]-roi[1]) > roi[3]*.18:
      self.display = None
      return
    x, y, w, h = found_roi
    body = self.display[4] if len(self.display) > 4 else None
    if body is not None and model_body is None:
      a,b,bw,bh = body
      draw_vehicle((x+a*display_step,y+b*display_step,bw*display_step,bh*display_step))
    # Individual lamp locations remain internal observations/label metadata.
    # Only the whole vehicle outline carries the displayed ON/OFF/UNKNOWN state.
