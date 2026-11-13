"""Sparse diagnostics for frame delivery delays, without changing validity."""
import math


class PipelineTiming:
  def __init__(self):
    self.previous = None
    self.last_report = -math.inf

  def update(self, now, camera_eof, receive_seconds, preprocess_seconds, inference_seconds):
    values = (now, camera_eof, receive_seconds, preprocess_seconds, inference_seconds)
    if not all(math.isfinite(v) for v in values):
      return None
    gap = now-self.previous if self.previous is not None else 0.
    self.previous = now
    age = now-camera_eof
    if (gap <= .09 and age <= .1) or now-self.last_report < 2.:
      return None
    self.last_report = now
    return dict(publish_gap_ms=round(gap*1000., 1), frame_age_ms=round(age*1000., 1),
                receive_ms=round(receive_seconds*1000., 1), preprocess_ms=round(preprocess_seconds*1000., 1),
                inference_ms=round(inference_seconds*1000., 1))
