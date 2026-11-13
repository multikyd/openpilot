"""Allow only an isolated model frame skip between recent accepted odometry samples."""
import math


class CameraOdometryValidity:
  MAX_GAP = 0.15  # seconds; longer outages must retain the normal failure path

  def __init__(self):
    self.previous = None

  def update(self, valid, frame_id, timestamp_eof, log_time, accepted):
    previous = self.previous
    self.previous = (frame_id, timestamp_eof, log_time, bool(valid and accepted))
    if valid:
      return True
    if previous is None or not previous[3]:
      return False
    # modeld deliberately invalidates the pose after skipping a camera frame.
    # Never feed that pose to the estimator or extend this grace to another bad sample.
    frame, eof, stamp, _ = previous
    return (frame_id == frame + 2 and
            all(math.isfinite(v) for v in (timestamp_eof, log_time, eof, stamp)) and
            0 < timestamp_eof - eof <= self.MAX_GAP and
            0 < log_time - stamp <= self.MAX_GAP)


class LocationCameraOdometryValidity:
  """Bridge at most 0.5 s of skipped-camera odometry with healthy other inputs.

  This changes the temporary-error input flag only. Invalid poses are never
  passed to the estimator. Calibration retains its independent 0.15 s policy.
  """
  MAX_GAP = .5
  MAX_FRAME_GAP = 10  # camera frame IDs advance at 20 Hz

  def __init__(self):
    self.last_good = self.previous = None
    self.holding = False

  def update(self, valid, frame_id, timestamp_eof, log_time, accepted, *, allow_grace):
    previous = self.previous
    was_holding = self.holding
    self.previous = (frame_id, timestamp_eof, log_time)
    self.holding = False
    finite = all(math.isfinite(v) for v in (timestamp_eof, log_time))
    if valid:
      self.last_good = ((frame_id, timestamp_eof, log_time) if
                        accepted and allow_grace and finite and frame_id >= 0 and
                        0 <= log_time-timestamp_eof <= self.MAX_GAP else None)
      # Keep the existing handling of estimator rejects: their separate bad
      # input counters decide validity; they cannot seed subsequent grace.
      return True
    good = self.last_good
    if (not allow_grace or good is None or previous is None or not finite or
        not 0 <= log_time-timestamp_eof <= self.MAX_GAP):
      self.last_good = None
      return False
    frame, eof, stamp = good
    prev_frame, prev_eof, prev_stamp = previous
    self.holding = bool(
      2 <= frame_id-frame <= self.MAX_FRAME_GAP and
      frame_id > prev_frame and timestamp_eof > prev_eof and log_time > prev_stamp and
      0 < timestamp_eof-eof <= self.MAX_GAP and 0 < log_time-stamp <= self.MAX_GAP and
      (was_holding or previous == good))
    if not self.holding:
      # A broken/expired window needs a newly accepted sample to rearm.
      self.last_good = None
    return self.holding


class CalibrationOdometryValidity:
  """Retain trusted calibration only for one fresh, isolated odometry skip.

  This never makes an invalid odometry sample usable for calibration updates.
  The caller must separately keep all other service and calibration checks.
  """
  def __init__(self):
    self.camera = CameraOdometryValidity()
    self.holding = False
    self.eof = self.stamp = None

  def update(self, valid, frame_id, timestamp_eof, log_time, *, trusted, now):
    isolated = self.camera.update(valid, frame_id, timestamp_eof, log_time, valid and trusted)
    self.eof, self.stamp = timestamp_eof, log_time
    self.holding = bool(not valid and trusted and isolated)
    return self.can_hold(now, trusted=trusted)

  def can_hold(self, now, *, trusted):
    fresh = (self.holding and trusted and
             all(math.isfinite(v) for v in (now, self.eof, self.stamp)) and
             0 <= self.stamp - self.eof <= self.camera.MAX_GAP and
             0 <= now - self.eof <= self.camera.MAX_GAP and
             0 <= now - self.stamp <= self.camera.MAX_GAP)
    if not fresh:
      self.holding = False
    return bool(fresh)
