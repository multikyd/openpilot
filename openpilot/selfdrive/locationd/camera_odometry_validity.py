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
