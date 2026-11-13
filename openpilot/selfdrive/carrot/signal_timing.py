"""Countdown display age; never infer a lamp transition from an expired timer."""
import math


def display_timing(timestamp_ms, receive_wall):
  try:
    source = float(timestamp_ms)/1000.
    age = float(receive_wall)-source
    if not all(math.isfinite(x) for x in (source, age)) or source < 1600000000:
      raise ValueError
    # Different clocks, a future stamp or an old callback are not a measured delay.
    if not 0 <= age <= 3.:
      return {'display_age_s': 0., 'display_timing': 'clock_or_age_mismatch'}
    return {'display_age_s': age, 'source_timestamp_ms': int(float(timestamp_ms)),
            'display_timing': 'phone_timestamp'}
  except (TypeError, ValueError, OverflowError):
    return {'display_age_s': 0., 'display_timing': 'no_timestamp'}
