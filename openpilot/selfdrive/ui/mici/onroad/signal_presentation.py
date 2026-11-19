"""Direction-specific signal lamps and timers, independent of graphics."""
import math

# User-selected display lead, in addition to measured phone transport age.
# Never applied to signal observations, learning, control or lamp colours.
TMAP_COUNTDOWN_ADVANCE = .5


def signal_panel_visible(display):
  """Keep learning/model observations out of the live signal panel."""
  if display.source in ('TMAP', 'VISION'):
    return (display.operatingMode in ('OFF', 'FLASH') or
            any(getattr(display, direction+'State', 0) in (1, 2, 3)
                for direction in ('straight', 'left', 'right')))
  if display.source == 'EST':
    low, high, confidence = display.estimateLow, display.estimateHigh, display.confidence
    return (all(math.isfinite(v) for v in (low, high, confidence)) and
            0 <= low <= high and high > 0 and confidence > 0 and
            (display.operatingMode == 'WAIT' or display.predictionState in (1, 2)))
  return False


def timer_text(remain, advance=0.):
  return str(math.ceil(max(0., remain-advance))) if math.isfinite(remain) and remain > 0 else '--'


def signal_lamps(display):
  """Round lamps show straight; the turning arrow illuminates green only."""
  if display.source not in ('TMAP', 'VISION'):
    return (0, 0, 0)
  straight = display.straightState
  arrow = getattr(display, 'rightState', 0) if display.movement == 'right' else display.leftState
  return (straight if straight in (1, 3) else 0, 2 if arrow == 2 else 0,
          2 if straight == 2 else 0)


def countdown_rows(display):
  """Show only the route movement's received countdown."""
  movement = 'straight' if display.movement == 'observed' else display.movement
  if movement not in ('straight', 'left', 'right'):
    return [('', 0, '--')]
  advance = TMAP_COUNTDOWN_ADVANCE if display.source == 'TMAP' else 0.
  state = getattr(display, movement+'State', 0)
  remain = getattr(display, movement+'Remain', 0.)
  return [('', state, timer_text(remain, advance) if state in (1, 2, 3) else '--')]


def next_green_wait(display, stopped, state, value):
  return (stopped and display.source == 'TMAP' and display.movement in ('straight', 'left', 'right') and
          display.operatingMode not in ('OFF', 'FLASH') and state == 1 and value.isdecimal())


def signal_source_label(source):
  """Navigation provider label only; retain original source IDs internally."""
  return 'NAVI' if source == 'TMAP' else source
