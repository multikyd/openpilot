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
  """Unknown movement retains labelled live timers instead of discarding them."""
  directions = (('straight', 'S'), ('left', 'L'), ('right', 'R'))
  advance = TMAP_COUNTDOWN_ADVANCE if display.source == 'TMAP' else 0.
  selected = 'straight' if display.movement == 'observed' else display.movement
  if selected in ('straight', 'left', 'right'):
    return [('', getattr(display, selected+'State', 0), timer_text(getattr(display, selected+'Remain', 0.), advance))]
  if display.source in ('TMAP', 'VISION'):
    rows = [(label, getattr(display, direction+'State', 0),
             timer_text(getattr(display, direction+'Remain', 0.), advance))
            for direction, label in directions if getattr(display, direction+'State', 0) in (1, 2, 3)]
    if rows:
      return rows
  return [('', 0, '--')]
