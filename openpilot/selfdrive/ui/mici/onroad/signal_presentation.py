"""Direction-specific signal lamps and timers, independent of graphics."""
import math


def timer_text(remain):
  return str(math.ceil(remain)) if math.isfinite(remain) and remain > 0 else '--'


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
  selected = 'straight' if display.movement == 'observed' else display.movement
  if selected in ('straight', 'left', 'right'):
    return [('', getattr(display, selected+'State', 0), timer_text(getattr(display, selected+'Remain', 0.)))]
  if display.source in ('TMAP', 'VISION'):
    rows = [(label, getattr(display, direction+'State', 0),
             timer_text(getattr(display, direction+'Remain', 0.)))
            for direction, label in directions if getattr(display, direction+'State', 0) in (1, 2, 3)]
    if rows:
      return rows
  return [('', 0, '--')]
