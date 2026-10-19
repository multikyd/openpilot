"""Bounded phone transfer status; the C4 supplies its own freshness clock."""
import math

STATES = {'waiting', 'receiving', 'saved', 'error', 'stopped'}


def sanitize_report(body, now):
  if not isinstance(body, dict) or body.get('state') not in STATES:
    raise ValueError('Invalid transfer state')
  result = {'state': body['state'], 'ts': float(now)}
  for key, maximum in (('pending', 10000), ('percent', 100)):
    value = body.get(key, 0)
    if type(value) is not int or not 0 <= value <= maximum:
      raise ValueError('Invalid ' + key)
    result[key] = value
  return result


def status_text(report, now):
  try:
    stamp = float(report['ts'])
    if not math.isfinite(stamp) or not 0 <= now-stamp < 30:
      return '', 'waiting'
    state = report['state']
    if now-stamp > 8:
      return ('LOG LINK LOST', 'error') if state not in ('saved', 'stopped') else ('', 'waiting')
    if state == 'stopped':
      return '', 'waiting'
    if state == 'saved':
      return 'BOOKMARK SAVED', 'saved'
    pending = max(0, min(10000, int(report.get('pending', 0))))
    if state == 'receiving':
      percent = max(0, min(100, int(report.get('percent', 0))))
      return f'LOG {percent}%  /  {pending} WAIT', state
    if state == 'error':
      return 'LOG RETRY', state
    return (f'LOG {pending} WAIT' if pending else ''), 'waiting'
  except (KeyError, ValueError, TypeError, OverflowError):
    return '', 'waiting'
