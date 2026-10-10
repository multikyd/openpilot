"""Disengage and verify fresh control feedback before agent-triggered reboot."""
import math
import subprocess
import threading
import time

REQUEST_KEY = "KisaRebootRequest"
LEASE_SECONDS = 15.0
REBOOT_LOCK = threading.Lock()


def reboot_requested(params, now=None):
  try:
    raw = params.get(REQUEST_KEY)
  except Exception:
    # An older native Params build has no key; do not crash driving processes.
    return False
  if not raw:
    return False
  try:
    deadline = float(raw)
    now = time.monotonic() if now is None else now
    return math.isfinite(deadline) and now < deadline <= now + LEASE_SECONDS + 2
  except (TypeError, ValueError):
    return False


class DeviceFeedback:
  def __init__(self):
    import cereal.messaging as messaging
    self.sm = messaging.SubMaster(['deviceState', 'selfdriveState', 'carControl', 'carState'])

  def cleared(self, since, now, params):
    self.sm.update(100)
    now = time.monotonic()
    def fresh(service):
      stamp = self.sm.logMonoTime[service] / 1e9
      return self.sm.valid[service] and since <= stamp <= now and now - stamp < 1.0
    if not fresh('deviceState'):
      return False
    if not self.sm['deviceState'].started and not params.get_bool('IsOnroad'):
      if fresh('carControl') and (self.sm['carControl'].enabled or self.sm['carControl'].latActive or self.sm['carControl'].longActive):
        return False
      if fresh('selfdriveState') and (self.sm['selfdriveState'].enabled or self.sm['selfdriveState'].active):
        return False
      if fresh('carState') and self.sm['carState'].cruiseState.enabled:
        return False
      return True
    if not all(fresh(s) for s in ('selfdriveState', 'carControl', 'carState')):
      return False
    sd, cc, cs = self.sm['selfdriveState'], self.sm['carControl'], self.sm['carState']
    return not (sd.enabled or sd.active or cc.enabled or cc.latActive or cc.longActive or cs.cruiseState.enabled)


class RebootGuard:
  def __init__(self, params, log=print, feedback=None, clock=time.monotonic, sleep=time.sleep, force=None):
    self.params, self.log = params, log
    self.feedback = DeviceFeedback() if feedback is None else feedback
    self.clock, self.sleep = clock, sleep
    self.force = force or (lambda: subprocess.run(['sudo', 'reboot'], check=True))
    self.stop = threading.Event()
    self.heartbeat = None
    self.locked = False

  def __enter__(self):
    if not REBOOT_LOCK.acquire(blocking=False):
      raise RuntimeError('Another reboot operation is in progress')
    self.locked = True
    try:
      self.started = self.clock()
      self.refresh()
      self.heartbeat = threading.Thread(target=self.keep_alive, daemon=True)
      self.heartbeat.start()
      self.log('Disengage requested; waiting for controls and vehicle cruise to clear...')
      self.wait_clear()
      return self
    except Exception:
      self.__exit__(None, None, None)
      raise

  def refresh(self):
    self.params.put(REQUEST_KEY, str(self.clock() + LEASE_SECONDS))

  def keep_alive(self):
    while not self.stop.wait(1.0):
      try:
        self.refresh()
      except Exception:
        self.stop.set()

  def wait_clear(self, timeout=10.0):
    deadline, stable = self.clock() + timeout, None
    while self.clock() < deadline:
      if self.stop.is_set() or not reboot_requested(self.params, self.clock()):
        raise RuntimeError('Disengage request expired; reboot cancelled')
      now = self.clock()
      if self.feedback.cleared(self.started, now, self.params):
        stable = now if stable is None else stable
        if now - stable >= 1.0:
          self.log('Disengage confirmed (controls inactive, vehicle cruise off).')
          return
      else:
        stable = None
      self.sleep(0.05)
    raise RuntimeError('Disengage/state confirmation timed out; reboot cancelled')

  def reboot(self, force=False):
    # Check again after long Git/model operations; never rely on an old acknowledgement.
    self.wait_clear()
    self.log('Rebooting (Force).' if force else 'Rebooting (Safe).')
    if force:
      self.force()
    else:
      self.params.put_bool('DoReboot', True)
    # Keep the request alive while manager/OS shuts down. Release if shutdown stalls.
    self.sleep(10.0)

  def __exit__(self, *_):
    self.stop.set()
    if self.heartbeat is not None:
      self.heartbeat.join(timeout=2.0)
    try:
      self.params.remove(REQUEST_KEY)
    finally:
      if self.locked:
        self.locked = False
        REBOOT_LOCK.release()
