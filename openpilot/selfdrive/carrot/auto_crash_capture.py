"""Persist unexpected exits; package evidence offroad in a low-priority child."""
import argparse
import json
import logging
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

EVENT_DIR = Path('/data/crash_capture')
DESTINATION = Path('/data/media/0/realdata')
log = logging.getLogger(__name__)


def atomic_json(path, value):
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_suffix(path.suffix + '.tmp')
  with temporary.open('w', encoding='utf-8') as output:
    json.dump(value, output, ensure_ascii=False)
    output.flush()
    os.fsync(output.fileno())
  os.replace(temporary, path)
  try:
    directory_fd = os.open(path.parent, os.O_RDONLY)
    try:
      os.fsync(directory_fd)
    finally:
      os.close(directory_fd)
  except OSError:
    pass  # Directory fsync is unsupported on some non-device test platforms.


class AutoCrashCapture:
  def __init__(self, directory=EVENT_DIR, destination=DESTINATION, spawn=subprocess.Popen,
               route_path=Path('/data/params/d/CurrentRoute')):
    self.directory, self.destination, self.spawn = Path(directory), Path(destination), spawn
    self.route_path = Path(route_path)
    self.events = queue.SimpleQueue()
    self.onroad = True  # Unknown startup state is never permission to compress.
    self.child = None
    self.retry_at = 0.
    self.wake = threading.Event()
    self.closed = False
    self.thread = None

  def record(self, process, pid, exit_code, signal_name):
    self.events.put(dict(process=process, pid=pid, exit_code=exit_code, signal=signal_name,
                         wall=time.time(), monotonic=time.monotonic()))
    self.wake.set()

  def pending(self):
    return [p for p in sorted(self.directory.glob('exit-*.json')) if not p.with_suffix('.done').exists()]

  def flush_events(self):
    while not self.events.empty():
      event = self.events.get()
      try:
        try:
          event['route'] = self.route_path.read_text(encoding='utf-8').strip()
          # Record segment identities now, not at the later collection time.
          from openpilot.selfdrive.carrot.collect_crash import nearby_segments
          event['segments'] = nearby_segments(self.destination, event['route'])
        except Exception as error:
          event['route_error'] = str(error)
        path = self.directory / f"exit-{event['wall']:.6f}-{event['process']}-{event['pid']}.json"
        atomic_json(path, event)
      except OSError:
        self.events.put(event)
        raise

  def step(self):
    self.flush_events()
    if self.child is not None:
      code = self.child.poll()
      if code is None:
        if self.onroad:
          self.child.terminate()
        return
      self.child = None
      if code != 0:
        self.retry_at = time.monotonic() + 60
    if self.closed or self.onroad or time.monotonic() < self.retry_at:
      return
    pending = self.pending()
    if pending:
      self.child = self.spawn([sys.executable, '-m', 'openpilot.selfdrive.carrot.auto_crash_capture',
                               '--collect', str(pending[0]), '--output', str(self.destination)],
                              stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

  def set_onroad(self, onroad):
    self.onroad = bool(onroad)
    self.wake.set()

  def start(self):
    if self.thread is None:
      self.thread = threading.Thread(target=self.run, name='auto-crash-capture', daemon=True)
      self.thread.start()

  def run(self):
    while not self.closed:
      self.wake.clear()
      try:
        self.step()
      except Exception:
        log.exception('Automatic crash capture deferred; original evidence retained')
        self.retry_at = time.monotonic() + 60
      self.wake.wait(1.)

  def close(self):
    self.closed = True
    self.wake.set()
    if self.thread is not None:
      self.thread.join(timeout=2.)
    if self.child is not None and self.child.poll() is None:
      self.child.terminate()


_capture = None


def capture_service():
  global _capture
  if _capture is None:
    _capture = AutoCrashCapture()
    _capture.start()
  return _capture


def package_event(path, destination):
  # Survive manager restarts without two collectors packaging the same event.
  with Path(str(path) + '.lock').open('a') as lock:
    try:
      import fcntl
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except ImportError:
      pass
    return _package_event(path, destination)


def _package_event(path, destination):
  from openpilot.selfdrive.carrot.collect_crash import collect
  try:
    os.nice(19)
  except (AttributeError, OSError):
    pass
  event = json.loads(Path(path).read_text(encoding='utf-8'))
  archive = collect(destination, event=event, automatic=True)
  atomic_json(Path(path).with_suffix('.done'), dict(archive=str(archive), completed=time.time()))
  # Only rotate our automatic archives after a successful collection.
  archives = sorted(Path(destination).glob('process-crash-report-*.tar.gz'), key=lambda p: p.stat().st_mtime)
  for old in archives[:-5]:
    old.unlink()
  return archive


def guard_offroad(parent_pid, state_path=Path('/data/params/d/IsOffroad')):
  # Independent of the manager: stop if it dies or recording starts again.
  while True:
    try:
      allowed = os.getppid() == parent_pid and state_path.read_bytes().strip() == b'1'
    except OSError:
      allowed = False
    if not allowed:
      os._exit(75)  # Event remains pending; partial archive is never marked complete.
    time.sleep(.5)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--collect', required=True)
  parser.add_argument('--output', default=str(DESTINATION))
  args = parser.parse_args()
  threading.Thread(target=guard_offroad, args=(os.getppid(),), daemon=True).start()
  print(package_event(args.collect, args.output))


if __name__ == '__main__':
  main()
