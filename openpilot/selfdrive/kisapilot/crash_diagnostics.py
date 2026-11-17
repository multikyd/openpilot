"""Keep fatal Python stacks without suppressing the native crash handler."""
import faulthandler
import json
import os
from pathlib import Path
import signal
import sys
import time

FAULT_DIR = Path('/data/carrot_crash')
_fault_file = None


def enable_crash_diagnostics(directory=FAULT_DIR):
  global _fault_file
  if _fault_file is not None:
    return str(_fault_file.name)
  try:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    # Only rotate our own inactive traces; never touch system crash dumps.
    for old in sorted(directory.glob('carrot-fault-*.log'), key=lambda p: p.stat().st_mtime)[:-4]:
      old.unlink()
    path = directory / f'carrot-fault-{time.strftime("%Y%m%d-%H%M%S")}-{os.getpid()}.log'
    output = path.open('a', encoding='utf-8', buffering=1)
    metadata = dict(pid=os.getpid(), wall=time.time(), monotonic=time.monotonic(),
                    python=sys.version, executable=sys.executable)
    try:
      import resource
      metadata['core_limit'] = resource.getrlimit(resource.RLIMIT_CORE)
      metadata['core_pattern'] = Path('/proc/sys/kernel/core_pattern').read_text().strip()
    except (ImportError, OSError, ValueError):
      pass
    output.write(json.dumps(metadata) + '\n')
    output.flush()
    faulthandler.enable(file=output, all_threads=True)
    # Retain the descriptor for the lifetime of the signal handler.
    _fault_file = output
    if hasattr(signal, 'SIGUSR1'):
      try:
        faulthandler.register(signal.SIGUSR1, file=output, all_threads=True)
      except (OSError, RuntimeError, ValueError):
        pass
    return str(path)
  except (OSError, RuntimeError, ValueError) as error:
    print(f'carrot fatal-stack file unavailable: {error}', file=sys.stderr)
    try:
      faulthandler.enable(all_threads=True)
    except (OSError, RuntimeError, ValueError):
      pass
    return ''
