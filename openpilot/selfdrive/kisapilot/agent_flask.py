"""Put the KisaAgent Flask stack on AGNOS's writable data partition."""

import os
import sys
from pathlib import Path


PACKAGE_DIR = Path("/data/kisa-agent-packages") / sys.implementation.cache_tag


def add_package_path() -> None:
  package_dir = str(PACKAGE_DIR)
  if PACKAGE_DIR.is_dir() and package_dir not in sys.path:
    sys.path.insert(0, package_dir)


def main() -> None:
  match sys.argv[1:]:
    case ["check"]:
      add_package_path()
      from flask import Flask  # noqa: F401
    case ["install"]:
      command = [sys.executable, "-m", "pip", "install", "--disable-pip-version-check",
                 "--retries", "1", "--timeout", "10", "--target", str(PACKAGE_DIR),
                 "--upgrade", "flask"]
      os.execv(sys.executable, command)
    case _:
      raise SystemExit("Usage: python -m openpilot.selfdrive.kisapilot.agent_flask {check|install}")


if __name__ == "__main__":
  main()
