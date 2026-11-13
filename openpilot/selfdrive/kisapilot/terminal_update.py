"""Terminal gi uses the same disengage-confirmed update as KisaAgent."""
import json
from pathlib import Path
import re
import sys
import time
from urllib.request import Request, urlopen

BASE = "http://127.0.0.1:5005"
ALIAS = "alias gi='python3 /data/openpilot/openpilot/selfdrive/kisapilot/terminal_update.py'"


def install_alias(path):
  path = Path(path)
  content = path.read_text() if path.exists() else ""
  # Replace older gi aliases, preserving all other user shell configuration.
  content = re.sub(r"^[ \t]*alias[ \t]+gi=.*(?:\n|$)", "", content, flags=re.MULTILINE)
  path.write_text(content.rstrip("\n") + "\n" + ALIAS + "\n")


def request_json(path, payload=None):
  data = None if payload is None else json.dumps(payload).encode()
  request = Request(BASE + path, data=data, headers={"Content-Type": "application/json"})
  with urlopen(request, timeout=5) as response:
    return json.load(response)


def update(request=request_json, sleep=time.sleep, clock=time.monotonic):
  capability = request("/reboot/capabilities")
  if not capability.get("disengageBeforeReboot"):
    raise RuntimeError("Update C4 reboot integration first; no reboot was requested")
  result = request("/cmd/run", {"id": "git_pull_reboot"})
  command_id = result.get("id")
  if result.get("status") != "started" or not isinstance(command_id, str):
    raise RuntimeError("Update command was not accepted")
  print("Waiting for control/cruise disengagement, then Git pull and orderly reboot...", flush=True)
  deadline, printed = clock() + 600., 0
  while clock() < deadline:
    state = request("/cmd/output?id=" + command_id)
    output = state.get("output", "")
    print(output[printed:], end="", flush=True)
    printed = len(output)
    if "Operation stopped:" in output:
      raise RuntimeError("Update stopped; no forced reboot fallback")
    if "Rebooting (Safe)." in output:
      print("Orderly manager reboot requested.", flush=True)
      return
    if state.get("done"):
      raise RuntimeError("Command finished without confirming an orderly reboot")
    sleep(.5)
  raise RuntimeError("Update status timed out; check the agent log before retrying")


def main():
  if len(sys.argv) == 3 and sys.argv[1] == "--install-alias":
    install_alias(sys.argv[2])
    return
  if len(sys.argv) != 1:
    raise SystemExit("Usage: terminal_update.py [--install-alias PATH]")
  try:
    update()
  except Exception as error:
    print(f"gi: {error}. No direct reboot command was sent.", file=sys.stderr)
    raise SystemExit(1)


if __name__ == "__main__":
  main()
