import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("terminal_update", Path(__file__).resolve().parents[1] / "terminal_update.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TestTerminalUpdate(unittest.TestCase):
  def run_update(self, states, *, supported=True):
    calls = []
    def request(path, payload=None):
      calls.append((path, payload))
      if path == "/reboot/capabilities": return {"disengageBeforeReboot": supported}
      if path == "/cmd/run": return {"status": "started", "id": "test-id"}
      return states.pop(0)
    with contextlib.redirect_stdout(io.StringIO()):
      module.update(request=request, sleep=lambda _: None, clock=lambda: 0.)
    return calls

  def test_only_guarded_safe_update_is_requested(self):
    calls = self.run_update([{"output": "Disengage requested", "done": False},
                             {"output": "Disengage confirmed\nRebooting (Safe).", "done": False}])
    self.assertEqual(calls[1], ("/cmd/run", {"id": "git_pull_reboot"}))

  def test_missing_capability_does_not_send_update(self):
    calls = []
    def request(path, payload=None):
      calls.append(path)
      return {"disengageBeforeReboot": False}
    with self.assertRaises(RuntimeError): module.update(request=request)
    self.assertEqual(calls, ["/reboot/capabilities"])

  def test_failed_pull_or_disengagement_is_not_success(self):
    with self.assertRaisesRegex(RuntimeError, "stopped"):
      self.run_update([{"output": "Operation stopped: Command failed; reboot cancelled", "done": True}])

  def test_done_without_reboot_is_not_success(self):
    with self.assertRaisesRegex(RuntimeError, "without confirming"):
      self.run_update([{"output": "", "done": True}])

  def test_alias_replacement_preserves_other_settings_and_is_idempotent(self):
    with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as tmp:
      path = Path(tmp) / "bashrc"
      path.write_text("export OTHER=1\nalias gi='sudo reboot'\nalias ls='ls -l'\nalias gi='old'\n")
      module.install_alias(path)
      first = path.read_text()
      module.install_alias(path)
      self.assertEqual(first, path.read_text())
      self.assertEqual(first.count("alias gi="), 1)
      self.assertIn("export OTHER=1", first)
      self.assertIn("alias ls=", first)
      self.assertNotIn("sudo reboot", first)


if __name__ == "__main__": unittest.main()
