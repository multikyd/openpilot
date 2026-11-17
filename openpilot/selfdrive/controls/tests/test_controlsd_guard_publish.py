"""Check the actual diagnostic condition without a vehicle/native runtime."""
import ast
import builtins
from pathlib import Path
import symtable
from types import SimpleNamespace as NS
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "controlsd.py"


class TestGuardPublish(unittest.TestCase):
  def setUp(self):
    self.source = SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(self.source)
    self.controls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Controls")
    self.methods = {n.name: n for n in self.controls.body if isinstance(n, ast.FunctionDef)}

  def draw_diagnostic(self, used, lat_active=True, speed=8.):
    node = next(n for n in self.methods["publish"].body if isinstance(n, ast.If) and
                "narrow_guard_used" in ast.unparse(n.test))
    cs = NS(debugMsg1="base")
    env = dict(self=NS(narrow_guard_used=used, model_narrow_road_guard=NS(accel=.12, reason="narrow")),
               CC=NS(latActive=lat_active), CS=NS(vEgo=speed), cs=cs)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), env)
    return cs.debugMsg1

  def test_publish_has_no_undefined_global_names(self):
    module = symtable.symtable(self.source, str(SOURCE), "exec")
    cls = next(t for t in module.get_children() if t.get_name() == "Controls")
    publish = next(t for t in cls.get_children() if t.get_name() == "publish")
    defined = {s.get_name() for s in module.get_symbols() if s.is_assigned() or s.is_imported()}
    undefined = {s.get_name() for s in publish.get_symbols() if s.is_global() and s.is_referenced()
                 and s.get_name() not in defined and not hasattr(builtins, s.get_name())}
    self.assertEqual(undefined, set())

  def test_shared_flag_initialized_and_reset_each_control_cycle(self):
    for method in ("__init__", "state_control"):
      assignments = [n for n in self.methods[method].body if isinstance(n, ast.Assign)
                     and any(ast.unparse(t) == "self.narrow_guard_used" for t in n.targets)]
      self.assertEqual(len(assignments), 1)
      self.assertIs(assignments[0].value.value, False)

  def test_model_mode_diagnostic_can_be_published(self):
    self.assertEqual(self.draw_diagnostic(True), "base NRG=+0.120:narrow")

  def test_lane_mode_disengaged_and_high_speed_skip_diagnostic(self):
    self.assertEqual(self.draw_diagnostic(False), "base")
    self.assertEqual(self.draw_diagnostic(True, lat_active=False), "base")
    self.assertEqual(self.draw_diagnostic(True, speed=20.), "base")


if __name__ == "__main__": unittest.main()
