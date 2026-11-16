import ast
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

class OnroadUpdatePolicyTest(unittest.TestCase):
  def policy(self, offroad, current='1', target='2'):
    path = Path(__file__).with_name('updated.py')
    tree = ast.parse(path.read_text(encoding='utf-8'))
    nodes = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in ('OffroadOSUpdateRequired', 'handle_agnos_update')]
    flash = Mock()
    hardware = SimpleNamespace(get_os_version=lambda: current)
    env = {'Params': lambda: SimpleNamespace(get_bool=lambda key: offroad), 'HARDWARE': hardware,
           'run': lambda *args: target, 'OVERLAY_MERGED': '/staging/merged', 'set_consistent_flag': Mock(),
           'cloudlog': Mock(), 'os': SimpleNamespace(path=SimpleNamespace(join=os.path.join, isfile=lambda _: True))}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), env)
    agnos = SimpleNamespace(flash_agnos_update=flash, get_target_slot_number=lambda: 1)
    return env, flash, agnos

  def test_onroad_different_os_is_staged_without_flashing(self):
    env, flash, agnos = self.policy(False)
    with patch.dict(sys.modules, {'openpilot.common.hardware.comma.agnos': agnos}):
      with self.assertRaises(env['OffroadOSUpdateRequired']): env['handle_agnos_update']()
    flash.assert_not_called()

  def test_onroad_same_os_does_not_block_branch_code(self):
    env, flash, agnos = self.policy(False, target='1')
    with patch.dict(sys.modules, {'openpilot.common.hardware.comma.agnos': agnos}): env['handle_agnos_update']()
    flash.assert_not_called()

  def test_offroad_can_prepare_required_os_slot(self):
    env, flash, agnos = self.policy(True)
    with patch.dict(sys.modules, {'openpilot.common.hardware.comma.agnos': agnos}): env['handle_agnos_update']()
    self.assertEqual(flash.call_count, 1)

if __name__ == '__main__': unittest.main()
