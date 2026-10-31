"""Packet selection regressions without Linux sockets or vehicle hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

source = Path(__file__).resolve().parents[2] / 'carrot' / 'carrot_serv.py'
tree = ast.parse(source.read_text(encoding='utf8'))
method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_update_sdi')
namespace = {}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
select = namespace['_update_sdi']


def packet(**changes):
  values = dict(nSdiType=4, nSdiSpeedLimit=80, nSdiDist=0, nSdiBlockType=2,
                nSdiBlockSpeed=80, nSdiBlockDist=1200, nSdiPlusType=-1, nSdiPlusDist=0,
                nSdiPlusBlockType=-1, nSdiPlusBlockSpeed=0, nSdiPlusBlockDist=0,
                roadcate=0, autoNaviSpeedCtrlMode=2, autoNaviSpeedSafetyFactor=1.,
                autoNaviSpeedBumpSpeed=30)
  values.update(changes)
  return SimpleNamespace(**values)


class TestSectionSpeed(unittest.TestCase):
  def test_end_camera_without_primary_speed_keeps_section(self):
    s = packet(nSdiType=3, nSdiSpeedLimit=0, nSdiDist=350, nSdiBlockType=3, nSdiBlockDist=350)
    select(s)
    self.assertEqual((s.xSpdType, s.xSpdLimit, s.xSpdDist), (4, 80, 350))

  def test_other_sign_does_not_release_section(self):
    s = packet(nSdiType=17, nSdiSpeedLimit=0)
    select(s)
    self.assertEqual((s.xSpdType, s.xSpdLimit), (4, 80))

  def test_lower_camera_and_section_cap_are_both_retained(self):
    s = packet(nSdiType=1, nSdiSpeedLimit=60, nSdiDist=400)
    select(s)
    self.assertEqual((s.xSpdType, s.xSpdLimit, s.xSpdDist), (1, 60, 400))
    self.assertEqual(s.section_speed_limit, 80)

  def test_secondary_section_is_independent(self):
    s = packet(nSdiType=17, nSdiSpeedLimit=0, nSdiBlockType=0,
               nSdiPlusBlockType=3, nSdiPlusBlockSpeed=80, nSdiPlusBlockDist=75)
    select(s)
    self.assertEqual((s.xSpdType, s.xSpdLimit, s.xSpdDist), (4, 80, 75))

  def test_start_camera_keeps_distance_based_approach(self):
    s = packet(nSdiType=2, nSdiDist=400, nSdiBlockType=1)
    select(s)
    self.assertEqual((s.xSpdType, s.xSpdDist), (2, 400))

  def test_missing_completed_and_disabled_sections_do_not_latch(self):
    for change in [dict(nSdiBlockType=0), dict(nSdiBlockDist=0),
                   dict(nSdiBlockSpeed=0), dict(autoNaviSpeedCtrlMode=0)]:
      s = packet(nSdiType=17, nSdiSpeedLimit=0, **change)
      select(s)
      self.assertEqual((s.section_speed_limit, s.xSpdLimit), (0, 0))


if __name__ == '__main__':
  unittest.main()
