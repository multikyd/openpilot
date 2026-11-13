"""Exercise the actual HUD selection and drawing without loading a GPU."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]/'mici/onroad'
spec = importlib.util.spec_from_file_location('advice_layout', ROOT/'hud_layout.py')
layout = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = layout
spec.loader.exec_module(layout)


class TestAdviceHUD(unittest.TestCase):
  def select(self, highway=True, recommended=True, plan_age=.05, nav_age=.05, plan_valid=True, old_schema=False):
    class SM(dict):
      valid = {'carrotMan': True, 'longitudinalPlan': plan_valid}
      alive = {'carrotMan': True, 'longitudinalPlan': True}
      recv_time = {'carrotMan': 10.-nav_age, 'longitudinalPlan': 10.-plan_age}
    sm = SM(carrotMan=NS(naviHighway=highway, signalDisplay='signal'),
            longitudinalPlan=NS() if old_schema else NS(overtakeAdvised=recommended))
    tree = ast.parse((ROOT/'hud_renderer.py').read_text(encoding='utf8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'HudRenderer')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_render')
    block = next(n for n in method.body if isinstance(n, ast.If) and
                 any(isinstance(c, ast.Call) and isinstance(c.func, ast.Name) and c.func.id == 'draw_overtake_advice'
                     for c in ast.walk(n)))
    advice, signal = MagicMock(), MagicMock()
    env = dict(self=NS(_can_draw_top_icons=True, _font_bold='font'), rect='rect',
               ui_state=NS(sm=sm, started=True), time=NS(monotonic=lambda:10.),
               draw_overtake_advice=advice, draw_signal_panel=signal)
    exec(compile(ast.Module(body=[block], type_ignores=[]), 'actual_hud_selection', 'exec'), env)
    return advice.call_count, signal.call_count

  def test_only_recommendation_on_highway(self):
    self.assertEqual(self.select(), (1, 0))
    self.assertEqual(self.select(recommended=False), (0, 0))

  def test_regular_road_keeps_signal(self):
    self.assertEqual(self.select(highway=False), (0, 1))

  def test_stale_invalid_or_old_schema_never_displays_advice(self):
    for kwargs in (dict(plan_age=1.), dict(nav_age=2.), dict(plan_valid=False),
                   dict(plan_age=-1.), dict(old_schema=True)):
      self.assertEqual(self.select(**kwargs), (0, 0))

  def test_actual_drawing_fits_existing_header_in_all_supported_widths(self):
    tree = ast.parse((ROOT/'signal_renderer.py').read_text(encoding='utf8'))
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'draw_overtake_advice')
    ray = MagicMock()
    ray.Rectangle = lambda x,y,w,h:NS(x=x,y=y,width=w,height=h)
    ray.Vector2 = lambda x,y:NS(x=x,y=y)
    measure = lambda f,t,s:NS(x=len(t)*s*.7, y=s)
    env = dict(rl=ray, signal_box=layout.signal_box, measure_text_cached=measure)
    exec(compile(ast.Module(body=[function],type_ignores=[]),'actual_advice_drawing','exec'),env)
    for width in (440,476,536):
      ray.reset_mock()
      env['draw_overtake_advice'](NS(x=0,y=0,width=width,height=240),'font')
      font, label, position, size, _, color = ray.draw_text_ex.call_args.args
      x,y,w,h = layout.signal_box(0,0,width)
      self.assertEqual(label,'PASS ADVISED')
      self.assertGreaterEqual(position.x,x)
      self.assertLessEqual(position.x+measure(font,label,size).x,x+w)
      self.assertGreaterEqual(position.y,y)
      self.assertLessEqual(position.y+size,y+h)
    ray.reset_mock()
    env['draw_overtake_advice'](NS(x=0,y=0,width=215,height=240),'font')
    ray.draw_text_ex.assert_not_called()


if __name__ == '__main__':
  unittest.main()
