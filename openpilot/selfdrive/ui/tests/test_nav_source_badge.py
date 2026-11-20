import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock


class TestNavSourceBadge(unittest.TestCase):
  def draw(self, source, connected=True, age=0.1, highway=False):
    path = Path(__file__).resolve().parents[1] / 'mici/onroad/hud_renderer.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'HudRenderer')
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_draw_navigation_status']
    cls.bases = []
    class SM(dict):
      valid = {'kisaNavigation': True}
      alive = {'kisaNavigation': True}
      recv_frame = {'kisaNavigation': 20}
      recv_time = {'kisaNavigation': 100.-age}
    sm = SM(kisaNavigation=NS(naviConnected=connected, naviHighway=highway, naviSource=source))
    ray = MagicMock()
    ray.Vector2 = lambda x, y: NS(x=x, y=y)
    state = NS(sm=sm, started=True, started_frame=10)
    env = dict(rl=ray, ui_state=state, time=NS(monotonic=lambda: 100.), HEADER_TOP=4,
               measure_text_cached=lambda font, text, size: NS(x=len(text)*size*.6))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), env)
    hud = env['HudRenderer']()
    hud._can_draw_top_icons = True
    hud._font_bold = None
    hud._draw_navigation_status(NS(x=0, y=0, width=536))
    return ray

  def test_selected_provider_is_a_circle(self):
    for source, letter in (('naver-map', 'N'), ('tmap', 'T')):
      ray = self.draw(source, highway=True)
      self.assertEqual([c.args[1] for c in ray.draw_text_ex.call_args_list], ['NAV', letter, 'HWay'])
      ray.draw_circle_v.assert_called_once()
      self.assertEqual(ray.draw_circle_v.call_args.args[1], 14)

  def test_stale_disconnected_and_unknown_sources_have_no_badge(self):
    for source, connected, age in (('tmap', False, .1), ('naver-map', True, 2.), ('legacy', True, .1), ('', True, .1)):
      ray = self.draw(source, connected, age)
      ray.draw_circle_v.assert_not_called()
      self.assertEqual([c.args[1] for c in ray.draw_text_ex.call_args_list], ['NAV'])


if __name__ == '__main__':
  unittest.main()
