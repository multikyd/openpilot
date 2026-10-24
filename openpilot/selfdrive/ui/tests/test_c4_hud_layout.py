"""Geometry and actual drawing-path checks without a GPU."""
import ast
import importlib.util
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("c4_layout_test", ROOT / "mici/onroad/hud_layout.py")
layout = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = layout
spec.loader.exec_module(layout)


def drawing_class(state, ray):
  tree = ast.parse((ROOT / "mici/onroad/hud_renderer.py").read_text(encoding="utf-8-sig"))
  methods = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "HudRenderer")
  wanted = {"_draw_lower_left", "_draw_car_stat"}
  methods.body = [n for n in methods.body if isinstance(n, ast.FunctionDef) and n.name in wanted]
  methods.bases = []
  methods.decorator_list = []
  namespace = dict(rl=ray, ui_state=state, time=time, KM_TO_MILE=.621371,
                   LEFT_CENTER=layout.LEFT_CENTER, lower_notice_box=layout.lower_notice_box,
                   left_car_box=layout.left_car_box, gap_bar_boxes=layout.gap_bar_boxes,
                   select_road_notice=layout.select_road_notice,
                   measure_text_cached=lambda f,t,s:NS(x=len(t)*s*.6,y=s))
  exec(compile(ast.Module(body=[methods],type_ignores=[]),str(ROOT / "mici/onroad/hud_renderer.py"),"exec"),namespace)
  return namespace["HudRenderer"]


class LayoutTest(unittest.TestCase):
  def test_shared_center_and_clearance(self):
    for x,y in ((0,0),(15,25)):
      car=layout.left_car_box(x,y)
      notice=layout.lower_notice_box(x,y,240)
      self.assertEqual(car[0]+car[2]/2,x+46)
      self.assertEqual(notice[0]+notice[2]/2,x+46)
      self.assertLess(car[1]+car[3],notice[1])
      self.assertLessEqual(notice[1]+notice[3],y+240)
      for bx,by,bw,bh in layout.gap_bar_boxes(x,y):
        self.assertEqual(bx+bw/2,x+46)
        self.assertGreater(by,car[1]);self.assertLess(by+bh,car[1]+car[3])
  def test_signal_header_and_inset(self):
    for width in (440,476,536):
      x,y,w,h=layout.signal_box(0,0,width)
      self.assertEqual(y,layout.HEADER_TOP)
      self.assertAlmostEqual(x+w,width-20)
      self.assertGreaterEqual(x,width/2+40+8)
    self.assertIsNone(layout.signal_box(0,0,215))
  def test_closer_notice_and_tie(self):
    for camera,bump,kind in ((200,100,"BUMP"),(100,200,"CAM"),(100,100,"BUMP")):
      self.assertEqual(layout.select_road_notice(1,50,50,camera,30,bump).kind,kind)
  def test_invalid_notice_and_passed_camera(self):
    self.assertIsNone(layout.select_road_notice(1,50,50,0,0,0))
    self.assertIsNone(layout.select_road_notice(1,50,50,float("nan"),30,float("inf")))
    self.assertEqual(layout.select_road_notice(22,50,30,80,0,0).kind,"BUMP")


class DrawingTest(unittest.TestCase):
  def setUp(self):
    sm=NS(valid={"carrotMan":True},alive={"carrotMan":True},recv_time={"carrotMan":time.monotonic()})
    self.state=NS(sm=sm,xSpdType=1,xSpdLimitOrg=50,xSpdLimit=50,xSpdDist=200,
                  bumpTarget=30,bumpDistance=100,is_metric=True,
                  tpmsPressureFl=35,tpmsPressureFr=35,tpmsPressureRl=35,tpmsPressureRr=35,
                  tpmsUnit=0,brakeLights=True,autoHold=True,cruise_gap=4)
    ray=self.ray=MagicMock()
    ray.Rectangle=lambda x,y,w,h:NS(x=x,y=y,width=w,height=h)
    ray.Vector2=lambda x,y:NS(x=x,y=y)
    ray.Color=lambda *v:v
    self.hud=drawing_class(self.state,ray)()
    self.hud._show_wheel_critical=False;self.hud._can_draw_top_icons=True
    self.hud.setspeed_changed=False;self.hud._font_bold=None
    self.hud.img_speed_bump=NS(width=32,height=22)
    self.hud.img_car=NS(width=40,height=70)
    self.hud.img_car_width=40;self.hud.img_car_height=70
    self.hud._draw_steering_wheel=MagicMock()
    self.rect=NS(x=0,y=0,width=476,height=240)
  def test_bump_then_camera_switch(self):
    self.hud._draw_lower_left(self.rect)
    self.assertEqual(self.ray.draw_text_ex.call_args_list[0].args[1],"BUMP")
    self.state.bumpTarget=0
    self.ray.draw_text_ex.reset_mock()
    self.hud._draw_lower_left(self.rect)
    self.assertEqual(self.ray.draw_text_ex.call_args_list[0].args[1],"CAM")
    self.hud._draw_steering_wheel.assert_not_called()
  def test_stale_and_critical_use_wheel(self):
    self.state.sm.recv_time["carrotMan"]-=2
    self.hud._draw_lower_left(self.rect)
    self.assertEqual(self.hud._draw_steering_wheel.call_count,1)
    self.state.sm.recv_time["carrotMan"]=time.monotonic()
    self.hud._show_wheel_critical=True
    self.hud._draw_lower_left(self.rect)
    self.assertEqual(self.hud._draw_steering_wheel.call_count,2)
    self.ray.draw_text_ex.assert_not_called()
  def test_four_slots_fixed_and_own_brake_lamps_preserved(self):
    boxes=[]
    for gap in (1,4):
      self.ray.draw_rectangle_rounded.reset_mock()
      self.state.cruise_gap=gap
      self.hud._draw_car_stat(self.rect)
      calls=self.ray.draw_rectangle_rounded.call_args_list
      self.assertEqual(len(calls),6)
      rects=[c.args[0] for c in calls]
      self.assertEqual([r.y for r in rects[-4:]],[100,104,108,112])
      self.assertEqual((rects[0].x+rects[1].x+8)/2,46)
      boxes.append([(r.x,r.y,r.width,r.height) for r in rects[-4:]])
    self.assertEqual(*boxes)


if __name__ == "__main__":
  unittest.main()
