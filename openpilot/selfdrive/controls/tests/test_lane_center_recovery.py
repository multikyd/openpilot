import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np
s=importlib.util.spec_from_file_location('guard',Path(__file__).parents[1]/'lib/narrow_road_bias.py')
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
class TestCenterRecovery(unittest.TestCase):
  def setUp(self):
    x=np.linspace(0.,30.,33);self.h=m.LaneCenterRecovery()
    self.md=NS(meta=NS(desire=0,laneChangeState=0),laneLines=[NS(x=x,y=np.zeros(33)),NS(x=x,y=np.full(33,-1.7)),NS(x=x,y=np.full(33,1.3))],laneLineProbs=[0.,.95,.95],laneLineStds=[0.,.1,.1])
    self.cs=NS(vEgo=12.,leftBlinker=False,rightBlinker=False,steeringPressed=False)
    self.path=np.column_stack([x,np.zeros(33),np.zeros(33)])
  def tick(self,**kw):
    a=dict(model=self.md,cs=self.cs,path=self.path,curvature=0.,preference=-.06,enabled=True);a.update(kw)
    return [self.h.update(**a) for _ in range(120)]
  def test_confirmation_rate_and_bound(self):
    a=self.tick();self.assertTrue(all(v==0 for v in a[:9]));self.assertGreaterEqual(min(a),-.06);self.assertLess(a[-1],0.);self.assertLessEqual(max(abs(np.diff(a))),.000501)
  def test_centered(self):
    self.md.laneLines[1].y+=.2;self.md.laneLines[2].y+=.2;self.assertEqual(self.tick()[-1],0.)
  def test_positive_preference(self):self.assertEqual(self.tick(preference=.04)[-1],0.)
  def test_driver(self):
    self.tick();self.cs.steeringPressed=True;self.assertEqual(self.tick()[-1],0.)
  def test_blinker(self):
    self.cs.rightBlinker=True;self.assertEqual(self.tick()[-1],0.)
  def test_uncertain(self):
    self.md.laneLineStds[2]=float('nan');self.assertEqual(self.tick()[-1],0.)
  def test_curve_and_speed(self):
    self.assertEqual(self.tick(curvature=.006)[-1],0.);self.cs.vEgo=25.;self.assertEqual(self.tick()[-1],0.)
  def test_left_margin(self):
    self.path[:,1]=-.5;self.assertEqual(self.tick()[-1],0.)
  def test_disabled(self):
    self.tick();self.assertEqual(self.tick(enabled=False)[-1],0.)
