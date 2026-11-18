import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[4]
PATH=ROOT/'opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py'
TREE=ast.parse(PATH.read_text(encoding='utf-8'))
FUNCTION=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='ccnc_lane_color')
ENV={};exec(compile(ast.Module(body=[FUNCTION],type_ignores=[]),str(PATH),'exec'),ENV)
color=ENV['ccnc_lane_color']


class TestCcncLaneHud(unittest.TestCase):
  def test_fresh_left_and_right_ready_are_green(self):
    self.assertEqual(color(11,True,False,True,True,0),6)
    self.assertEqual(color(14,True,False,True,True,0),6)

  def test_unready_or_stale_falls_back_to_painted_color(self):
    self.assertEqual(color(11,True,False,False,True,0),2)
    self.assertEqual(color(11,True,False,True,False,0),2)

  def test_yellow_edge_hidden_and_departure_have_priority(self):
    self.assertEqual(color(21,True,False,True,True,0),4)
    self.assertEqual(color(13,True,False,True,True,0),2)
    self.assertEqual(color(11,False,False,True,True,0),0)
    self.assertEqual(color(11,True,True,True,True,0),4)
    self.assertEqual(color(11,True,True,True,True,50),1)

  def test_actual_ccnc_assignments_use_independent_ready_sides(self):
    fn=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='create_ccnc_messages')
    assignments=[]
    for n in ast.walk(fn):
      if isinstance(n,ast.Assign) and any(ast.unparse(t) in
        ('ready_valid', "values['LANELINE_LEFT']", "values['LANELINE_RIGHT']") for t in n.targets):assignments.append(n)
    self.assertEqual(len(assignments),3)
    env=dict(ENV,CS=NS(out=NS(oemLaneChangeStateValid=True,leftOemLaneChangeReady=True,
             rightOemLaneChangeReady=False,leftLaneLine=11,rightLaneLine=11)),
             hud_control=NS(leftLaneVisible=True,rightLaneVisible=True,leftLaneDepart=False,rightLaneDepart=False),
             values={},frame=0)
    exec(compile(ast.Module(body=assignments,type_ignores=[]),str(PATH),'exec'),env)
    self.assertEqual(env['values'],{'LANELINE_LEFT':6,'LANELINE_RIGHT':2})


if __name__=='__main__':unittest.main()
