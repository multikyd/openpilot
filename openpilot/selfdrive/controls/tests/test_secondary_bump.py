"""Secondary live bumps compete with a main camera without replacing its state."""
import ast
import importlib.util
from pathlib import Path
import math
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock

ROOT=Path(__file__).parents[2]
PATH=ROOT/'carrot/carrot_serv.py'
TREE=ast.parse(PATH.read_text(encoding='utf8'))
SERV=next(n for n in TREE.body if isinstance(n,ast.ClassDef) and any(isinstance(m,ast.FunctionDef) and m.name=='_update_sdi' for m in n.body))
UPDATE=next(n for n in SERV.body if isinstance(n,ast.FunctionDef) and n.name=='update_navi')
METHODS={'_update_sdi','_live_plus_bump','_plus_bump_speed','_ui_bump_notice','calculate_current_speed'}
body=[n for n in SERV.body if isinstance(n,ast.FunctionDef) and n.name in METHODS]
cls=ast.ClassDef(name='RealMethods',bases=[],keywords=[],body=body,decorator_list=[]);ast.fix_missing_locations(cls)
ENV={'math':math};exec(compile(ast.Module(body=[cls],type_ignores=[]),str(PATH),'exec'),ENV)


def instance():
  h=ENV['RealMethods']()
  h.nSdiType=0;h.nSdiSpeedLimit=50;h.nSdiDist=398;h.nSdiBlockType=-1
  h.nSdiPlusType=22;h.nSdiPlusDist=16;h.autoNaviSpeedCtrlMode=2;h.autoNaviSpeedSafetyFactor=1.01
  h.roadcate=3;h.autoNaviSpeedBumpSpeed=30.;h.autoNaviSpeedBumpTime=4.;h.autoNaviSpeedDecelRate=1.2
  h.autoNaviSpeedCtrlEnd=9.;h.active_carrot=2;h.last_update_gps_time_navi=100.
  h.bump_display=None;h.bump_memory=NS(position=None);h.bump_passage=NS(matches=MagicMock(return_value=False),event=None)
  h.autoTurnControl=0;h.xTurnInfo=h.xTurnInfoNext=-1;h.xDistToTurn=h.xDistToTurnNext=1000
  h.update_auto_turn=lambda *a:(250.,'none',0,0)
  h._update_sdi()
  return h


start=next(i for i,n in enumerate(UPDATE.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='learned_bump_speed')
end=next(i for i,n in enumerate(UPDATE.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='speed_n_sources')
CAP_BLOCK=compile(ast.Module(body=UPDATE.body[start:end+1],type_ignores=[]),str(PATH),'exec')


def caps(h,now=100.,passed=False):
  env=dict(self=h,bump_now=now,v_ego=10.,v_ego_kph=36.,nav_bump_passed=passed,CS=NS(speedLimit=0,speedLimitDistance=0),
           sm=None,limit_speed=40.)
  exec(CAP_BLOCK,env)
  return min(env['speed_n_sources'],key=lambda item:item[0]),env


class TestSecondaryBump(unittest.TestCase):
  def test_recorded_camera_and_bump_case_now_selects_30(self):
    h=instance();best,env=caps(h)
    self.assertEqual(best,(30.,'bump'))
    self.assertEqual((h.xSpdType,h.xSpdDist,h.xSpdLimit),(0,398,50.5))
    self.assertEqual(h._ui_bump_notice(100.,10.),(30,16))
  def test_near_lower_speed_camera_keeps_its_cap(self):
    h=instance();h.nSdiSpeedLimit=20;h.nSdiDist=5;h._update_sdi()
    self.assertEqual(caps(h)[0],(20.2,'cam'))
  def test_no_bump_keeps_main_camera_and_road_behavior(self):
    h=instance();h.nSdiPlusType=-1
    self.assertEqual(caps(h)[0],(40.,'road'))
  def test_switching_learning_off_does_not_remove_live_cap(self):
    h=instance();h.bump_display=None
    self.assertEqual(h._plus_bump_speed(100.,10.),30.)
  def test_disabled_mode_and_highways_do_not_add_control_cap(self):
    for key,value in (('autoNaviSpeedCtrlMode',0),('autoNaviSpeedCtrlMode',1),('roadcate',0),('roadcate',1)):
      h=instance();setattr(h,key,value)
      self.assertEqual(h._plus_bump_speed(100.,10.),250.)
  def test_stale_future_and_inactive_data_are_rejected(self):
    for now in (99.9,103.01):
      self.assertIsNone(instance()._live_plus_bump(now,10.))
    h=instance();h.active_carrot=1
    self.assertIsNone(h._live_plus_bump(100.,10.))
  def test_bad_measurements_are_rejected(self):
    for distance in (0,-1,math.nan,math.inf):
      h=instance();h.nSdiPlusDist=distance
      self.assertIsNone(h._live_plus_bump(100.,10.))
    h=instance();h.autoNaviSpeedBumpSpeed=math.nan
    self.assertIsNone(h._live_plus_bump(100.,10.))
  def test_receipt_age_compensates_distance(self):
    h=instance();h.nSdiPlusDist=56
    self.assertAlmostEqual(h._live_plus_bump(101.,10.)['distance'],46.)
    self.assertGreater(h._plus_bump_speed(100.,10.),h._plus_bump_speed(101.,10.))
  def test_hold_at_crossing_then_release_after_allowance(self):
    h=instance();h.nSdiPlusDist=10
    self.assertEqual(h._plus_bump_speed(101.,10.),30.)
    self.assertEqual(h._plus_bump_speed(101.5,10.),30.)
    self.assertEqual(h._plus_bump_speed(101.9,10.),250.)
  def test_matching_learned_target_is_retained(self):
    h=instance();h.bump_display={'target':20.,'distance':15.}
    self.assertEqual(h._plus_bump_speed(100.,10.),20.)
    h.bump_display={'target':35.,'distance':15.}
    self.assertEqual(h._plus_bump_speed(100.,10.),35.)
    h.bump_display={'target':20.,'distance':100.}
    self.assertEqual(h._plus_bump_speed(100.,10.),30.)
  def test_main_bump_still_uses_existing_target(self):
    h=instance();h.nSdiType=22;h.nSdiPlusType=-1;h.nSdiDist=16;h._update_sdi()
    self.assertEqual(caps(h)[0],(30.,'bump'))
  def test_sensor_passage_releases_secondary_without_dropping_camera(self):
    h=instance()
    spec=importlib.util.spec_from_file_location('openpilot.selfdrive.carrot.bump_learning',ROOT/'carrot/bump_learning.py')
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    h.bump_memory.position=(100.,37.,127.,0.);h.bump_passage.matches.return_value=True
    self.assertEqual(h._plus_bump_speed(100.,10.),250.)
    self.assertEqual(h._ui_bump_notice(100.,10.),(0,0))
    self.assertEqual(caps(h)[0],(40.,'road'))
    self.assertEqual(h.xSpdType,0)
    h.nSdiSpeedLimit=20;h.nSdiDist=5;h._update_sdi()
    self.assertEqual(caps(h)[0],(20.2,'cam'))
  def test_secondary_passage_input_does_not_replace_primary_nav_distance(self):
    nodes=[]
    for n in ast.walk(UPDATE):
      if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id in ('nav_distance','passage_distance'):
        nodes.append(n)
    nodes.sort(key=lambda n:n.lineno)
    env=dict(self=instance(),plus_distance=16.)
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(PATH),'exec'),env)
    self.assertIsNone(env['nav_distance']);self.assertEqual(env['passage_distance'],16.)


if __name__=='__main__':unittest.main()
