import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

spec=importlib.util.spec_from_file_location('cushion_helper',Path(__file__).parents[1]/'lib/final_stop_cushion.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def lead(**kw):
  v=dict(present=True,radar=True,radarTrackId=40,modelProb=.99,dPath=0.,dRel=2.4,vRel=-.25,vLead=0.,aLeadK=0.)
  v.update(kw)
  return NS(**v)


class TestFinalStopCushion(unittest.TestCase):
  def setUp(self):
    self.h=m.FinalStopCushion();self.t=10.;self.lead=lead()
    self.x=dict(lead=self.lead,other=lead(present=False),speed=.25,standstill=False,
                command=-.7,previous=-.7,enabled=True,stopping=False,dt=.01)

  def tick(self,count=1,**kw):
    self.x.update(kw)
    for _ in range(count):
      self.t+=.05
      value=self.h.update(stamp=self.t,now=self.t,**self.x)
    return value

  def arm(self):self.tick(15)

  def test_only_final_moving_stopping_phase_gets_small_negative_release(self):
    self.arm()
    self.assertEqual(self.tick(),-.7)
    previous=-.7
    for _ in range(80):
      value=self.tick(stopping=True,previous=previous)
      self.assertLessEqual(value-previous,.0060001)
      self.assertLessEqual(value,-.4)
      previous=value
    self.assertAlmostEqual(value,-.4)

  def test_standstill_restores_native_hold_and_cannot_reapply_on_jitter(self):
    self.arm();self.assertGreater(self.tick(stopping=True),-.7)
    self.assertEqual(self.tick(speed=0.,standstill=True,command=-.8,previous=-.6),-.8)
    self.assertEqual(self.tick(speed=.1,standstill=False),-.8)

  def test_early_standstill_flag_does_not_end_cushion_while_wheels_still_move(self):
    self.arm()
    self.assertGreater(self.tick(stopping=True),-.7)
    value=self.tick(speed=.09,standstill=True,previous=-.5,command=-.508)
    self.assertGreater(value,-.508)
    self.assertFalse(self.h.finished)
    self.assertEqual(self.tick(speed=.01,standstill=True,command=-.6),-.6)
    self.assertTrue(self.h.finished)

  def test_close_gap_or_fast_closing_and_strong_request_keep_braking(self):
    for change in ('gap','closing','strong','speed'):
      self.setUp();self.arm()
      if change=='gap':self.lead.dRel=1.91
      elif change=='closing':self.lead.vRel=-.7
      elif change=='strong':self.x.update(command=-2.,previous=-2.)
      else:self.x['speed']=.5
      self.assertEqual(self.tick(stopping=True),self.x['command'])

  def test_stale_missing_changed_target_and_other_car_do_not_release(self):
    for change in ('stale','missing','identity','other','disabled'):
      self.setUp();self.arm()
      if change=='stale':
        self.assertEqual(self.h.update(stamp=self.t,now=self.t+.3,**dict(self.x,stopping=True)),-.7)
        continue
      if change=='missing':self.lead.present=False
      elif change=='identity':self.lead.radarTrackId=99
      elif change=='other':self.x['other']=lead(radarTrackId=99)
      else:self.x['enabled']=False
      self.assertEqual(self.tick(stopping=True),-.7)

  def test_duplicate_samples_cannot_build_confirmation(self):
    for _ in range(1000):
      value=self.h.update(stamp=10.,now=10.,**dict(self.x,stopping=True))
    self.assertEqual(value,-.7)

  def test_final_taper_does_not_change_lead_distance_or_stop_target(self):
    self.arm();distance=self.lead.dRel
    self.assertGreater(self.tick(stopping=True),-.7)
    self.assertEqual(self.lead.dRel,distance)


if __name__=='__main__':unittest.main()
