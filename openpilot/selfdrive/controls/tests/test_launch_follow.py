import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

spec=importlib.util.spec_from_file_location('launch_helper',Path(__file__).parents[1]/'lib/launch_follow.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def lead(**kw):
  v=dict(present=True,radar=True,radarTrackId=10,modelProb=.99,dPath=0.,dRel=3.,vRel=0.,vLead=0.,aLeadK=0.)
  v.update(kw)
  return NS(**v)


class TestLaunchFollow(unittest.TestCase):
  def setUp(self):
    self.h=m.LaunchFollow();self.t=10.;self.lead=lead()
    self.x=dict(enabled=True,can_accelerate=False,speed=0.,target_speed=15.,normal_max=.8,
                lead=self.lead,other=lead(present=False),follow=1.2)

  def tick(self,count=1,**kw):
    self.x.update(kw)
    for _ in range(count):
      self.t+=.05
      self.lead.dRel+=self.lead.vRel*.05
      value=self.h.update(now=self.t,stamp=self.t,**self.x)
    return value

  def arm(self):
    self.tick(15)
    self.assertTrue(self.h.armed)

  def depart(self,count=30):
    self.lead.vLead=self.lead.vRel=2.
    return self.tick(count,can_accelerate=True)

  def test_only_same_stopped_lead_can_enable_temporary_headroom(self):
    self.assertEqual(self.depart(),.8)
    self.setUp();self.arm()
    self.assertGreater(self.depart(),.8)
    self.assertLessEqual(self.depart(),1.15)

  def test_rise_is_smooth_and_never_exceeds_normal_plus_point35(self):
    self.arm();self.lead.vLead=self.lead.vRel=2.
    previous=.8
    for _ in range(80):
      value=self.tick(can_accelerate=True)
      self.assertLessEqual(value-previous,.2*.05+1e-9)
      self.assertLessEqual(value,1.15+1e-9)
      previous=value

  def test_eligibility_loss_or_changed_target_removes_extra(self):
    for action in ('disabled','identity','brake','stale'):
      self.setUp();self.arm();self.depart()
      if action=='disabled': result=self.tick(enabled=False)
      elif action=='identity':
        self.lead.radarTrackId=20;result=self.tick()
      elif action=='brake':
        self.lead.aLeadK=-1.;result=self.tick()
      else:
        result=self.h.update(now=self.t+.3,stamp=self.t,**self.x)
      self.assertEqual(result,.8,action)

  def test_no_boost_against_navigation_target_or_hold(self):
    self.arm();self.depart()
    self.assertEqual(self.tick(can_accelerate=False),.8)
    self.assertEqual(self.tick(50,can_accelerate=True,target_speed=0.),.8)

  def test_timeout_and_regained_gap_fade_to_normal(self):
    self.arm();self.depart()
    self.assertEqual(self.tick(200),.8)
    self.setUp();self.arm();self.depart()
    self.lead.vLead=self.lead.vRel=0.
    self.assertEqual(self.tick(30),.8)

  def test_no_boost_after_rolling_start_or_duplicate_stopped_samples(self):
    for _ in range(200):
      self.h.update(now=10.,stamp=10.,**self.x)
    self.assertFalse(self.h.armed)
    self.assertEqual(self.depart(),.8)

  def test_second_obstacle_veto_and_overlapping_vision_duplicate(self):
    self.arm();self.depart()
    self.assertEqual(self.tick(other=lead(radarTrackId=99,dRel=self.lead.dRel)),.8)
    self.setUp();self.arm()
    duplicate=lead(radar=False,radarTrackId=-1,dRel=3.1,vRel=0.)
    self.assertTrue(self.h.update(now=self.t,stamp=self.t,**dict(self.x,other=duplicate))==.8)
    self.assertTrue(self.h.armed)

  def test_same_vehicle_can_rearm_after_the_next_stop(self):
    self.arm();self.depart()
    self.lead.vLead=self.lead.vRel=0.
    self.tick(16,can_accelerate=False)
    self.assertTrue(self.h.armed)
    self.assertIsNone(self.h.started)
    self.assertGreater(self.depart(),.8)

  def test_absolute_maximum_does_not_raise_existing_stronger_profile(self):
    self.arm();self.depart()
    self.assertEqual(self.tick(normal_max=2.),2.)
    self.assertEqual(self.tick(speed=60./3.6),2.)


if __name__=='__main__':unittest.main()
