import unittest
from collections import deque
from openpilot.selfdrive.carrot.bump_learning import BumpPassage, offset

class TestFrontLift(unittest.TestCase):
  def drive(self, pitch=True, vertical=True, sign=1., stale=False, accel=0., strong=False, distance0=10., enabled=True, reverse=False):
    detector=BumpPassage();samples=deque(maxlen=1600);released=[];events=[]
    for i in range(171):
      t=i*.01;now=100.+t
      amplitude=(2. if strong else .65) if vertical and 1.<=t<=1.25 else .03
      samples.append((now,amplitude))
      if i%10:continue
      lat,lon=offset(37.,127.,0.,5.*t)
      angle=sign*min(.014,max(0.,t-1.)*.07)
      rate=sign*.07 if 1.<t<=1.2 else 0.
      data=(now-(1. if stale else 0.),angle,rate,accel) if pitch else None
      if reverse:data=(now,angle,-abs(rate),accel)
      value=detector.update(now,18.,distance0-5.*t,(now,lat,lon,0.),samples,now,enabled,pitch=data)
      if value:released.append(t)
      if detector.event:events.append(detector.event)
    return detector,released,events
  def test_rising_front_and_weak_vertical_motion_release(self):
    d,t,events=self.drive()
    self.assertTrue(t)
    self.assertLessEqual(t[0],1.3)
    self.assertFalse(events)  # no two-axle learning evidence
  def test_pitch_alone_does_not_release(self):
    self.assertFalse(self.drive(vertical=False)[1])
  def test_weak_vertical_motion_alone_does_not_release(self):
    self.assertFalse(self.drive(pitch=False)[1])
  def test_nose_down_does_not_trigger_front_release(self):
    self.assertFalse(self.drive(sign=-1.)[1])
  def test_stale_pitch_does_not_release(self):
    self.assertFalse(self.drive(stale=True)[1])
  def test_throttle_pitch_does_not_release(self):
    self.assertFalse(self.drive(accel=1.)[1])
  def test_too_far_or_disabled_keeps_limit(self):
    self.assertFalse(self.drive(distance0=30.)[1])
    self.assertFalse(self.drive(enabled=False)[1])
  def test_existing_strong_impact_works_without_pose(self):
    self.assertTrue(self.drive(pitch=False,strong=True)[1])
  def test_first_lift_does_not_match_next_bump(self):
    d,t,_=self.drive()
    self.assertIsNotNone(d.released)
    anchor=d.released
    self.assertFalse(d.matches(101.7,(*offset(anchor['lat'],anchor['lon'],0.,10.),0.)))
  def test_opposite_pitch_rate_does_not_release(self):
    self.assertFalse(self.drive(reverse=True)[1])
