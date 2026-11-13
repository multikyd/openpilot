import copy
import math
import unittest
from types import SimpleNamespace as NS
from openpilot.selfdrive.carrot.signal_follow import SignalFollowAssist, red_approach


def signal(distance=200.,speed=50/3.6):
  return dict(ts=100.,context_ts=100.,lat=37.+math.degrees(distance/6371000),lon=127.,distance=distance,
    navigation=dict(lat=37.,lon=127.,age=0.,speed=speed*3.6,bearing=0.,turn=-1,turn_distance=1000.,lane_points=[]),
    gps=dict(valid=True,age=0.,accuracy=1.,lat=37.,lon=127.,bearing=0.,speed=speed),
    movements={d:dict(state='RED_LIGHT_ON' if d=='straight' else 'UNAVAILABLE',remain=60.) for d in ['straight','left','right','uturn']})


def lead(distance=55.):
  return NS(present=True,radar=True,modelProb=.95,dRel=distance,vRel=-3.,aLeadK=-.8,vLead=10.)


class TestEarlierNavigationSignal(unittest.TestCase):
  def test_50_kph_verified_red_is_accepted_at_200m(self):
    self.assertEqual(red_approach(signal(),100.,50/3.6),(True,'red_ahead'))

  def test_outer_range_remains_bounded(self):
    self.assertFalse(red_approach(signal(230.),100.,50/3.6)[0])
    self.assertFalse(red_approach(signal(310.,20.),100.,20.)[0])

  def test_near_stop_is_left_to_existing_controller(self):
    self.assertFalse(red_approach(signal(15.),100.,50/3.6)[0])

  def test_green_and_wrong_movement_are_rejected(self):
    s=signal();s['movements']['straight']['state']='GREEN_LIGHT_ON'
    self.assertFalse(red_approach(s,100.,50/3.6)[0])
    s=signal();s['navigation'].update(turn=12,turn_distance=200.)
    self.assertFalse(red_approach(s,100.,50/3.6)[0])

  def test_stale_and_mismatched_positions_remain_rejected(self):
    self.assertFalse(red_approach(signal(),104.,50/3.6)[0])
    s=signal();s['gps']['lon']+=.001
    self.assertFalse(red_approach(s,100.,50/3.6)[0])

  def test_coast_starts_before_half_second_and_ramps(self):
    h=SignalFollowAssist();values=[]
    for i in range(8):
      h.update(signal(),100.,.05,50/3.6,lead(),True)
      values.append(h.strength)
    self.assertGreater(values[-1],0.)
    self.assertLess(values[-1],.3)
    self.assertTrue(all(0<=b-a<=.04+1e-9 for a,b in zip(values,values[1:])))

  def test_farther_slowing_lead_builds_bounded_gap(self):
    h=SignalFollowAssist()
    for _ in range(40):h.update(signal(),100.,.05,50/3.6,lead(),True)
    self.assertGreater(h.gap_extra,0.)
    self.assertLessEqual(h.gap_extra,.25)
    self.assertEqual(h.cruise_accel(1.5),0.)

  def test_no_lead_suppresses_acceleration_but_adds_no_gap(self):
    h=SignalFollowAssist();l=lead();l.present=False
    for _ in range(40):h.update(signal(),100.,.05,50/3.6,l,True)
    self.assertEqual(h.gap_extra,0.)
    self.assertEqual(h.cruise_accel(1.5),0.)
    self.assertEqual(h.cruise_accel(-.5),-.5)

  def test_driver_override_and_low_speed_reset_assistance(self):
    for override,speed in [(True,50/3.6),(False,1.)]:
      h=SignalFollowAssist()
      for _ in range(40):h.update(signal(),100.,.05,50/3.6,lead(),True)
      h.update(signal(),100.,.05,speed,lead(),True,override)
      self.assertEqual((h.strength,h.gap_extra),(0.,0.))
