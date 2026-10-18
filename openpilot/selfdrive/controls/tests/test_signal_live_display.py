import copy
import unittest

from openpilot.selfdrive.carrot.signal_memory import SignalMemory


class TestLiveSignalDisplay(unittest.TestCase):
  def setUp(self):
    self.now = 441.958268296
    self.wall = 1792295687.8644342
    self.gps = dict(valid=True,age=.025,lat=37.8054964,lon=128.8849167,
                    bearing=258.7128,accuracy=1.301)
    self.nav = dict(age=.714,lat=37.80545692424203,lon=128.88503386828066,
                    bearing=260.,speed=67.,turn=13,turn_distance=2350.,lane_points=[])
    self.signal = dict(ts=self.now,lat=37.80513466383355,lon=128.88378387544407,
                       distance=168.,movements={'straight':dict(state='GREEN_LIGHT_ON',remain=40.)})

  def show(self, age=0.):
    return SignalMemory(learn=False).update(self.signal,self.now+age,self.wall+age,
                                            self.gps,self.nav,{},18.768)

  def test_recorded_tmap_distance_differs_from_direct_gps(self):
    result=self.show()
    self.assertEqual(result['source'],'TMAP')
    self.assertEqual(result['straightRemain'],40.)
    self.assertEqual(result['distance'],168.)

  def test_no_learning_required_for_live_countdown(self):
    self.assertAlmostEqual(self.show(.8)['straightRemain'],39.2)

  def test_route_movement_uses_routed_distance(self):
    self.nav.update(turn=12,turn_distance=168.)
    self.signal['movements']['left']=dict(state='RED_LIGHT_ON',remain=25.)
    result=self.show()
    self.assertEqual(result['movement'],'left')
    self.assertEqual(result['leftRemain'],25.)

  def test_stale_signal_does_not_show_live_time(self):
    self.assertNotEqual(self.show(3.1)['source'],'TMAP')

  def test_wrong_approach_is_rejected(self):
    self.gps['bearing']=78.7
    self.nav['bearing']=78.7
    self.assertNotEqual(self.show()['source'],'TMAP')

  def test_unrelated_navigation_position_is_rejected(self):
    self.nav['lat']+=.01
    self.assertNotEqual(self.show()['source'],'TMAP')

  def test_implausible_distance_is_rejected(self):
    self.signal['distance']=600.
    self.assertNotEqual(self.show()['source'],'TMAP')

  def test_no_gps_is_rejected(self):
    self.gps['valid']=False
    self.assertEqual(self.show()['source'],'')

  def test_invalid_remaining_time_is_not_invented(self):
    self.signal['movements']['straight']['remain']=0.
    self.assertEqual(self.show()['straightRemain'],0.)


if __name__ == '__main__':
  unittest.main()
