import unittest
from openpilot.selfdrive.carrot.signal_follow import directional_signal, usable_signal

class TestSignalIngress(unittest.TestCase):
  def signal(self, **changes):
    data=dict(lat=37.59214,lon=126.71348,distance=200,straight='RED_LIGHT_ON',straight_remain_time=117)
    data.update(changes)
    return directional_signal(data,100.,{}, {})
  def test_valid_red_and_green_are_accepted(self):
    self.assertTrue(usable_signal(self.signal()))
    self.assertTrue(usable_signal(self.signal(straight='GREEN_LIGHT_ON')))
  def test_corrupt_distance_is_rejected(self):
    for distance in (6805185,-1,float('nan')):
      self.assertFalse(usable_signal(self.signal(distance=distance)))
  def test_unavailable_signal_is_rejected(self):
    self.assertFalse(usable_signal(self.signal(straight='UNAVAILABLE')))
  def test_missing_position_is_rejected(self):
    self.assertFalse(usable_signal(self.signal(lat=None)))
