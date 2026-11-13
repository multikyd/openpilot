import unittest
from openpilot.selfdrive.carrot.signal_timing import display_timing
from openpilot.selfdrive.carrot.signal_follow import signal_from_sinf, directional_signal
from openpilot.selfdrive.controls.tests import test_signal_live_display as live


class SignalTimingTest(unittest.TestCase):
  def test_phone_delay_and_elapsed_time_are_both_removed(self):
    fixture = live.TestLiveSignalDisplay();fixture.setUp()
    fixture.signal.update(display_timing((fixture.wall-.8)*1000,fixture.wall))
    self.assertAlmostEqual(fixture.show(.4)['straightRemain'],38.8,places=4)
    self.assertEqual(fixture.show(.4)['straightState'],2)

  def test_missing_and_invalid_clocks_do_not_shift_countdown(self):
    for stamp in (None,0,float('nan'),'bad',1790000010000,1789999990000):
      self.assertEqual(display_timing(stamp,1790000000.)['display_age_s'],0.)

  def test_legacy_conversion_keeps_sender_time(self):
    stamp = 1790000000123
    signal = signal_from_sinf(dict(_source_timestamp_ms=stamp,redLightOn=True,
                                   location=dict(latitude=37.5,longitude=126.7)))
    self.assertEqual(signal['_source_timestamp_ms'],stamp)
    normalized = directional_signal(signal,100.,{}, {})
    self.assertEqual(normalized['ts'],100.)  # control freshness remains reception-based

  def test_expired_timer_does_not_predict_a_lamp_change(self):
    fixture = live.TestLiveSignalDisplay();fixture.setUp()
    fixture.signal['movements']['straight'].update(state='RED_LIGHT_ON',remain=1.)
    fixture.signal.update(display_age_s=.8)
    result = fixture.show(.4)
    self.assertEqual(result['straightRemain'],0.)
    self.assertEqual(result['straightState'],1)

  def test_same_sender_timestamp_keeps_time_across_reception(self):
    first = display_timing(1790000000000,1790000000.2)['display_age_s']+.8
    second = display_timing(1790000000000,1790000001.)['display_age_s']
    self.assertAlmostEqual(first,second,places=5)


if __name__ == '__main__':
  unittest.main()
