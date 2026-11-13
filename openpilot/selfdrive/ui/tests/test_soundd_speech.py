import unittest

from openpilot.selfdrive.ui.soundd import Soundd, AudibleAlert, sound_list, SAMPLE_RATE


class TestSpeechPlayback(unittest.TestCase):
  def sound(self):
    sound = Soundd.__new__(Soundd)
    sound.load_sounds()
    sound.current_alert = AudibleAlert.none
    sound.current_sound = AudibleAlert.none
    sound.current_sound_frame = 0
    sound.current_volume = 1.
    sound.pending_stop = False
    return sound

  def test_short_events_finish_the_first_spoken_sentence(self):
    for alert in (AudibleAlert.stopping, AudibleAlert.trafficSignGreen,
                  AudibleAlert.trafficSignChanged, AudibleAlert.signalCoast):
      with self.subTest(alert=alert):
        sound = self.sound()
        self.assertEqual(sound_list[alert][1], 1)
        sound.update_alert(alert)
        sound.get_sound_data(int(.2*SAMPLE_RATE))
        sound.update_alert(AudibleAlert.none)
        self.assertEqual(sound.current_alert, alert)
        sound.get_sound_data(len(sound.loaded_sounds[alert]))
        self.assertEqual(sound.current_sound_frame, len(sound.loaded_sounds[alert]))
        sound.update_alert(AudibleAlert.none)
        self.assertEqual(sound.current_alert, AudibleAlert.none)

  def test_spoken_alert_does_not_loop_if_event_persists(self):
    sound = self.sound();alert = AudibleAlert.stopping
    sound.update_alert(alert)
    sound.get_sound_data(len(sound.loaded_sounds[alert]))
    sound.update_alert(alert)
    self.assertFalse(sound.get_sound_data(4096).any())

  def test_urgent_warning_can_interrupt_speech(self):
    sound = self.sound()
    sound.update_alert(AudibleAlert.signalCoast)
    sound.get_sound_data(4096)
    sound.update_alert(AudibleAlert.warningImmediate)
    self.assertEqual(sound.current_alert, AudibleAlert.warningImmediate)
    self.assertEqual(sound.current_sound_frame, 0)

  def test_wav_format_matches_output(self):
    sound = self.sound()
    self.assertGreater(len(sound.loaded_sounds[AudibleAlert.signalCoast]), SAMPLE_RATE)


if __name__ == '__main__':
  unittest.main()
