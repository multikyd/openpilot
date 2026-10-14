import math
import numpy as np
import time
import wave


from openpilot.cereal import log, messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog

from openpilot.system import micd
from openpilot.common.hardware import HARDWARE

from openpilot.common.params import Params

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096 # (approx 100ms)
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1
ALERT_RAMP_TIME = 4 # seconds to ramp critical alerts to max volume
ALERT_MAX_TIME = 8 # seconds before critical alerts switch to the max sound
SELFDRIVE_STATE_TIMEOUT = 5 # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 22 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

VOLUME_BASE = 20
if HARDWARE.get_device_type() == "tizi":
  AMBIENT_DB = 30
  VOLUME_BASE = 10

AudibleAlert = log.SelfdriveState.AudibleAlert
CRITICAL_MAX = -1 # internal sound key, not an AudibleAlert
BUMP_CHIME = -2 # dedicated one-shot bump tone, local to soundd
SIGNAL_COAST_VOICE = -3 # one-shot Korean signal-assisted coasting announcement


sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("warning.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("warning.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("dm_warning.wav", None, MAX_VOLUME),

  AudibleAlert.preAlert: ("pre_alert.wav", 1, MAX_VOLUME),

  AudibleAlert.warningSoft: ("critical.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("dm_critical.wav", None, MAX_VOLUME),
  CRITICAL_MAX: ("dm_critical_max.wav", None, MAX_VOLUME),

  AudibleAlert.trafficSignGreen: ("traffic_sign_green.wav", None, MAX_VOLUME),
  AudibleAlert.trafficSignChanged: ("traffic_sign_changed.wav", None, MAX_VOLUME),
  AudibleAlert.bsdWarning: ("audio_car_watchout.wav", None, MAX_VOLUME),
  AudibleAlert.laneChange: ("audio_lane_change.wav", None, MAX_VOLUME),
  AudibleAlert.stopStop: ("audio_stopstop.wav", None, MAX_VOLUME),
  AudibleAlert.stopping: ("audio_stopping.wav", None, MAX_VOLUME),
  AudibleAlert.autoHold: ("audio_auto_hold.wav", None, MAX_VOLUME),
  AudibleAlert.speedDown:  ("audio_speed_down.wav", None, MAX_VOLUME),
  AudibleAlert.reverseGear: ("reverse_gear.wav", 1, MAX_VOLUME),
  AudibleAlert.dingdong: ("dingdong.wav", 1, MAX_VOLUME),
  BUMP_CHIME: ("bump_ding.wav", 1, 0.7),
  SIGNAL_COAST_VOICE: ("audio_signal_coast.wav", 1, MAX_VOLUME),
}

def check_selfdrive_timeout_alert(sm):
  ss_missing = time.monotonic() - sm.recv_time['selfdriveState']

  if ss_missing > SELFDRIVE_STATE_TIMEOUT:
    if sm['selfdriveState'].enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10:
      return True

  return False


class Soundd:
  def __init__(self):
    self.soundVolumeAdjust = float(Params().get("SoundVolumeAdjust"))/100.
    self.bump_sound_sequence = None
    self.signal_voice_armed = True
    self.signal_voice_last = -100.0
    self.signal_voice_active_since = None
    self.signal_voice_inactive_since = None
    self.load_sounds()

    self.current_alert = AudibleAlert.none
    self.current_sound = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.ramp_start_volume = MIN_VOLUME
    self.ramp_start_time = 0.

    self.selfdrive_timeout_alert = False
    self.pending_stop = False

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      with wave.open(BASEDIR + "/openpilot/selfdrive/assets/sounds/" + filename, 'r') as wavefile:
        assert wavefile.getnchannels() == 1
        assert wavefile.getsampwidth() == 2
        assert wavefile.getframerate() == SAMPLE_RATE

        length = wavefile.getnframes()
        self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      num_loops = sound_list[self.current_alert][1]
      sound_data = self.loaded_sounds[self.current_sound]
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write
        current_sound_frame = self.current_sound_frame % len(sound_data)
        loops = self.current_sound_frame // len(sound_data)
        if self.pending_stop and current_sound_frame == 0:
          self.current_alert = AudibleAlert.none
          self.pending_stop = False
          break

    return ret * self.current_volume

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert):
    current_alert_played_once = self.current_alert == AudibleAlert.none or self.current_sound_frame >= len(self.loaded_sounds[self.current_sound])
    # let looping sounds finish the current loop instead of cutting off mid tone
    if new_alert == AudibleAlert.none and self.current_alert != AudibleAlert.none and sound_list[self.current_alert][1] is None:
      if current_alert_played_once:
        self.pending_stop = True
      else:
        self.current_alert = AudibleAlert.none
        self.current_sound_frame = 0
      return
    self.pending_stop = False
    if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
      if new_alert in (AudibleAlert.warningSoft, AudibleAlert.warningImmediate):
        self.ramp_start_volume = self.current_volume
        self.ramp_start_time = time.monotonic()
      self.current_alert = new_alert
      self.current_sound = new_alert
      self.current_sound_frame = 0

  def update_carrot_alert(self, sm, new_alert):
    cm = sm['carrotMan']
    sequence = cm.bumpSoundSequence
    changed = self.bump_sound_sequence is not None and sequence > self.bump_sound_sequence
    self.bump_sound_sequence = sequence
    if (changed and cm.bumpSoundEnabled and not 0 <= cm.leftSec <= 11 and
        sm.valid['carrotMan'] and sm.alive['carrotMan'] and
        time.monotonic() - sm.recv_time['carrotMan'] < 1 and new_alert == AudibleAlert.none):
      return BUMP_CHIME
    return new_alert

  def update_signal_voice(self, sm, new_alert):
    now = time.monotonic()
    active = (sm.valid['longitudinalPlan'] and sm.alive['longitudinalPlan'] and
              0 <= now - sm.recv_time['longitudinalPlan'] < 0.3 and
              sm.valid['selfdriveState'] and sm.alive['selfdriveState'] and
              sm['selfdriveState'].enabled and sm['longitudinalPlan'].signalCoastActive)
    if not active:
      self.signal_voice_active_since = None
      if self.signal_voice_inactive_since is None:
        self.signal_voice_inactive_since = now
      if now - self.signal_voice_inactive_since >= 10.0:
        self.signal_voice_armed = True
      return new_alert
    self.signal_voice_inactive_since = None
    if self.signal_voice_active_since is None:
      self.signal_voice_active_since = now
    if (new_alert == AudibleAlert.none and self.signal_voice_armed and
        now - self.signal_voice_last >= 30.0 and now - self.signal_voice_active_since >= 0.25):
      self.signal_voice_armed = False
      self.signal_voice_last = now
      return SIGNAL_COAST_VOICE
    return new_alert

  def get_audible_alert(self, sm):
    if sm.updated['selfdriveState']:
      new_alert = self.update_carrot_alert(sm, sm['selfdriveState'].alertSound.raw)
      new_alert = self.update_signal_voice(sm, new_alert)
      self.update_alert(new_alert)
    elif check_selfdrive_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate)
      self.selfdrive_timeout_alert = True
    elif self.selfdrive_timeout_alert:
      self.update_alert(AudibleAlert.none)
      self.selfdrive_timeout_alert = False

  def calculate_volume(self, weighted_db):
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (MAX_VOLUME - MIN_VOLUME) + MIN_VOLUME
    return math.pow(VOLUME_BASE, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  @retry(attempts=10, delay=3)
  def get_stream(self, sd):
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    return sd.OutputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER)

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd
    micd.patch_sounddevice(sd)

    sm = messaging.SubMaster(['selfdriveState', 'soundPressure', 'carrotMan', 'longitudinalPlan'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        sm.update(0)

        # freeze volume during alerts to avoid mic feedback increasing volume
        if sm.updated['soundPressure']:
          self.spl_filter_weighted.update(sm["soundPressure"].soundPressureWeightedDb)
          if self.current_alert == AudibleAlert.none:
            self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x)) * self.soundVolumeAdjust

        self.get_audible_alert(sm)

        if self.current_alert in (AudibleAlert.warningSoft, AudibleAlert.warningImmediate):
          elapsed = time.monotonic() - self.ramp_start_time
          ramp_vol = float(np.interp(elapsed, [0, ALERT_RAMP_TIME], [self.ramp_start_volume, MAX_VOLUME]))
          self.current_volume = max(self.current_volume, ramp_vol)
          if elapsed >= ALERT_MAX_TIME and self.current_sound != CRITICAL_MAX:
            self.current_sound = CRITICAL_MAX
            self.current_sound_frame = 0

        rk.keep_time()

        assert stream.active


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
