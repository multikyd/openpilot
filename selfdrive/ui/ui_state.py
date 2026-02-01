import pyray as rl
import numpy as np
import time
import threading
from collections.abc import Callable
from enum import Enum, IntEnum
from cereal import messaging, car, log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.lib.prime_state import PrimeState
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.hardware import HARDWARE, PC

BACKLIGHT_OFFROAD = 65 if HARDWARE.get_device_type() == "mici" else 50


class UIStatus(Enum):
  DISENGAGED = "disengaged"
  ENGAGED = "engaged"
  OVERRIDE = "override"


class GearShifter(IntEnum):
  unknown = 0
  park = 1
  drive = 2
  neutral = 3
  reverse = 4
  sport = 5
  low = 6
  brake = 7
  eco = 8
  manumatic = 9


class UIState:
  _instance: 'UIState | None' = None

  def __new__(cls):
    if cls._instance is None:
      cls._instance = super().__new__(cls)
      cls._instance._initialize()
    return cls._instance

  def _initialize(self):
    self.params = Params()
    self.sm = messaging.SubMaster(
      [
        "modelV2",
        "controlsState",
        "onroadEvents",
        "liveCalibration",
        "radarState",
        "deviceState",
        "pandaStates",
        "carParams",
        "driverMonitoringState",
        "carState",
        "driverStateV2",
        "roadCameraState",
        "wideRoadCameraState",
        "managerState",
        "selfdriveState",
        "longitudinalPlan",
        "gpsLocationExternal",
        "carOutput",
        "carControl",
        "liveParameters",
        "rawAudioData",
        "peripheralState",
        "gpsLocation",
        "lateralPlan",
        "carrotMan",
      ]
    )

    self.prime_state = PrimeState()

    # UI Status tracking
    self.status: UIStatus = UIStatus.DISENGAGED
    self.started_frame: int = 0
    self.started_time: float = 0.0
    self._engaged_prev: bool = False
    self._started_prev: bool = False

    # Core state variables
    self.is_metric: bool = self.params.get_bool("IsMetric")
    self.is_release = self.params.get_bool("IsReleaseBranch")
    self.always_on_dm: bool = self.params.get_bool("AlwaysOnDM")
    self.started: bool = False
    self.ignition: bool = False
    self.recording_audio: bool = False
    self.panda_type: log.PandaState.PandaType = log.PandaState.PandaType.unknown
    self.personality: log.LongitudinalPersonality = log.LongitudinalPersonality.standard
    self.has_longitudinal_control: bool = False
    self.CP: car.CarParams | None = None
    self.light_sensor: float = -1.0
    self._param_update_time: float = 0.0

    # Kisa state variables
    self.controlAllowed: bool = False
    self.freeSpace: float = 0.0
    self.memoryUsage: float = 0.0
    self.cpuTemp: float = 0.0
    self.gpuTemp: float = 0.0
    self.cpuUsage: float = 0.0
    self.voltage: float = 0.0
    self.fanSpeedPercentDesired: int = 0
    self.storageUsage: int = 0
    self.dspTemp: float = 0.0
    self.memoryTemp: float = 0.0
    self.modemTemp: float = 0.0
    self.pmicTemp: float = 0.0
    self.intakeTemp: float = 0.0
    self.exhaustTemp: float = 0.0
    self.caseTemp: float = 0.0
    self.maxTemp: float = 0.0
    self.fanSpeedRpm: int = 0

    self.gpsAccuracy: float = 0.0
    self.altitude: float = 0.0
    self.bearing: float = 0.0

    self.angleOffsetDeg: float = 0.0
    self.angleOffsetAverageDeg: float = 0.0
    self.steerRatio: float = 0.0

    self.accel: float = 0.0
    self.debugMsg1: str = ""
    self.debugMsg2: str = ""
    self.debugMsg3: str = ""
    self.standstillElapsedTimer: int = 0
    self.activeLaneLine: bool = False
    self.pandaSafetyModel: str = ""
    self.interfaceSafetyModel: str = ""
    self.rxChecks: bool = False
    self.mismatchCounter: bool = False

    self.brakePress: bool = False
    self.gasPress: bool = False
    self.brakeLights: bool = False
    self.gearShifter: GearShifter = GearShifter.unknown
    self.leftBlinker: bool = False
    self.rightBlinker: bool = False
    self.leftblindspot: bool = False
    self.rightblindspot: bool = False
    self.tpmsUnit: int = 0
    self.tpmsPressureFl: float = 0
    self.tpmsPressureFr: float = 0
    self.tpmsPressureRl: float = 0
    self.tpmsPressureRr: float = 0
    self.radarDRel: float = 0.0
    self.radarVRel: float = 0.0
    self.vSetDis: float = 0
    self.cruiseAccStatus: bool = False
    self.angleSteers: float = 0.0
    self.autoHold: bool = False
    self.aReqValue: float = 0.0
    self.latEnabled: bool = False
    self.lProb: float = 0.0
    self.rProb: float = 0.0

    self.enabled: bool = False
    self.show_ui_bsm: bool = self.params.get_bool("KisaBlindSpotDetect")
    self.rec_status: bool = False
    self.cruise_gap: int = self.params.get("LongitudinalPersonality") + 1
    self.debug_msg: int = self.params.get("ShowDebugUI")
    self.camera_scc: int = self.params.get("HyundaiCameraSCC")
    self.show_radar_info: int = self.params.get("ShowRadarInfo")
    self.radar_lat_factor: float = self.params.get("RadarLatFactor")
    self.show_plot_mode: int = self.params.get("ShowPlotMode")
    self.driving_model: str = self.params.get("DrivingModel")

    # Carrot
    self.active_carrot: int = 0
    self.xSpdLimit: int = 0
    self.xSpdLimitOrg: int = 0
    self.xSpdDist: int = 0
    self.xSpdType: int = 0
    self.desiredSpeed: int = 0
    self.desiredSource: str = ""

    # Callbacks
    self._offroad_transition_callbacks: list[Callable[[], None]] = []
    self._engaged_transition_callbacks: list[Callable[[], None]] = []

    self.update_params()

  def add_offroad_transition_callback(self, callback: Callable[[], None]):
    self._offroad_transition_callbacks.append(callback)

  def add_engaged_transition_callback(self, callback: Callable[[], None]):
    self._engaged_transition_callbacks.append(callback)

  @property
  def engaged(self) -> bool:
    return self.started and self.sm["selfdriveState"].enabled

  def is_onroad(self) -> bool:
    return self.started

  def is_offroad(self) -> bool:
    return not self.started

  def update(self) -> None:
    self.prime_state.start()  # start thread after manager forks ui
    self.sm.update(0)
    self._update_state()
    self._update_status()
    if time.monotonic() - self._param_update_time > 5.0:
      self.update_params()
    device.update()

  def _update_state(self) -> None:
    # Handle panda states updates
    if self.sm.updated["pandaStates"]:
      panda_states = self.sm["pandaStates"]

      if len(panda_states) > 0:
        # Get panda type from first panda
        self.panda_type = panda_states[0].pandaType
        # Check ignition status across all pandas
        if self.panda_type != log.PandaState.PandaType.unknown:
          self.ignition = any(state.ignitionLine or state.ignitionCan for state in panda_states)
          self.controlAllowed = panda_states[0].controlsAllowed

    elif self.sm.frame - self.sm.recv_frame["pandaStates"] > 5 * rl.get_fps():
      self.panda_type = log.PandaState.PandaType.unknown

    # Handle wide road camera state updates
    if self.sm.updated["wideRoadCameraState"]:
      cam_state = self.sm["wideRoadCameraState"]
      self.light_sensor = max(100.0 - cam_state.exposureValPercent, 0.0)
    elif not self.sm.alive["wideRoadCameraState"] or not self.sm.valid["wideRoadCameraState"]:
      self.light_sensor = -1

    # Update started state
    self.started = self.sm["deviceState"].started and self.ignition

    # Update recording audio state
    self.recording_audio = self.params.get_bool("RecordAudio") and self.started

    self.is_metric = self.params.get_bool("IsMetric")
    self.always_on_dm = self.params.get_bool("AlwaysOnDM")

    # Kisa states update
    if self.sm.updated["deviceState"]:
      device_state = self.sm["deviceState"]
      self.freeSpace = device_state.freeSpacePercent
      self.memoryUsage = device_state.memoryUsagePercent
      cpu_temps = list(device_state.cpuTempC)
      gpu_temps = list(device_state.gpuTempC)
      cpu_usages = list(device_state.cpuUsagePercent)
      if len(cpu_temps) > 0:
        self.cpuTemp = sum(cpu_temps) / len(cpu_temps)
      else:
        self.cpuTemp = 0.0
      if len(gpu_temps) > 0:
        self.gpuTemp = sum(gpu_temps) / len(gpu_temps)
      else:
        self.gpuTemp = 0.0
      valid_usages = [u for u in cpu_usages if u > 0]
      if len(valid_usages) > 0:
        self.cpuUsage = sum(valid_usages) / len(valid_usages)
      else:
        self.cpuUsage = 0.0
      self.dspTemp = device_state.dspTempC
      self.memoryTemp = device_state.memoryTempC
      modem_temps = list(device_state.modemTempC)
      if len(modem_temps) > 0:
        self.modemTemp = sum(modem_temps) / len(modem_temps)
      else:
        self.modemTemp = 0.0
      pmic_temps = list(device_state.pmicTempC)
      if len(pmic_temps) > 0:
        self.pmicTemp = sum(pmic_temps) / len(pmic_temps)
      else:
        self.pmicTemp = 0.0
      self.intakeTemp = device_state.intakeTempC
      self.exhaustTemp = device_state.exhaustTempC
      self.caseTemp = device_state.caseTempC
      self.maxTemp = device_state.maxTempC

      self.fanSpeedPercentDesired = device_state.fanSpeedPercentDesired
      self.storageUsage = int(round(100. - device_state.freeSpacePercent))

    if self.sm.updated["peripheralState"]:
      peripheral_state = self.sm["peripheralState"]
      self.voltage = peripheral_state.voltage / 1000.0
      self.fanSpeedRpm = peripheral_state.fanSpeedRpm

    if self.sm.updated["gpsLocation"]:
      gps_Location = self.sm["gpsLocation"]
      self.gpsAccuracy = gps_Location.verticalAccuracy
      self.altitude = gps_Location.altitude
      self.bearing = gps_Location.bearingDeg

    if self.sm.updated["liveParameters"]:
      live_Parameters = self.sm["liveParameters"]
      self.angleOffsetDeg = live_Parameters.angleOffsetDeg
      self.angleOffsetAverageDeg = live_Parameters.angleOffsetAverageDeg
      self.steerRatio = live_Parameters.steerRatio

    if self.sm.updated["controlsState"]:
      controls_state = self.sm["controlsState"]
      self.accel = controls_state.accel
      self.standstillElapsedTimer = controls_state.standStillTimer
      self.activeLaneLine = controls_state.activeLaneLine
      self.debugMsg1 = controls_state.debugMsg1
      #self.debugMsg2 = controls_state.debugMsg2
      #self.debugMsg3 = controls_state.debugMsg3

    if self.sm.updated["carState"]:
      car_state = self.sm["carState"]
      self.brakePress = car_state.brakePressed
      self.gasPress = car_state.gasPressed
      self.brakeLights = car_state.brakeLights
      self.gearShifter = car_state.gearShifter
      self.leftBlinker = car_state.leftBlinker
      self.rightBlinker = car_state.rightBlinker
      self.leftblindspot = car_state.leftBlindspot
      self.rightblindspot = car_state.rightBlindspot
      self.tpmsUnit = car_state.tpms.unit
      self.tpmsPressureFl = car_state.tpms.fl
      self.tpmsPressureFr = car_state.tpms.fr
      self.tpmsPressureRl = car_state.tpms.rl
      self.tpmsPressureRr = car_state.tpms.rr
      self.radarDRel = car_state.radarDRel
      self.radarVRel = car_state.radarVRel
      self.vSetDis = car_state.vSetDis
      self.cruiseAccStatus = car_state.cruiseState.enabled
      self.angleSteers = car_state.steeringAngleDeg
      self.autoHold = car_state.brakeHoldActive
      self.aReqValue = car_state.aReqValue
      self.latEnabled = car_state.latEnabled
      self.cruise_gap = car_state.cruiseGap

    if self.sm.updated["lateralPlan"]:
      lat_plan = self.sm["lateralPlan"]
      self.lProb = lat_plan.lProb
      self.rProb = lat_plan.rProb

    if self.sm.updated["carrotMan"]:
      carrotman_state = self.sm["carrotMan"]
      self.active_carrot = carrotman_state.activeCarrot
      self.xSpdLimit = carrotman_state.xSpdLimit
      self.xSpdLimitOrg = carrotman_state.xSpdLimitOrg
      self.xSpdDist = carrotman_state.xSpdDist
      self.xSpdType = carrotman_state.xSpdType
      self.desiredSpeed = carrotman_state.desiredSpeed
      self.desiredSource = carrotman_state.desiredSource

  def _update_status(self) -> None:
    if self.started and self.sm.updated["selfdriveState"]:
      ss = self.sm["selfdriveState"]
      state = ss.state

      if state in (log.SelfdriveState.OpenpilotState.preEnabled, log.SelfdriveState.OpenpilotState.overriding):
        self.status = UIStatus.OVERRIDE
      else:
        self.status = UIStatus.ENGAGED if ss.enabled or self.latEnabled else UIStatus.DISENGAGED

      self.enabled = ss.enabled
      self.pandaSafetyModel = ss.pandaSafetyModel
      self.interfaceSafetyModel = ss.interfaceSafetyModel
      self.rxChecks = ss.rxChecks
      self.mismatchCounter = ss.mismatchCounter

    # Check for engagement state changes
    if self.engaged != self._engaged_prev:
      for callback in self._engaged_transition_callbacks:
        callback()
      self._engaged_prev = self.engaged

    # Handle onroad/offroad transition
    if self.started != self._started_prev or self.sm.frame == 1:
      if self.started:
        self.status = UIStatus.DISENGAGED
        self.started_frame = self.sm.frame
        self.started_time = time.monotonic()

      for callback in self._offroad_transition_callbacks:
        callback()

      self._started_prev = self.started

  def update_params(self) -> None:
    # For slower operations
    # Update longitudinal control state
    CP_bytes = self.params.get("CarParamsPersistent")
    if CP_bytes is not None:
      self.CP = messaging.log_from_bytes(CP_bytes, car.CarParams)
      if self.CP.alphaLongitudinalAvailable:
        self.has_longitudinal_control = self.params.get_bool("AlphaLongitudinalEnabled")
      else:
        self.has_longitudinal_control = self.CP.openpilotLongitudinalControl

    # Update user params
    self.show_ui_bsm = self.params.get_bool("KisaBlindSpotDetect")
    self.debug_msg = self.params.get("ShowDebugUI")
    self.show_radar_info = self.params.get("ShowRadarInfo")
    self.radar_lat_factor = self.params.get("RadarLatFactor")
    self.show_plot_mode = self.params.get("ShowPlotMode")
    self.rec_status = self.params.get_bool("RecordingRunning")

    self._param_update_time = time.monotonic()


class Device:
  def __init__(self):
    self._ignition = False
    self._interaction_time: float = -1
    self._override_interactive_timeout: int | None = None
    self._interactive_timeout_callbacks: list[Callable] = []
    self._prev_timed_out = False
    self._awake: bool = True

    self._offroad_brightness: int = BACKLIGHT_OFFROAD
    self._last_brightness: int = 0
    self._brightness_filter = FirstOrderFilter(BACKLIGHT_OFFROAD, 10.00, 1 / gui_app.target_fps)
    self._brightness_thread: threading.Thread | None = None

    self.custom_brightness: int = Params().get("ShowCustomBrightness")
    self._default_custom_brightness = self.custom_brightness

  @property
  def awake(self) -> bool:
    return self._awake

  def set_override_interactive_timeout(self, timeout: int | None) -> None:
    # Override the interactive timeout duration temporarily
    self._override_interactive_timeout = timeout
    self._reset_interactive_timeout()

  @property
  def interactive_timeout(self) -> int:
    if self._override_interactive_timeout is not None:
      return self._override_interactive_timeout

    ignition_timeout = 10 if gui_app.big_ui() else 5
    return ignition_timeout if ui_state.ignition else 30

  def _reset_interactive_timeout(self) -> None:
    self._interaction_time = time.monotonic() + self.interactive_timeout

  def add_interactive_timeout_callback(self, callback: Callable):
    self._interactive_timeout_callbacks.append(callback)

  def update(self):
    # do initial reset
    if self._interaction_time <= 0:
      self._reset_interactive_timeout()

    self._update_brightness()
    self._update_wakefulness()

  def set_offroad_brightness(self, brightness: int | None):
    if brightness is None:
      brightness = BACKLIGHT_OFFROAD
    self._offroad_brightness = min(max(brightness, 0), 100)

  def _update_brightness(self):
    clipped_brightness = self._offroad_brightness

    if ui_state.started and ui_state.light_sensor >= 0:
      clipped_brightness = ui_state.light_sensor

      # CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
      if clipped_brightness <= 8:
        clipped_brightness = clipped_brightness / 903.3
      else:
        clipped_brightness = ((clipped_brightness + 16.0) / 116.0) ** 3.0

      clipped_brightness = float(np.interp(clipped_brightness, [0, 1], [30, 100]))

    brightness = round(self._brightness_filter.update(clipped_brightness))
    brightness = round(brightness * self.custom_brightness / 100)
    brightness = round(np.clip(brightness, 10, 100))
    if not self._awake:
      brightness = 0

    if brightness != self._last_brightness:
      if self._brightness_thread is None or not self._brightness_thread.is_alive():
        self._brightness_thread = threading.Thread(target=HARDWARE.set_screen_brightness, args=(brightness,))
        self._brightness_thread.start()
        self._last_brightness = brightness

  def _update_wakefulness(self):
    # Handle interactive timeout
    ignition_just_turned_off = not ui_state.ignition and self._ignition
    self._ignition = ui_state.ignition

    if ignition_just_turned_off or any(ev.left_down for ev in gui_app.mouse_events):
      self._reset_interactive_timeout()
      self.custom_brightness = 100

    interaction_timeout = time.monotonic() > self._interaction_time
    if interaction_timeout and not self._prev_timed_out:
      for callback in self._interactive_timeout_callbacks:
        callback()
      self.custom_brightness = self._default_custom_brightness
    self._prev_timed_out = interaction_timeout

    self._set_awake(ui_state.ignition or not interaction_timeout or PC)

  def _set_awake(self, on: bool):
    if on != self._awake:
      self._awake = on
      cloudlog.debug(f"setting display power {int(on)}")
      HARDWARE.set_display_power(on)
      gui_app.set_should_render(on)


# Global instance
ui_state = UIState()
device = Device()
