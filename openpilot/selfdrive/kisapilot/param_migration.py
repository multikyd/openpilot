"""Preserve stored tuning values when the parameter registry changes."""

import os
import stat
from pathlib import Path


RENAMED_PARAMS = {
  "CarrotCruiseDecel": "KisaCruiseDecel",
  "CarrotCruiseAtcDecel": "KisaTurnDecel",
  "StopDistanceCarrot": "KisaStandstillDistance",
  "CarrotSpeedTable": "KisaSpeedTable",
  "CarrotSmartSpeedControl": "KisaSmartSpeedControl",
  "LongitudinalPersonalityMax": "MaxFollowingPersonality",
  "ShowCustomBrightness": "DisplayBrightnessPct",
  "HDPuse": "StockHudAssistDisplayMode",
  "AutoCruiseControl": "AutoCruiseResumeMode",
  "CruiseEcoControl": "CruiseRecoverySpeedBoost",
  "AutoGasTokSpeed": "PedalTapResumeThreshold",
  "AutoGasSyncSpeed": "SyncSetSpeedOnAccelerator",
  "AutoEngage": "StartupEngagementMode",
  "DisableMinSteerSpeed": "IgnoreMinimumSteeringSpeed",
  "AutoSpeedUptoRoadSpeedLimit": "LeadSpeedIncreaseLimitPct",
  "AutoRoadSpeedAdjust": "SpeedLimitSetpointPolicy",
  "AutoCurveSpeedLowerLimit": "CurveSpeedFloorKph",
  "AutoCurveSpeedFactor": "VisionCurveSensitivityPct",
  "TbtBlinkerExperiment": "NavTurnSignalExperiment",
  "AutoTurnControl": "NavTurnAssistMode",
  "AutoTurnControlSpeedTurn": "NavTurnTargetKph",
  "AutoTurnControlTurnEnd": "NavTurnArrivalMarginSec",
  "AutoTurnMapChange": "NavTurnMapDisplay",
  "AutoNaviSpeedCtrlEnd": "EnforcementArrivalMarginSec",
  "AutoNaviSpeedCtrlMode": "NavSpeedControlScope",
  "AutoRoadSpeedLimitOffset": "RoadLimitOffsetKph",
  "AutoNaviSpeedBumpTime": "BumpArrivalMarginSec",
  "AutoNaviSpeedBumpSpeed": "BumpTargetKph",
  "BumpLearningSound": "LearnedBumpAudioEnabled",
  "BumpLearningRaise": "BumpLearningMaxIncreaseKph",
  "BumpLearningData": "LearnedBumpLocations",
  "SignalLearningData": "LearnedSignalCycles",
  "ManualStopLineData": "SavedStopLines",
  "AutoNaviSpeedDecelRate": "NavSpeedDecelMps2",
  "AutoNaviSpeedSafetyFactor": "EnforcementTargetPct",
  "AutoNaviCountDownMode": "NavCountdownScope",
  "MapTurnSpeedFactor": "MapCurveTargetPct",
  "ModelTurnSpeedFactor": "VisionTurnLookaheadSec",
  "StoppingAccel": "StoppingStateAccelMps2",
  "JLeadFactor3": "LeadJerkWeight",
  "CruiseSpeedUnit": "CruiseButtonExtendedStep",
  "CruiseSpeedUnitBasic": "CruiseButtonBasicStep",
  "MyDrivingMode": "DrivingProfile",
  "MyDrivingModeAuto": "AdaptiveDrivingProfile",
  "TrafficLightDetectMode": "TrafficSignalControlMode",
  "LongTuningKpV": "LongitudinalSpeedKp",
  "LongTuningKiV": "LongitudinalSpeedKi",
  "LongActuatorDelay": "LongitudinalDelaySec",
  "VEgoStopping": "ModelStopThresholdMps",
  "RadarLatFactor": "RadarCutInLateralFactor",
  "HyundaiCameraSCC": "HyundaiCameraCruiseMode",
  "CanfdHDA2": "CanfdHighwayAssistMode",
  "SoundVolumeAdjust": "AlertVolumePct",
  "TrafficStopDistanceAdjust": "SignalStopDistanceOffsetM",
  "AChangeCostStarting": "LaunchAccelChangeCost",
  "UseLaneLineSpeed": "LaneModeMinSpeedKph",
  "UseLaneLineCurveSpeed": "LaneModeCurveThresholdKph",
  "LaneChangeNeedTorque": "LaneChangeRequiresTorque",
  "LaneChangeDelay": "LaneChangeDelaySec",
  "LaneChangeBsd": "LaneChangeBlindspotPolicy",
  "CustomSteerMax": "SteeringTorqueLimit",
  "CustomSteerDeltaUp": "SteeringTorqueRiseLimit",
  "CustomSteerDeltaDown": "SteeringTorqueFallLimit",
  "CustomSteerDeltaUpLC": "LaneChangeTorqueRiseLimit",
  "CustomSteerDeltaDownLC": "LaneChangeTorqueFallLimit",
  "SteerActuatorDelay": "SteeringDelaySec",
  "LatSmoothSec": "LateralSmoothingSec",
  "CustomSR": "ManualSteeringRatio",
  "SteerRatioRate": "LiveSteeringRatioPct",
  "CruiseOnDist": "AutoResumeLeadDistanceM",
  "AChangeCost2": "LeadJerkAccelChangeCost",
  "CameraOffsetAdj": "CameraLateralOffsetM",
  "EnableSpeedTF": "SpeedBasedFollowingGap",
  "CruiseMaxVals0": "MaxAccelAt0Kph",
  "CruiseMaxVals1": "MaxAccelAt10Kph",
  "CruiseMaxVals2": "MaxAccelAt40Kph",
  "CruiseMaxVals3": "MaxAccelAt60Kph",
  "CruiseMaxVals4": "MaxAccelAt80Kph",
  "CruiseMaxVals5": "MaxAccelAt110Kph",
  "CruiseMaxVals6": "MaxAccelAt140Kph",
  "CruiseSpeed1": "CruiseSpeedPreset1",
  "CruiseSpeed2": "CruiseSpeedPreset2",
  "CruiseSpeed3": "CruiseSpeedPreset3",
  "CruiseSpeed4": "CruiseSpeedPreset4",
  "CruiseSpeed5": "CruiseSpeedPreset5",
  "TFollowGap1": "FollowingTimeGap1",
  "TFollowGap2": "FollowingTimeGap2",
  "TFollowGap3": "FollowingTimeGap3",
  "TFollowGap4": "FollowingTimeGap4",
}


def migrate_parameters(params):
  directory = Path(params.get_param_path())
  for previous, current in RENAMED_PARAMS.items():
    source = directory / previous
    target = directory / current
    if target.exists() or target.is_symlink():
      continue
    try:
      if stat.S_ISREG(source.lstat().st_mode):
        # Manager runs this before it clears unregistered keys or starts services.
        os.rename(source, target)
    except FileNotFoundError:
      continue
