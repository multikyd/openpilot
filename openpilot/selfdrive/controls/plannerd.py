#!/usr/bin/env python3
import math

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

from openpilot.selfdrive.controls.lib.lateral_planner import LateralPlanner
import openpilot.cereal.messaging as messaging

from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner


def log_highway_overtake_candidate(sm, last_logged):
  """Observe opportunities without issuing a lane-change or signal request."""
  now = sm.logMonoTime['modelV2'] * 1e-9
  services = ('carState', 'carControl', 'carrotMan', 'radarState')
  if (now - last_logged < 1. or not sm['carrotMan'].naviHighway or
      not sm.all_checks(services) or
      any(abs(now - sm.logMonoTime[s] * 1e-9) > .3 for s in services)):
    return last_logged

  cs, cc = sm['carState'], sm['carControl']
  lead = sm['radarState'].leadOne
  speed_kph = cs.vEgo * CV.MS_TO_KPH
  lead_kph = lead.vLead * CV.MS_TO_KPH
  if (not (cs.canValid and cc.latActive and cc.longActive and lead.present and lead.radar) or
      not (speed_kph >= 70. and speed_kph <= cs.vCruise + 5. and cs.vCruise <= V_CRUISE_MAX) or
      not (0. <= lead_kph <= cs.vCruise - 10. and 10. <= lead.dRel <= 100. and lead.modelProb >= .9) or
      not math.isfinite(lead.vRel)):
    return last_logged

  cloudlog.event('highway_overtake_observation', speed_kph=speed_kph, cruise_kph=float(cs.vCruise),
                 lead_track=int(lead.radarTrackId), lead_distance_m=float(lead.dRel), lead_speed_kph=lead_kph,
                 closing_kph=-lead.vRel * CV.MS_TO_KPH,
                 left_lane_marking=int(cs.leftLaneLine), right_lane_marking=int(cs.rightLaneLine),
                 left_oem_ready=bool(cs.leftOemLaneChangeReady), right_oem_ready=bool(cs.rightOemLaneChangeReady),
                 left_bsm=bool(cs.leftBlindspot), right_bsm=bool(cs.rightBlindspot),
                 left_blinker=bool(cs.leftBlinker), right_blinker=bool(cs.rightBlinker),
                 rear_approach_verified=False)
  return now


def main():
  # Keep planning off CPU5, which runs the higher-priority UI and display IRQs.
  # CPU4 control loops retain CTRL_HIGH priority and preempt this CTRL_LOW task.
  config_realtime_process(4, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  ldw = LaneDepartureWarning()
  longitudinal_planner = LongitudinalPlanner(CP)
  lateral_planner = LateralPlanner(CP, debug=False)

  pm = messaging.PubMaster(['longitudinalPlan', 'driverAssistance', 'lateralPlan'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'vehicleParameters', 'radarState',
                            'modelV2', 'selfdriveState', 'carrotMan', 'leadBrakeObservation'],
                           poll='modelV2', ignore_avg_freq=['radarState'],
                           ignore_alive=['leadBrakeObservation'], ignore_valid=['leadBrakeObservation'])
  carrot = CarrotPlanner()
  observe_overtake = CP.carFingerprint == 'HYUNDAI_IONIQ_5_PE' and CP.openpilotLongitudinalControl
  last_overtake_log = -1.

  while True:
    sm.update()
    if sm.updated['modelV2']:
      lateral_planner.update(sm, carrot)
      lateral_planner.publish(sm, pm)
      longitudinal_planner.update(sm, carrot)
      longitudinal_planner.publish(sm, pm, carrot)
      if observe_overtake:
        last_overtake_log = log_highway_overtake_candidate(sm, last_overtake_log)

      ldw.update(sm.frame, sm['modelV2'], sm['carState'], sm['carControl'])
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks()
      msg.driverAssistance.leftLaneDeparture = ldw.left
      msg.driverAssistance.rightLaneDeparture = ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
