import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper


class FakeParams:
  def get(self, key):
    return {'LaneChangeNeedTorque': 0, 'LaneChangeBsd': 0, 'LaneChangeDelay': 0., 'ModelTurnSpeedFactor': 0.}[key]


class TestLaneChangeCancel(unittest.TestCase):
  def setUp(self):
    with patch(DesireHelper.__module__+'.Params', FakeParams):
      self.helper = DesireHelper()
    self.helper.cancel_torque_threshold = 250.
    self.cs = NS(vEgo=20., aEgo=0., leftBlinker=False, rightBlinker=False, steeringTorque=0.,
                 steeringPressed=False, steeringAngleDeg=0., leftBlindspot=False, rightBlindspot=False,
                 leftLaneLine=0, rightLaneLine=0)
    lane = lambda y: NS(x=[i*3. for i in range(33)], y=[y]*33)
    self.model = NS(laneLines=[lane(-5.4), lane(-1.8), lane(1.8), lane(5.4)],
                    laneLineProbs=[.99]*4, roadEdges=[lane(-7.2), lane(7.2)],
                    orientationRate=NS(z=[0.]*33), meta=NS(desireState=[0.]*7))
    self.carrot = NS(atcType='', carrotCmdIndex=0, carrotCmd='', carrotArg='')
    self.radar = NS(leadLeft=NS(present=False), leadRight=NS(present=False))
    self.tick(30)

  def tick(self, count=1, probability=.8, active=True):
    for _ in range(count):
      self.helper.update(self.cs, self.model, active, probability, self.carrot, self.radar)

  def begin(self, side='left'):
    setattr(self.cs, side+'Blinker', True)
    self.tick(2)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.laneChangeStarting)

  def geometry(self, center):
    for line, y in zip(self.model.laneLines, [-5.4, -1.8, 1.8, 5.4]):
      line.y = [y+center]*33
    self.model.roadEdges[0].y = [-7.2+center]*33
    self.model.roadEdges[1].y = [7.2+center]*33

  def test_one_blinker_does_not_start_a_second_change(self):
    self.begin()
    self.tick(50, probability=0.)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)
    self.cs.steeringPressed, self.cs.steeringTorque = True, 300.
    self.tick(30)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)
    self.assertEqual(self.helper.desire, log.Desire.none)

  def test_off_then_on_rearms(self):
    self.begin()
    self.tick(50, probability=0.)
    self.cs.leftBlinker = False
    self.tick(8)
    self.begin()

  def test_short_signal_dropout_does_not_rearm(self):
    self.begin()
    self.tick(50, probability=0.)
    self.cs.leftBlinker = False
    self.tick(1)
    self.cs.leftBlinker = True
    self.tick(20)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)

  def test_strong_raw_opposition_cancels_without_pressed_filter(self):
    self.begin()
    self.cs.steeringTorque = -250.
    self.tick()
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)
    self.assertEqual(self.helper.maneuver_end_reason, 'cancelled')
    self.assertEqual(self.helper.lane_change_ll_prob, 1.)
    self.assertEqual(self.helper.turn_direction, log.Desire.none)
    self.assertEqual(self.helper.maneuver_type, 'none')
    self.assertFalse(self.helper.auto_lane_change_enable)
    self.assertEqual(self.helper.carrot_lane_change_count, 0)

  def test_persistent_lighter_opposition_cancels(self):
    self.begin()
    self.cs.steeringTorque = -100.
    self.tick()
    self.assertNotEqual(self.helper.lane_change_state, log.LaneChangeState.off)
    self.tick()
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)

  def test_small_noise_and_brief_opposition_do_not_cancel(self):
    self.begin()
    self.cs.steeringTorque = -20.
    self.tick(5)
    self.assertNotEqual(self.helper.lane_change_state, log.LaneChangeState.off)
    self.cs.steeringTorque = -100.
    self.tick()
    self.cs.steeringTorque = 0.
    self.tick()
    self.assertNotEqual(self.helper.lane_change_state, log.LaneChangeState.off)

  def test_right_change_cancel_is_symmetric(self):
    self.begin('right')
    self.cs.steeringTorque = 250.
    self.tick()
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertEqual(self.helper.maneuver_end_reason, 'cancelled')

  def test_cancellation_uses_active_direction_after_stalk_release(self):
    self.begin('right')
    self.cs.rightBlinker = False
    self.cs.steeringTorque = 250.
    self.tick()
    self.assertEqual(self.helper.maneuver_end_reason, 'cancelled')

  def test_same_direction_torque_does_not_cancel(self):
    self.begin()
    self.cs.steeringPressed, self.cs.steeringTorque = True, 300.
    self.tick(5)
    self.assertNotEqual(self.helper.lane_change_state, log.LaneChangeState.off)

  def test_cancel_stays_latched_with_same_blinker(self):
    self.begin()
    self.cs.steeringTorque = -250.
    self.tick()
    self.cs.steeringTorque = 300.
    self.cs.steeringPressed = True
    self.tick(40)
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)

  def test_already_moving_at_late_blinker_is_not_a_new_change(self):
    for i in range(12):
      self.geometry(.1*i)
      self.tick()
    self.cs.leftBlinker = True
    self.tick(10)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)
    self.assertTrue(self.helper.driver_request_consumed)

  def test_blinker_just_after_crossing_does_not_request_next_lane(self):
    self.geometry(1.2)
    self.tick(4)
    self.geometry(-1.3)
    self.tick()
    self.cs.leftBlinker = True
    self.tick(10)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)

  def test_recenter_countersteer_after_crossing_completes_in_new_lane(self):
    self.begin()
    self.geometry(1.2)
    self.tick(4)
    self.geometry(-1.3)
    self.tick()
    self.assertTrue(self.helper.lane_change_crossed)
    self.cs.steeringTorque = -100.
    self.tick(2)
    self.assertEqual(self.helper.maneuver_end_reason, 'completed')
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)
    self.assertEqual(self.model.laneLines[1].y[0], -3.1)

  def test_merging_state_can_finish(self):
    self.begin()
    self.helper.lane_change_state = log.LaneChangeState.laneChangeMerging
    self.helper.lane_change_ll_prob = .05
    self.tick(30, probability=0.)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)
    self.assertEqual(self.helper.maneuver_end_reason, 'completed')

  def test_turn_desire_is_not_blocked_as_late_lane_change(self):
    self.cs.vEgo = 3.
    self.model.meta.desireState[1] = .3
    self.cs.leftBlinker = True
    self.tick()
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)
    self.cs.steeringPressed, self.cs.steeringTorque = True, -300.
    self.tick()
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertEqual(self.helper.turn_direction, log.Desire.none)

  def test_right_crossing_countersteer_completes_in_current_lane(self):
    self.begin('right')
    self.geometry(-1.2)
    self.tick(4)
    self.geometry(1.3)
    self.tick()
    self.assertTrue(self.helper.lane_change_crossed)
    self.cs.steeringTorque = 100.
    self.tick(2)
    self.assertEqual(self.helper.maneuver_end_reason, 'completed')
    self.assertEqual(self.helper.desire, log.Desire.none)

  def test_stable_offset_is_not_a_late_moving_blinker(self):
    self.geometry(1.1)
    self.tick(20)
    self.begin()

  def test_torque_required_setting_still_waits(self):
    self.helper.laneChangeNeedTorque = 1
    self.cs.leftBlinker = True
    self.tick(4)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.preLaneChange)
    self.cs.steeringPressed, self.cs.steeringTorque = True, 300.
    self.tick()
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.laneChangeStarting)

  def test_blindspot_block_setting_still_blocks(self):
    self.helper.laneChangeBsd = 1
    self.cs.leftBlinker = self.cs.leftBlindspot = True
    self.cs.steeringPressed, self.cs.steeringTorque = True, 300.
    self.tick(5)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.preLaneChange)

  def test_same_blinker_cannot_recreate_cancelled_turn(self):
    self.cs.vEgo = 3.
    self.model.meta.desireState[1] = .3
    self.cs.leftBlinker = True
    self.tick()
    self.cs.steeringTorque = -250.
    self.tick()
    self.cs.steeringTorque = 0.
    self.tick(20)
    self.assertEqual(self.helper.desire, log.Desire.none)

  def test_same_atc_request_cannot_restart_cancelled_change(self):
    self.carrot.atcType = 'fork left'
    self.tick(4)
    # Force the active state, independent of automatic-start lane availability.
    self.helper.lane_change_state = log.LaneChangeState.laneChangeStarting
    self.helper.lane_change_direction = log.LaneChangeDirection.left
    self.helper.maneuver_type = 'lane_change'
    self.cs.steeringPressed, self.cs.steeringTorque = True, -300.
    self.tick()
    self.cs.steeringPressed, self.cs.steeringTorque = False, 0.
    self.tick(20)
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertEqual(self.helper.lane_change_state, log.LaneChangeState.off)

  def test_lateral_inactive_does_not_start(self):
    self.cs.leftBlinker = True
    self.tick(10, active=False)
    self.assertEqual(self.helper.desire, log.Desire.none)


if __name__ == '__main__':
  unittest.main()
