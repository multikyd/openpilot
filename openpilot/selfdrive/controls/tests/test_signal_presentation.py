import unittest
from types import SimpleNamespace

from openpilot.selfdrive.carrot.signal_memory import movement
# Load the graphics-independent helper without importing the onroad widget package.
import importlib.util
from pathlib import Path
spec = importlib.util.spec_from_file_location('signal_presentation',
    Path(__file__).parents[2] / 'ui/mici/onroad/signal_presentation.py')
presentation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(presentation)
countdown_rows, signal_lamps = presentation.countdown_rows, presentation.signal_lamps
signal_panel_visible = presentation.signal_panel_visible


def display(**updates):
  values = dict(source='TMAP', movement='other', straightState=2, leftState=1,
                rightState=0, straightRemain=61.24, leftRemain=65.24, rightRemain=0.,
                operatingMode='NORMAL', estimateLow=0., estimateHigh=0., confidence=1., predictionState=0)
  values.update(updates)
  return SimpleNamespace(**values)


class TestSignalPresentation(unittest.TestCase):
  def test_model_and_learning_panels_are_hidden_without_mutating_observations(self):
    for source in ('', 'MODEL', 'LEARN', 'OTHER'):
      value = display(source=source)
      original = vars(value).copy()
      self.assertFalse(signal_panel_visible(value))
      self.assertEqual(vars(value), original)

  def test_live_colors_and_untimed_vision_remain_visible(self):
    for source in ('TMAP', 'VISION'):
      for direction in ('straight', 'left', 'right'):
        value = display(source=source, straightState=0, leftState=0, rightState=0,
                        straightRemain=0., leftRemain=0., rightRemain=0.)
        setattr(value, direction+'State', 2)
        self.assertTrue(signal_panel_visible(value))

  def test_empty_live_panel_is_hidden(self):
    for source in ('TMAP', 'VISION'):
      self.assertFalse(signal_panel_visible(display(source=source, straightState=0, leftState=0, rightState=0)))

  def test_observed_off_and_flashing_signals_remain_visible(self):
    for mode in ('OFF', 'FLASH'):
      self.assertTrue(signal_panel_visible(display(source='VISION', operatingMode=mode,
                                                 straightState=0, leftState=0, rightState=0)))

  def test_valid_estimate_and_wait_range_remain_visible(self):
    self.assertTrue(signal_panel_visible(display(source='EST', estimateLow=10., estimateHigh=15.,
                                               confidence=.65, predictionState=1)))
    self.assertTrue(signal_panel_visible(display(source='EST', estimateLow=0., estimateHigh=12.,
                                               confidence=.3, operatingMode='WAIT')))

  def test_invalid_estimates_are_hidden(self):
    for updates in (dict(estimateLow=float('nan')), dict(estimateHigh=float('inf')),
                    dict(estimateLow=20.), dict(estimateLow=-1.), dict(confidence=0.),
                    dict(predictionState=0), dict(estimateLow=0., estimateHigh=0.)):
      value = dict(source='EST', estimateLow=10., estimateHigh=15., confidence=.65, predictionState=1)
      value.update(updates)
      self.assertFalse(signal_panel_visible(display(**value)))

  def test_left_red_does_not_light_straight_red(self):
    self.assertEqual(signal_lamps(display()), (0, 0, 2))

  def test_straight_red_and_left_green_are_distinct(self):
    self.assertEqual(signal_lamps(display(straightState=1, leftState=2)), (1, 2, 0))

  def test_right_red_does_not_light_straight_red(self):
    self.assertEqual(signal_lamps(display(movement='right', rightState=1)), (0, 0, 2))

  def test_yellow_is_shown_in_its_own_direction(self):
    self.assertEqual(signal_lamps(display(straightState=3, leftState=3)), (3, 0, 0))

  def test_four_road_signal_combinations(self):
    for straight, left, expected in ((1, 1, (1, 0, 0)), (1, 2, (1, 2, 0)),
                                      (2, 2, (0, 2, 2)), (2, 1, (0, 0, 2))):
      with self.subTest(straight=straight, left=left):
        self.assertEqual(signal_lamps(display(straightState=straight, leftState=left)), expected)

  def test_model_estimates_do_not_light_live_lamps(self):
    self.assertEqual(signal_lamps(display(source='EST')), (0, 0, 0))

  def test_unknown_direction_keeps_both_labelled_live_times(self):
    self.assertEqual(countdown_rows(display()), [('S', 2, '62'), ('L', 1, '66')])

  def test_selected_left_uses_left_time(self):
    self.assertEqual(countdown_rows(display(movement='left')), [('', 1, '66')])

  def test_observed_uses_straight_time(self):
    self.assertEqual(countdown_rows(display(movement='observed')), [('', 2, '62')])

  def test_three_digit_values_and_missing_timer(self):
    self.assertEqual(countdown_rows(display(leftRemain=100., straightRemain=0.)),
                     [('S', 2, '--'), ('L', 1, '100')])

  def test_invalid_timer_is_not_invented(self):
    for value in (float('nan'), float('inf'), -3.):
      self.assertEqual(countdown_rows(display(movement='straight', straightRemain=value))[0][2], '--')

  def test_distant_unhandled_route_instruction_is_not_this_signal(self):
    for turn, distance, turn_distance in ((120, 87, 2589), (120, 119, 451), (121, 100, 700)):
      self.assertEqual(movement(dict(turn=turn, turn_distance=turn_distance), distance), 'straight')

  def test_near_unknown_instruction_remains_ambiguous(self):
    self.assertEqual(movement(dict(turn=120, turn_distance=100), 100), 'other')

  def test_turn_at_current_signal_is_preserved(self):
    self.assertEqual(movement(dict(turn=12, turn_distance=100), 100), 'left')
    self.assertEqual(movement(dict(turn=13, turn_distance=100), 100), 'right')
    self.assertEqual(movement(dict(turn=14, turn_distance=100), 100), 'other')

  def test_turn_before_signal_remains_ambiguous(self):
    self.assertEqual(movement(dict(turn=12, turn_distance=50), 120), 'other')

  def test_matched_lane_point_overrides_distant_instruction(self):
    nav=dict(turn=120, turn_distance=900, lane_points=[dict(lat=37., lon=127., distance=100., movement='left')])
    self.assertEqual(movement(nav, 100., 37., 127.), 'left')

  def test_conflicting_lane_points_stay_ambiguous(self):
    nav=dict(turn=120, turn_distance=900, lane_points=[dict(lat=37., lon=127., distance=100., movement=d)
                                                    for d in ('straight', 'left')])
    self.assertEqual(movement(nav, 100., 37., 127.), 'other')


if __name__ == '__main__':
  unittest.main()
