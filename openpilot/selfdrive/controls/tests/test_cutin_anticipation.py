import unittest
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.cutin_anticipation import CutInAnticipation


def fixture():
  lane = lambda y: NS(x=[0., 20., 40., 80.], y=[y]*4)
  model = NS(laneLines=[lane(-5.4), lane(-1.8), lane(1.8), lane(5.4)],
             laneLineProbs=[.5, .99, .99, .5], laneLineStds=[.2]*4)
  return model


class TestCutInAnticipation(unittest.TestCase):
  def simulate(self, speed=20., start=-2.8, lateral=.6, relative=-3., distance=20., duration=1.5,
               curvature=0., enabled=True, confidence=.99, offset_error=0., changing_id=False, stale=False,
               normal_accel=1.6):
    helper, model, trace = CutInAnticipation(), fixture(), []
    model.laneLineProbs[1] = confidence
    for i in range(round(duration/.05)):
      t = i*.05
      y = start+lateral*t
      lead = NS(present=True, radar=True, radarTrackId=200+i if changing_id else 201,
                dRel=distance+relative*t, yRel=y, dPath=y+offset_error, vRel=relative,
                vLead=speed+relative, vLat=lateral)
      radar = NS(leadsLeft=[lead] if start > 0 else [], leadsRight=[lead] if start < 0 else [])
      cap = helper.update(radar, model, speed, normal_accel, curvature, t, t+.3 if stale else t, enabled)
      trace.append(cap)
    return helper, trace

  def test_medium_speed_cut_in_yields_then_coasts(self):
    helper, trace = self.simulate()
    self.assertEqual(helper.stage, 'coast')
    self.assertLess(trace[-1], 1.)

  def test_high_speed_cut_in(self):
    helper, trace = self.simulate(speed=32., distance=28., relative=-5.)
    self.assertEqual(helper.stage, 'coast')
    self.assertIsNotNone(trace[-1])

  def test_fast_closing_uses_gap_at_entry_instead_of_end_of_horizon(self):
    _, trace = self.simulate(speed=25., start=-2.5, lateral=.8, distance=12., relative=-8., duration=.9)
    self.assertTrue(any(x is not None for x in trace))

  def test_low_speed_close_inward_motion(self):
    helper, trace = self.simulate(speed=4., start=-3.2, lateral=.2, relative=-.1, distance=3., duration=1.)
    self.assertEqual(helper.stage, 'yield')
    self.assertLess(trace[-1], 1.6)

  def test_left_right_symmetry(self):
    _, left = self.simulate(start=2.8, lateral=-.6)
    _, right = self.simulate()
    self.assertEqual(left, right)

  def test_parallel_adjacent_traffic_is_unchanged(self):
    _, trace = self.simulate(lateral=0.)
    self.assertTrue(all(x is None for x in trace))

  def test_outward_motion_is_unchanged(self):
    _, trace = self.simulate(start=-2., lateral=-.5)
    self.assertTrue(all(x is None for x in trace))

  def test_pulling_away_fast_is_unchanged(self):
    _, trace = self.simulate(relative=4.)
    self.assertTrue(all(x is None for x in trace))

  def test_disabled_stale_low_confidence_and_curve_are_unchanged(self):
    for config in [dict(enabled=False), dict(stale=True), dict(confidence=.3), dict(curvature=.02),
                   dict(offset_error=1.), dict(speed=32., curvature=.002)]:
      with self.subTest(config=config):
        _, trace = self.simulate(**config)
        self.assertTrue(all(x is None for x in trace))

  def test_track_switches_cannot_accumulate_confirmation(self):
    _, trace = self.simulate(changing_id=True)
    self.assertTrue(all(x is None for x in trace))

  def test_caps_change_gradually(self):
    _, trace = self.simulate(duration=2.)
    values = [1.6]+[x for x in trace if x is not None]
    self.assertTrue(all(b-a >= -.7*.05-1e-8 for a, b in zip(values, values[1:])))
    self.assertGreaterEqual(min(values), -.3)

  def test_missing_track_releases_smoothly(self):
    helper, trace = self.simulate(duration=2.)
    model = fixture()
    previous = trace[-1]
    for i in range(150):
      t = 2.+i*.05
      value = helper.update(NS(leadsLeft=[], leadsRight=[]), model, 20., 1.6, 0., t, t, True)
      if value is not None:
        self.assertLessEqual(value-previous, .35*.05+1e-8)
        previous = value
    self.assertIsNone(value)

  def test_duplicate_radar_message_does_not_confirm(self):
    helper, model = CutInAnticipation(), fixture()
    lead = NS(present=True, radar=True, radarTrackId=201, dRel=20., yRel=-2.8, dPath=-2.8,
              vRel=-3., vLead=17., vLat=.6)
    for i in range(4):
      value = helper.update(NS(leadsLeft=[], leadsRight=[lead]), model, 20., 1.6, 0., 0., i*.05, True)
      self.assertIsNone(value)

  def test_nonfinite_input_and_geometry_rejected(self):
    for config in [dict(lateral=float('nan')), dict(curvature=float('nan'))]:
      _, trace = self.simulate(**config)
      self.assertTrue(all(x is None for x in trace))

  def test_lateral_velocity_without_path_approach_is_ignored(self):
    helper, model = CutInAnticipation(), fixture()
    lead = NS(present=True, radar=True, radarTrackId=201, dRel=20., yRel=-2.8, dPath=-2.8,
              vRel=-3., vLead=17., vLat=.6)
    for i in range(30):
      t = i*.05
      self.assertIsNone(helper.update(NS(leadsLeft=[], leadsRight=[lead]), model, 20., 1.6, .001, t, t, True))

  def test_existing_lower_limit_wins_and_mild_braking_is_bounded(self):
    helper, model = CutInAnticipation(), fixture()
    values = []
    for i in range(35):
      t = i*.05
      y = -2.8+.6*t
      lead = NS(present=True, radar=True, radarTrackId=201, dRel=20.-3.*t, yRel=y, dPath=y,
                vRel=-3., vLead=17., vLat=.6)
      value = helper.update(NS(leadsLeft=[], leadsRight=[lead]), model, 20., .2, 0., t, t, True, current_accel=.2)
      if value is not None:
        values.append(value)
    self.assertLess(min(values), 0.)
    self.assertGreaterEqual(min(values), -.3)
    self.assertTrue(all(x <= .2 for x in values))

  def test_bad_lane_geometry_clears_assistance(self):
    helper, _ = self.simulate()
    model = fixture()
    model.laneLines[1].x[1] = -1.
    self.assertIsNone(helper.update(NS(leadsLeft=[], leadsRight=[]), model, 20., 1.6, 0., 1.5, 1.5, True))

  def test_manual_intervention_clears_assistance_immediately(self):
    helper, _ = self.simulate()
    self.assertIsNone(helper.update(NS(leadsLeft=[], leadsRight=[]), fixture(), 20., 1.6, 0., 1.5, 1.5, False))
    self.assertFalse(helper.history)

  def test_close_strong_inward_motion_prepares_before_normal_confirmation(self):
    helper, trace = self.simulate(speed=20.,start=-2.6,lateral=.6,relative=-1.,distance=10.,duration=.3)
    self.assertIsNotNone(trace[-1])
    self.assertEqual(helper.stage,'prepare')
    self.assertGreaterEqual(trace[-1],0.)

  def test_distant_or_uncertain_candidate_has_no_short_confirmation(self):
    for options in (dict(distance=30.),dict(confidence=.75)):
      _, trace = self.simulate(speed=20.,start=-2.6,lateral=.6,relative=-1.,duration=.3,**options)
      self.assertTrue(all(x is None for x in trace))

  def test_recent_inward_reversal_prepares_without_premature_braking(self):
    helper,model = CutInAnticipation(),fixture()
    for i in range(20):
      t=i*.05;moving=max(0.,t-.65)
      y=-2.3-.4*min(t,.65)+.6*moving
      lead=NS(present=True,radar=True,radarTrackId=201,dRel=10.,yRel=y,dPath=y,
              vRel=0.,vLead=20.,vLat=.6 if moving>0 else -.4)
      cap=helper.update(NS(leadsLeft=[],leadsRight=[lead]),model,20.,1.6,0.,t,t,True,current_accel=.1)
    self.assertEqual(helper.stage,'prepare')
    self.assertIsNotNone(cap)
    self.assertGreaterEqual(cap,0.)

  def test_close_closing_cut_in_opens_room_before_lead_switch(self):
    helper, trace = self.simulate(speed=16., start=-3.2, lateral=.6, relative=-1.5,
                                  distance=8., duration=1.2, normal_accel=.4)
    self.assertEqual(helper.stage, 'coast')
    self.assertLess(trace[-1], -.5)
    self.assertGreaterEqual(trace[-1], -1.5)

  def test_lane_occlusion_continues_only_confirmed_target_briefly(self):
    helper, _ = self.simulate(speed=16., start=-3.2, lateral=.6, relative=-1.5,
                              distance=8., duration=1.2)
    model = fixture()
    model.laneLineStds[2] = .55
    def update(t, state=helper, key=201):
      y = -3.2+.6*t
      lead = NS(present=True, radar=True, radarTrackId=key, dRel=8.-1.5*t,
                yRel=y, dPath=y, vRel=-1.5, vLead=14.5, vLat=.6)
      return state.update(NS(leadsLeft=[], leadsRight=[lead]), model, 16., 1.6, 0., t, t, True)
    self.assertIsNotNone(update(1.2))
    self.assertEqual(helper.stage, 'coast')
    self.assertIsNone(update(1.75))
    fresh = CutInAnticipation()
    self.assertIsNone(update(1.2, fresh))

  def test_urgent_cut_in_keeps_stronger_planner_braking(self):
    helper, _ = self.simulate(speed=16., start=-3.2, lateral=.6, relative=-1.5,
                              distance=8., duration=1.2)
    lead = NS(present=True, radar=True, radarTrackId=201, dRel=6.2, yRel=-2.48,
              dPath=-2.48, vRel=-1.5, vLead=14.5, vLat=.6)
    cap = helper.update(NS(leadsLeft=[], leadsRight=[lead]), fixture(), 16., -3., 0., 1.2, 1.2, True)
    self.assertLessEqual(cap, -3.)

  def test_urgent_cut_in_is_symmetric_and_ramps_braking(self):
    options = dict(speed=16., relative=-1.5, distance=8., duration=1.2, normal_accel=.4)
    _, right = self.simulate(start=-3.2, lateral=.6, **options)
    _, left = self.simulate(start=3.2, lateral=-.6, **options)
    self.assertEqual(left, right)
    values = [.4]+[v for v in right if v is not None]
    self.assertTrue(all(b-a >= -1.5*.05-1e-8 for a, b in zip(values, values[1:])))

  def test_cached_lane_cannot_confirm_a_different_vehicle(self):
    helper, _ = self.simulate(speed=16., start=-3.2, lateral=.6, relative=-1.5,
                              distance=8., duration=1.2, normal_accel=.4)
    model = fixture()
    model.laneLineStds[2] = .55
    lead = NS(present=True, radar=True, radarTrackId=205, dRel=6.2, yRel=-2.48,
              dPath=-2.48, vRel=-1.5, vLead=14.5, vLat=.6)
    helper.update(NS(leadsLeft=[], leadsRight=[lead]), model, 16., .4, 0., 1.2, 1.2, True)
    self.assertIsNone(helper.track_id)
    self.assertNotIn(205, helper.history)


if __name__ == '__main__':
  unittest.main()

