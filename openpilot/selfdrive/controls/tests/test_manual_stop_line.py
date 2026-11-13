import copy
import unittest

from openpilot.selfdrive.carrot.manual_stop_line import ManualStopLineMemory, ManualStopLineCorrection


class TestManualStopLine(unittest.TestCase):
  def setUp(self):
    self.memory=ManualStopLineMemory()
    self.now=100.
    self.wall=1792300000.
    self.lat,self.lon=37.,127.

  def gps(self):
    return dict(valid=True,age=.01,accuracy=1.,lat=self.lat,lon=self.lon,stamp=self.now)

  def tick(self,speed=0.,brake=True,gas=False,mark=False,**kwargs):
    self.now+=.2
    self.wall+=.2
    values=dict(now=self.now,wall=self.wall,gps=self.gps(),speed=speed,heading=0.,
                movement='straight',red=True,no_lead=True,brake=brake,gas=gas,
                steering=0.,drive=True,bookmark_stamp=self.now if mark else None)
    values.update(kwargs)
    return self.memory.observe(**values)

  def visit(self):
    self.memory.reset()
    self.tick(speed=2.)
    for _ in range(12):self.tick()
    self.assertFalse(self.tick(mark=True))
    return self.tick(mark=True)

  def train(self):
    for seconds in (0,1200,86400):
      self.wall=1792300000.+seconds
      self.assertTrue(self.visit())

  def target(self,**kwargs):
    values=dict(now=self.now,wall=self.wall,gps=dict(self.gps(),lat=37.-15/111194.9266),
                speed=3.,heading=0.,movement='straight',red=True,no_lead=True)
    values.update(kwargs)
    return self.memory.target(**values)

  def test_manual_brake_and_final_confirmation_save(self):
    self.assertTrue(self.visit())
    self.assertEqual(len(self.memory.points[0]['samples']),1)

  def test_late_stop_without_confirmation_is_not_learned(self):
    for _ in range(40):self.tick()
    self.assertEqual(self.memory.points,[])

  def test_automatic_stop_without_pedal_is_not_learned(self):
    for _ in range(12):self.tick(brake=False)
    self.assertFalse(self.tick(brake=False,mark=True))
    self.assertFalse(self.tick(brake=False,mark=True))

  def test_single_issue_bookmark_does_not_confirm_position(self):
    for _ in range(12):self.tick()
    self.assertFalse(self.tick(mark=True))
    for _ in range(25):self.tick()
    self.assertFalse(self.tick(mark=True))
    self.assertEqual(self.memory.points,[])

  def test_forward_reposition_uses_final_location(self):
    for _ in range(12):self.tick()
    self.tick(speed=1.,brake=False,gas=True)
    self.lat+=2/111194.9266
    saved = any([self.tick() for _ in range(12)])
    if not saved:
      self.assertFalse(self.tick(mark=True))
      self.assertTrue(self.tick(mark=True))
    self.assertAlmostEqual(self.memory.points[0]['lat'],self.lat)

  def test_bookmark_before_reposition_is_not_reused(self):
    for _ in range(12):self.tick()
    old=self.now
    self.tick(speed=1.,gas=True,bookmark_stamp=old)
    for _ in range(12):self.tick(bookmark_stamp=old)
    self.assertFalse(self.tick(bookmark_stamp=old))

  def test_no_collection_with_lead_green_reverse_or_right_turn(self):
    for invalid in [dict(no_lead=False),dict(red=False),dict(drive=False),dict(movement='right')]:
      self.memory.reset()
      for _ in range(12):self.tick(**invalid)
      self.assertFalse(self.tick(mark=True,**invalid))
      self.assertFalse(self.tick(mark=True,**invalid))
    self.assertEqual(self.memory.points,[])

  def test_bad_or_stale_gps_rejected(self):
    for change in [dict(valid=False),dict(age=1.),dict(accuracy=3.)]:
      self.memory.reset()
      for _ in range(12):self.tick(gps=dict(self.gps(),**change))
      self.assertFalse(self.tick(mark=True,gps=dict(self.gps(),**change)))
      self.assertFalse(self.tick(mark=True,gps=dict(self.gps(),**change)))

  def test_frozen_gps_does_not_accumulate_samples(self):
    frozen=self.gps()
    for _ in range(12):self.tick(gps=frozen)
    self.assertFalse(self.tick(mark=True,gps=frozen))
    self.assertFalse(self.tick(mark=True,gps=frozen))

  def test_gps_jump_restarts_stationary_confirmation(self):
    for _ in range(12):self.tick()
    self.lat+=3/111194.9266
    self.assertFalse(self.tick(mark=True))

  def test_duplicate_swipes_do_not_add_visits(self):
    self.assertTrue(self.visit())
    for _ in range(12):self.tick()
    self.assertFalse(self.tick(mark=True))
    self.assertFalse(self.tick(mark=True))
    self.assertEqual(len(self.memory.points[0]['samples']),1)

  def test_one_visit_not_used_for_control(self):
    self.visit()
    self.assertIsNone(self.target())

  def test_three_visits_on_two_days_produce_target(self):
    self.train()
    target=self.target()
    self.assertEqual(target['samples'],3)
    self.assertAlmostEqual(target['distance'],15.,places=3)

  def test_all_visits_on_same_day_not_used(self):
    for _ in range(3):
      self.visit();self.wall+=1200
    self.assertIsNone(self.target())

  def test_other_direction_or_adjacent_lane_not_used(self):
    self.train()
    self.assertIsNone(self.target(heading=180.))
    self.assertIsNone(self.target(movement='left'))
    self.assertIsNone(self.target(gps=dict(self.gps(),lat=37.-15/111194.9266,lon=127.00005)))

  def test_known_overshot_position_not_saved(self):
    self.train();self.memory.reset();self.wall+=86400
    self.lat+=4/111194.9266
    for _ in range(12):self.tick()
    self.assertFalse(self.tick(mark=True))
    self.assertFalse(self.tick(mark=True))
    self.memory.reset();self.lat=37.+10/111194.9266
    for _ in range(12):self.tick()
    self.assertFalse(self.tick(mark=True))
    self.assertFalse(self.tick(mark=True))

  def test_saved_data_round_trip(self):
    self.train()
    restored=ManualStopLineMemory(self.memory.export())
    self.assertEqual(restored.export(),self.memory.export())

  def test_old_samples_expire(self):
    self.train();self.wall+=31*86400
    self.assertIsNone(self.target())

  def test_invalid_persisted_data_is_ignored(self):
    self.assertEqual(ManualStopLineMemory('{broken').points,[])
    self.assertEqual(ManualStopLineMemory(dict(version=999,points=[])).points,[])

  def test_deliberate_forward_correction_learns_without_swipes(self):
    for _ in range(12):self.tick(brake=False)
    self.tick(speed=1.,brake=False,gas=True)
    self.lat += 3/111194.9266
    saved = any([self.tick(brake=True) for _ in range(15)])
    self.assertTrue(saved)
    self.assertEqual(len(self.memory.points[0]['samples']),1)

  def test_forward_creep_without_final_braking_not_learned(self):
    for _ in range(12):self.tick(brake=False)
    self.tick(speed=1.,brake=False,gas=True)
    self.lat += 3/111194.9266
    for _ in range(20):self.tick(brake=False)
    self.assertEqual(self.memory.points,[])

  def test_large_or_sideways_reposition_not_learned_automatically(self):
    for lat_m, lon_m in [(15,0),(3,4)]:
      self.memory.reset();self.lat,self.lon=37.,127.
      for _ in range(12):self.tick(brake=False)
      self.tick(speed=1.,gas=True,brake=False)
      self.lat += lat_m/111194.9266;self.lon += lon_m/88000
      for _ in range(20):self.tick()
      self.assertEqual(self.memory.points,[])

  def test_model_hint_loss_during_reposition_does_not_discard_stop(self):
    for _ in range(12):self.tick(brake=False)
    self.tick(speed=1.,gas=True,brake=False,red=False)
    self.lat += 3/111194.9266
    self.assertTrue(any([self.tick(red=False) for _ in range(15)]))

  def test_live_green_cancels_reposition_learning(self):
    for _ in range(12):self.tick(brake=False)
    self.tick(speed=1.,gas=True,brake=False)
    self.lat += 3/111194.9266
    for _ in range(15):self.tick(red=False,trusted_green=True)
    self.assertEqual(self.memory.points,[])

  def test_long_model_hint_loss_does_not_learn(self):
    for _ in range(12):self.tick(brake=False)
    self.tick(speed=1.,gas=True,brake=False,red=False)
    self.lat += 3/111194.9266
    for _ in range(70):self.tick(speed=.2,red=False)
    for _ in range(15):self.tick(red=False)
    self.assertEqual(self.memory.points,[])


class TestManualStopCorrection(unittest.TestCase):
  def setUp(self):
    self.helper=ManualStopLineCorrection()
    self.target=dict(ts=100.,distance=15.,lateral=0.,samples=3,scatter=.1,accuracy=1.)

  def apply(self,stop=20.,active=True,target=None,now=100.,speed=3.):
    return self.helper.update(stop,self.target if target is None else target,now,speed,3.6,0.,active,.05)

  def test_can_only_shorten_and_at_most_one_metre(self):
    previous=20.
    for _ in range(200):
      result=self.apply()
      self.assertLessEqual(result,20.)
      self.assertGreaterEqual(result,19.)
      self.assertLessEqual(abs(result-previous),.01+1e-9)
      previous=result
    self.assertAlmostEqual(result,19.)

  def test_single_update_does_not_extend_early_model_stop(self):
    self.assertEqual(self.apply(stop=10.),10.)

  def test_clearance_conversion_matches_mpc_obstacle(self):
    for _ in range(200):result=self.apply(stop=19.)
    self.assertAlmostEqual(result,18.6)

  def test_lost_gps_does_not_release_a_shortened_stop(self):
    for _ in range(50):self.apply()
    before=self.helper.offset
    self.assertAlmostEqual(self.apply(target={}),20.+before)

  def test_no_stop_creation_or_restart(self):
    self.assertEqual(self.apply(active=False),20.)
    self.assertEqual(self.apply(stop=0.,speed=0.),0.)

  def test_stale_target_not_first_applied(self):
    self.assertEqual(self.apply(now=101.),20.)

  def test_driver_override_clears_episode(self):
    for _ in range(20):self.apply()
    self.apply(active=False)
    self.assertEqual(self.helper.offset,0.)

  def test_confirmed_target_slowly_advances_an_early_stop(self):
    results=[self.apply(stop=10.) for _ in range(100)]
    self.assertEqual(results[0],10.)
    self.assertGreater(results[-1],10.5)
    self.assertTrue(all(abs(b-a)<=.015+1e-9 for a,b in zip(results,results[1:])))
    self.assertLessEqual(results[-1],18.)

  def test_no_extension_near_standstill_or_large_mismatch(self):
    for speed, stop in [(0.,10.),(.5,10.),(3.,2.),(3.,1.)]:
      self.helper=ManualStopLineCorrection()
      for _ in range(100):result=self.apply(stop=stop,speed=speed)
      self.assertLessEqual(result,stop)

  def test_positive_offset_does_not_restart_stopped_vehicle(self):
    for _ in range(100):self.apply(stop=10.)
    self.assertGreater(self.helper.offset,0.)
    self.assertEqual(self.apply(stop=0.,speed=0.),0.)

  def test_invalid_target_cannot_begin_extension(self):
    for change in [dict(samples=1),dict(scatter=2.),dict(accuracy=4.),dict(lateral=2.)]:
      self.helper=ManualStopLineCorrection()
      for _ in range(100):result=self.apply(stop=10.,target=dict(self.target,**change))
      self.assertEqual(result,10.)

  def test_learned_point_also_relaxes_braking_envelope_gradually(self):
    from openpilot.selfdrive.controls.lib.signal_stop_approach import SignalStopApproach
    learned, baseline = SignalStopApproach(), SignalStopApproach()
    for i in range(40):
      prior=self.helper.offset
      corrected=self.apply(stop=10.,speed=1.5)
      if learned.distance is not None:
        learned.distance += self.helper.offset-prior
      actual=learned.update(corrected,1.5,-.5,2.,4.6,True,.05)
      original=baseline.update(10.,1.5,-.5,2.,4.6,True,.05)
    self.assertGreater(actual,original)
    self.assertGreater(learned.accel_cap,baseline.accel_cap)
    self.assertLess(learned.accel_cap,0.)


if __name__=='__main__':
  unittest.main()
