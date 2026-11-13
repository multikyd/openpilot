import unittest
from openpilot.selfdrive.carrot.bump_learning import BumpMemory, offset

class TestLocalBumpEligibility(unittest.TestCase):
  def make(self, passes=0, impact=False, cap=50, cross=0, physical=0):
    m=BumpMemory();lat,lon=offset(37.,127.,0.,40)
    if cross:lat,lon=offset(lat,lon,90.,cross)
    m.records=[dict(lat=lat,lon=lon,heading=0.,target=30,confirmed=True,passes=passes,smooth=0,
                    impact_learned=impact,manual_cap=cap,physical_passes=physical)]
    m.gps(100.,37.,127.,0.,1.)
    return m

  def test_navigation_cache_alone_does_not_slow_again(self):
    self.assertIsNone(self.make().tick(100.,30.))
    self.assertIsNone(self.make(passes=20).tick(100.,30.))

  def test_live_navigation_still_works(self):
    self.assertIsNotNone(self.make().tick(100.,30.,nav_distance=40.))

  def test_unrelated_live_bump_does_not_validate_cached_point(self):
    m=self.make();m.tick(100.,30.,nav_distance=150.)
    self.assertEqual(len(m.records),1)
    self.assertIsNone(m.tick(100.,30.,nav_distance=150.))

  def test_independent_repeated_impact_remains_active(self):
    self.assertIsNone(self.make(passes=1,impact=True).tick(100.,30.))
    self.assertIsNotNone(self.make(passes=2,impact=True,physical=2).tick(100.,30.))

  def test_manual_repeated_slowdown_does_not_confirm_physical_bump(self):
    self.assertIsNone(self.make(passes=2,cap=20).tick(100.,30.))

  def test_adjacent_lane_record_does_not_apply_without_navigation(self):
    self.assertIsNone(self.make(passes=3,impact=True,cross=4.).tick(100.,30.))

  def test_records_preserved_for_future_observation(self):
    m=self.make();before=m.export();m.tick(100.,30.)
    self.assertEqual(m.export(),before)

  def test_live_smooth_bump_progresses_to_40_and_keeps_learned_target(self):
    m=self.make();r=m.records[0]
    for i in range(9):
      m.finish_pass(r,1.,0,100,r['target'],False,True,200.+i*100.)
      applied=m.tick(100.,30.,nav_distance=40.)
      self.assertEqual(applied['target'],r['target'])
      self.assertEqual(r['target'],min(40,30+5*((i+1)//3)))

  def test_manual_cap_and_rough_pass_still_reduce_target(self):
    m=self.make(cap=20);r=m.records[0];r['target']=20
    for i in range(3):m.finish_pass(r,1.,0,100,20.,False,True,200.+i*100.)
    self.assertEqual(r['target'],20.)
    m.finish_pass(r,4.,3,100,20.,False,True,600.)
    self.assertEqual(r['target'],15.)

  def test_missing_sensor_samples_do_not_raise_speed(self):
    m=self.make();r=m.records[0]
    for i in range(6):m.finish_pass(r,0.,0,5,30.,False,True,200.+i*100.)
    self.assertEqual(r['target'],30)

  def test_legacy_target_above_40_is_bounded(self):
    m=self.make();m.records[0]['target']=50.
    restored=BumpMemory(m.export())
    self.assertEqual(restored.records[0]['target'],40.)
