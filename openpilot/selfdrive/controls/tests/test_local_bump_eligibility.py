import unittest
from openpilot.selfdrive.carrot.bump_learning import BumpMemory, offset

class TestLocalBumpEligibility(unittest.TestCase):
  def make(self, passes=0, impact=False, cap=50, cross=0):
    m=BumpMemory();lat,lon=offset(37.,127.,0.,40)
    if cross:lat,lon=offset(lat,lon,90.,cross)
    m.records=[dict(lat=lat,lon=lon,heading=0.,target=30,confirmed=True,passes=passes,
                    impact_learned=impact,manual_cap=cap)]
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
    self.assertIsNotNone(self.make(passes=2,impact=True).tick(100.,30.))

  def test_manual_repeated_slowdown_remains_active(self):
    self.assertIsNotNone(self.make(passes=2,cap=20).tick(100.,30.))

  def test_adjacent_lane_record_does_not_apply_without_navigation(self):
    self.assertIsNone(self.make(passes=3,impact=True,cross=4.).tick(100.,30.))

  def test_records_preserved_for_future_observation(self):
    m=self.make();before=m.export();m.tick(100.,30.)
    self.assertEqual(m.export(),before)
