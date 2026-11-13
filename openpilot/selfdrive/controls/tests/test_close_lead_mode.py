import unittest
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.close_lead_mode import CloseLeadModelMode


class TestCloseLeadMode(unittest.TestCase):
  def setUp(self):
    self.mode = CloseLeadModelMode()

  def step(self, t, speed=6., distance=10., probs=(.4, .8, .85, .4), status=True, fresh=True, enabled=True):
    return self.mode.update(t, speed, NS(present=status, dRel=distance), probs, fresh=fresh, enabled=enabled)

  def activate(self):
    self.assertFalse(self.step(0.))
    self.assertTrue(self.step(.11))

  def test_requires_all_entry_evidence(self):
    for kwargs in (dict(speed=10.), dict(distance=40.), dict(status=False),
                   dict(probs=(.5, .99, .99, .5)), dict(fresh=False)):
      with self.subTest(kwargs=kwargs):
        self.mode = CloseLeadModelMode()
        self.assertFalse(self.step(0., **kwargs))
        self.assertFalse(self.step(1., **kwargs))

  def test_brief_loss_does_not_switch(self):
    self.step(0.)
    self.assertFalse(self.step(.05))
    self.assertFalse(self.step(.06, probs=(.5, .99, .99, .5)))
    self.assertFalse(self.step(.2))

  def test_stop_and_go_and_lane_flash_stay_model(self):
    self.activate()
    for t in (1., 2., 4.):
      self.assertTrue(self.step(t, speed=0., probs=(.5, .99, .99, .5)))
    self.assertTrue(self.step(5., status=False, probs=(.5, .99, .99, .5)))
    self.assertTrue(self.step(6., status=False, probs=(.5, .99, .99, .5)))
    self.assertTrue(self.step(6.5))

  def test_recovery_requires_two_continuous_seconds(self):
    self.activate()
    self.assertTrue(self.step(1., distance=50., probs=(.5, .99, .99, .5)))
    self.assertTrue(self.step(2.9, distance=50., probs=(.5, .99, .99, .5)))
    self.assertFalse(self.step(3.01, distance=50., probs=(.5, .99, .99, .5)))

  def test_speed_recovery_and_stale_breaks_timer(self):
    self.activate()
    self.assertTrue(self.step(1., speed=12.))
    self.assertTrue(self.step(2., speed=12., fresh=False))
    self.assertTrue(self.step(4., speed=12.))
    self.assertFalse(self.step(6.01, speed=12.))

  def test_disengage_or_lane_change_resets(self):
    self.activate()
    self.assertFalse(self.step(1., enabled=False))
    self.assertFalse(self.step(1.01))

  def test_invalid_input_cannot_trigger(self):
    for kwargs in (dict(distance=float('nan')), dict(speed=float('nan')),
                   dict(probs=(.4, float('nan'), .8, .4))):
      self.mode = CloseLeadModelMode()
      self.assertFalse(self.step(0., **kwargs))
      self.assertFalse(self.step(2., **kwargs))


if __name__ == '__main__':
  unittest.main()
