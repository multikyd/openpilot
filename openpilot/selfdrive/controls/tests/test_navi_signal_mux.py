import unittest
from openpilot.selfdrive.kisapilot.nav_transport import NaviSourceMux


class TestNaviSignalMux(unittest.TestCase):
  def setUp(self):
    self.now, self.events, self.changes = 100., [], []
    self.mux = NaviSourceMux(self.events.append, lambda *args: self.changes.append(args), clock=lambda: self.now)
    self.mux.open('t', ('phone', 1))
    self.mux.open('n', ('phone', 2))
    self.seq = {'t': 0, 'n': 0}

  def feed(self, key, signal=None, active=True, **extra):
    self.seq[key] += 1
    packet = dict(source='tmap' if key == 't' else 'naver-map', timestamp_ms=123,
      navi_state=dict(instance_id=key, sequence=self.seq[key], active=active, foreground=key == 't'))
    if signal is not None:
      packet['ssinf'] = dict(lat=37.5, lon=126.7, distance=100, straight=signal, straight_remain_time=20)
    packet.update(extra)
    self.mux.feed(key, packet)

  def test_background_first_then_other_new_signal(self):
    self.feed('n', 'RED_LIGHT_ON', active=False)
    self.feed('t', 'GREEN_LIGHT_ON')
    self.assertEqual([x['source'] for x in self.events], ['naver-map', 'tmap'])

  def test_unavailable_background_does_not_clear_valid_signal(self):
    self.feed('t', 'RED_LIGHT_ON')
    self.feed('n', 'UNAVAILABLE', active=False)
    self.assertEqual(len(self.events), 1)

  def test_old_local_countdown_cannot_overwrite_new_other_provider(self):
    self.feed('n', 'GREEN_LIGHT_ON', active=False)
    self.now += .1
    self.feed('t', 'RED_LIGHT_ON', signal_update=dict(kind='local_countdown'))
    self.assertEqual(len(self.events), 1)
    self.feed('t', 'RED_LIGHT_ON')
    self.assertEqual(self.events[-1]['source'], 'tmap')

  def test_source_switch_does_not_replay_cached_signal_or_background_tbt(self):
    rg = dict(vpPosPointLat=37.5, vpPosPointLon=126.7)
    self.feed('n', 'RED_LIGHT_ON', active=False, rgdata=rg)
    self.feed('t', rgdata=rg)
    self.feed('n', active=True)
    signals = [p for p in self.events if 'ssinf' in p]
    positions = [p['source'] for p in self.events if 'rgdata' in p]
    self.assertEqual(len(signals), 1)
    self.assertEqual(positions, ['tmap'])

  def test_expired_phase_and_reordered_packets_are_ignored(self):
    self.feed('n', 'RED_LIGHT_ON', signal_update=dict(status='phase_expired'))
    self.assertEqual(self.events, [])
    self.feed('n', 'GREEN_LIGHT_ON')
    self.seq['n'] -= 1
    self.feed('n', 'RED_LIGHT_ON')
    self.assertEqual(len(self.events), 1)

  def test_signal_metadata_retained(self):
    raw = dict(states=[dict(movement=1, lightState=3)])
    self.feed('n', 'RED_LIGHT_ON', raw_ssinf=raw)
    self.assertEqual(self.events[-1]['raw_ssinf'], raw)


if __name__ == '__main__':
  unittest.main()
