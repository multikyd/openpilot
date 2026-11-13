import unittest
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD


class TestCommDiagnostics(unittest.TestCase):
  def setUp(self):
    self.subject = SelfdriveD.__new__(SelfdriveD)
    self.subject.logged_comm_issue = None
    self.subject.last_comm_issue_log_t = -float('inf')
    class State(dict):
      pass
    self.subject.sm = State(deviceState=SimpleNamespace(cpuUsagePercent=[42, 80]))
    self.subject.sm.logMonoTime = {'deviceState': 123000000000}
    self.subject.sm.valid = {'deviceState': True}
    self.module = SelfdriveD.log_comm_issue.__module__
    self.issue = {'invalid': ['cameraOdometry'], 'not_alive': [], 'not_freq_ok': []}

  def test_error_storm_is_bounded_and_latest_change_is_reported(self):
    with patch(self.module+'.cloudlog.event') as event, patch(self.module+'.time.monotonic') as clock:
      for i in range(300):
        clock.return_value = i/100.
        self.subject.log_comm_issue({**self.issue, 'invalid': [str(i)]})
      self.assertEqual(event.call_count, 3)
      self.assertEqual(event.call_args.kwargs['invalid'], ['200'])

  def test_unchanged_issue_does_not_repeat(self):
    with patch(self.module+'.cloudlog.event') as event, patch(self.module+'.time.monotonic', return_value=10.):
      self.subject.log_comm_issue(self.issue)
      self.subject.last_comm_issue_log_t = 0.
      self.subject.log_comm_issue(self.issue)
      self.assertEqual(event.call_count, 1)

  def test_repeated_recovery_does_not_bypass_rate_limit(self):
    with patch(self.module+'.cloudlog.event') as event, patch(self.module+'.time.monotonic') as clock:
      for i in range(100):
        clock.return_value = i/100.
        self.subject.logged_comm_issue = None
        self.subject.log_comm_issue(self.issue)
      self.assertEqual(event.call_count, 1)

  def test_cpu_snapshot_is_reused_without_process_filesystem_access(self):
    with patch(self.module+'.cloudlog.event') as event, patch(self.module+'.time.monotonic', return_value=10.), \
         patch('builtins.open', side_effect=AssertionError('diagnostics must not read files')), \
         patch(self.module+'.os.listdir', side_effect=AssertionError('diagnostics must not enumerate processes')):
      self.subject.log_comm_issue(self.issue)
      self.assertEqual(event.call_args.kwargs['cpu_per_core'], {'cpu0': 42, 'cpu1': 80})
      self.assertEqual(event.call_args.kwargs['cpu_sample_mono_time'], 123000000000)
      self.assertTrue(event.call_args.kwargs['error'])

  def test_invalid_cpu_sample_is_labelled(self):
    self.subject.sm.valid['deviceState'] = False
    with patch(self.module+'.cloudlog.event') as event, patch(self.module+'.time.monotonic', return_value=10.):
      self.subject.log_comm_issue(self.issue)
      self.assertFalse(event.call_args.kwargs['cpu_sample_valid'])


if __name__ == '__main__':
  unittest.main()
