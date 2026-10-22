import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import Mock, patch

from openpilot.selfdrive.carrot import auto_crash_capture as auto
from openpilot.selfdrive.carrot import collect_crash as collector


class TestAutoCrashCapture(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.root = Path(self.temp.name)
    self.events = self.root / 'events'
    self.logs = self.root / 'logs'
    self.logs.mkdir()
    self.route = self.root / 'CurrentRoute'
    self.route.write_text('route--abc')
    self.spawn = Mock()
    self.child = Mock()
    self.child.poll.return_value = None
    self.spawn.return_value = self.child
    self.service = auto.AutoCrashCapture(self.events, self.logs, self.spawn, self.route)

  def record(self):
    self.service.record('carrot_man', 123, -6, 'SIGABRT')
    self.service.step()
    return self.service.pending()[0]

  def test_background_service_flushes_without_manual_command(self):
    import time
    self.service.start()
    self.addCleanup(self.service.close)
    self.service.record('carrot_man', 123, -6, 'SIGABRT')
    deadline = time.monotonic() + 3
    while not self.service.pending() and time.monotonic() < deadline:
      time.sleep(.01)
    self.assertEqual(len(self.service.pending()), 1)
    self.spawn.assert_not_called()

  def test_child_guard_stops_for_missing_offroad_state(self):
    with patch.object(auto.os, 'getppid', return_value=10), patch.object(auto.os, '_exit', side_effect=SystemExit(75)) as stop:
      with self.assertRaises(SystemExit):
        auto.guard_offroad(10, self.root / 'missing')
    stop.assert_called_once_with(75)

  def test_child_guard_stops_when_onroad(self):
    state = self.root / 'IsOffroad'
    state.write_bytes(b'0')
    with patch.object(auto.os, 'getppid', return_value=10), patch.object(auto.os, '_exit', side_effect=SystemExit(75)):
      with self.assertRaises(SystemExit):
        auto.guard_offroad(10, state)

  def test_child_guard_stops_if_manager_exits(self):
    state = self.root / 'IsOffroad'
    state.write_bytes(b'1')
    with patch.object(auto.os, 'getppid', return_value=1), patch.object(auto.os, '_exit', side_effect=SystemExit(75)):
      with self.assertRaises(SystemExit):
        auto.guard_offroad(10, state)

  def test_onroad_event_is_durable_without_compression(self):
    event = self.record()
    self.spawn.assert_not_called()
    self.assertEqual(json.loads(event.read_text())['exit_code'], -6)
    self.assertEqual(json.loads(event.read_text())['route'], 'route--abc')

  def test_restart_finds_pending_event_then_collects_offroad(self):
    event = self.record()
    restored = auto.AutoCrashCapture(self.events, self.logs, self.spawn, self.route)
    self.assertEqual(restored.pending(), [event])
    restored.set_onroad(False)
    restored.step()
    self.assertIn(str(event), self.spawn.call_args.args[0])

  def test_active_collection_stops_when_driving_restarts(self):
    self.record()
    self.service.set_onroad(False)
    self.service.step()
    self.service.set_onroad(True)
    self.service.step()
    self.child.terminate.assert_called_once()

  def test_done_event_is_not_collected_again(self):
    event = self.record()
    auto.atomic_json(event.with_suffix('.done'), {'archive': 'saved'})
    self.service.set_onroad(False)
    self.service.step()
    self.spawn.assert_not_called()

  def test_two_exits_are_queued_without_replacing_each_other(self):
    self.record()
    self.service.record('modeld', 456, -11, 'SIGSEGV')
    self.service.step()
    self.assertEqual(len(self.service.pending()), 2)

  def test_failed_storage_keeps_event_for_retry(self):
    self.service.record('carrot_man', 123, -6, 'SIGABRT')
    with patch.object(auto, 'atomic_json', side_effect=OSError('disk full')):
      with self.assertRaises(OSError):
        self.service.step()
    self.service.step()
    self.assertEqual(len(self.service.pending()), 1)

  def test_failed_child_retries_later_without_removing_event(self):
    self.record()
    self.service.set_onroad(False)
    self.service.step()
    self.child.poll.return_value = 1
    self.service.step()
    self.assertEqual(self.spawn.call_count, 1)
    self.assertEqual(len(self.service.pending()), 1)

  def test_current_and_previous_segments_are_recorded_at_failure(self):
    for n in range(3):
      (self.logs / f'route--abc--{n}').mkdir()
    event = self.record()
    self.assertEqual(json.loads(event.read_text())['segments'], ['route--abc--1', 'route--abc--2'])
    self.assertEqual(collector.nearby_segments(self.logs, '../bad'), [])

  def test_automatic_archive_contains_event_fault_and_existing_logs(self):
    fault = self.root / 'carrot_crash'
    fault.mkdir()
    (fault / 'carrot-fault-test.log').write_text('fatal stack')
    segment = self.logs / 'route--abc--1'
    segment.mkdir()
    (segment / 'rlog.zst').write_bytes(b'original')
    event = dict(process='carrot_man', pid=123, exit_code=-6, wall=1234., segments=[segment.name])
    with patch.object(collector, 'command', return_value={'fixture': True}):
      archive = collector.collect(self.logs, roots=(fault,), event=event, automatic=True)
    with tarfile.open(archive) as bundle:
      report = json.load(bundle.extractfile('report.json'))
      self.assertEqual(report['event']['exit_code'], -6)
      self.assertTrue(any(name.endswith('rlog.zst') for name in bundle.getnames()))
      self.assertTrue(any(name.endswith('carrot-fault-test.log') for name in bundle.getnames()))
    self.assertFalse(archive.with_suffix('.gz.partial').exists())
    self.assertEqual((segment / 'rlog.zst').read_bytes(), b'original')

  def test_package_marks_done_and_limits_only_automatic_archives(self):
    event = self.record()
    manual = self.logs / 'carrot-crash-report-manual.tar.gz'
    manual.touch()
    for n in range(6):
      (self.logs / f'process-crash-report-{n}.tar.gz').touch()
    archive = self.logs / 'process-crash-report-new.tar.gz'
    archive.touch()
    with patch.object(collector, 'collect', return_value=archive), patch.object(auto.os, 'nice', create=True):
      auto.package_event(event, self.logs)
    self.assertTrue(event.with_suffix('.done').exists())
    self.assertEqual(len(list(self.logs.glob('process-crash-report-*.tar.gz'))), 5)
    self.assertTrue(manual.exists())

  def test_collection_failure_does_not_mark_done(self):
    event = self.record()
    with patch.object(collector, 'collect', side_effect=OSError('full')), patch.object(auto.os, 'nice', create=True):
      with self.assertRaises(OSError):
        auto.package_event(event, self.logs)
    self.assertFalse(event.with_suffix('.done').exists())

  def test_file_and_total_limits_preserve_oversize_original(self):
    fault = self.root / 'carrot_crash'
    fault.mkdir()
    original = fault / 'large.log'
    original.write_bytes(b'123456')
    chosen, skipped, total = collector.select_files((fault,), __import__('time').time(), max_file=4, max_total=4)
    self.assertEqual(chosen, [])
    self.assertTrue(skipped)
    self.assertTrue(original.exists())


class TestManagerCrashHook(unittest.TestCase):
  def method(self):
    import ast
    import signal
    source = Path(__file__).parents[3] / 'system/manager/process.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ManagerProcess')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'report_unexpected_exit')
    env = dict(signal=signal, time=__import__('time'), cloudlog=Mock())
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), env)
    return env['report_unexpected_exit']

  def owner(self, code=-6, shutting_down=False):
    from types import SimpleNamespace
    return SimpleNamespace(proc=SimpleNamespace(pid=123, exitcode=code),
                           shutting_down=shutting_down, _reported_exit_proc=None, name='carrot_man')

  def test_unexpected_exit_is_enqueued_once(self):
    service = Mock()
    import signal
    owner = self.owner(code=-int(signal.SIGABRT))
    method = self.method()
    with patch.object(auto, 'capture_service', return_value=service):
      method(owner)
      method(owner)
    service.record.assert_called_once_with('carrot_man', 123, -int(signal.SIGABRT), 'SIGABRT')

  def test_normal_shutdown_does_not_enqueue(self):
    with patch.object(auto, 'capture_service') as service:
      self.method()(self.owner(shutting_down=True))
    service.assert_not_called()

  def test_running_process_does_not_enqueue(self):
    with patch.object(auto, 'capture_service') as service:
      self.method()(self.owner(code=None))
    service.assert_not_called()


if __name__ == '__main__':
  unittest.main()
