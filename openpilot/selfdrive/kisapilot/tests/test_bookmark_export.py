import importlib.util
import ast
import os
import tempfile
import time
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

SOURCE = Path(__file__).parents[1] / 'bookmark_export.py'
spec = importlib.util.spec_from_file_location('bookmark_export', SOURCE)
b = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b)


class BookmarkFixture:
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.later = time.time() + 60

    def segment(self, route='old', number=0, locked=False):
        p = self.root / (route + '--' + str(number))
        p.mkdir()
        (p / 'rlog.zst').write_bytes(b'partial log')
        (p / 'fcamera.hevc').write_bytes(b'partial video')
        if locked:
            (p / 'rlog.lock').touch()
        return p

    def auto(self, recording, onroad=True):
        with patch.object(b.time, 'time', return_value=self.later):
            return b.auto_bookmarks(self.root, onroad, lambda p: p.name == 'old--0', recording)

    def export(self, recording):
        with patch.object(b.time, 'time', return_value=self.later):
            return b.export_bookmarks(self.root, ['old--0'], preserved=lambda p: p.name == 'old--0', recording=recording)


class BookmarkRecoveryTest(BookmarkFixture, unittest.TestCase):
    def test_reboot_with_old_lock_and_missing_next_exports_existing_files(self):
        p = self.segment(locked=True)
        self.assertTrue(self.auto(set(), False)['bookmarks'][0]['ready'])
        result = self.export(set())
        self.assertEqual(len(result['files']), 2)
        self.assertEqual(result['bookmark_metadata']['interrupted_segments'], ['old--0'])
        self.assertFalse(any('세그먼트 없음' in w for w in result['excluded']))
        self.assertTrue(any('미완성' in w for w in result['excluded']))
        self.assertTrue((p / 'rlog.lock').exists())
        self.assertEqual((p / 'rlog.zst').read_bytes(), b'partial log')

    def test_next_drive_does_not_wait_for_old_unfinished_next_segment(self):
        self.segment(locked=True)
        self.segment(number=1, locked=True)
        self.segment('new', locked=True)
        self.assertTrue(self.auto({'new'})['bookmarks'][0]['ready'])
        self.assertEqual(len(self.export({'new'})['files']), 2)

    def test_active_current_route_stays_locked(self):
        self.segment(locked=True)
        self.assertFalse(self.auto({'old'})['bookmarks'][0]['ready'])
        with self.assertRaises(ValueError):
            self.export({'old'})

    def test_unknown_recording_state_never_ignores_locks(self):
        self.segment(locked=True)
        self.assertFalse(self.auto(None)['bookmarks'][0]['ready'])
        with self.assertRaises(ValueError):
            self.export(None)

    def test_recent_files_keep_lock_protection(self):
        self.segment(locked=True)
        self.assertTrue(b.snapshot(self.root, recording=set())['old--0']['active'])

    def test_closed_current_bookmark_does_not_wait_for_future_segment(self):
        self.segment()
        self.assertTrue(self.auto({'old'})['bookmarks'][0]['ready'])

    def test_ended_closed_route_can_finish_without_next_segment(self):
        self.segment()
        self.assertTrue(self.auto({'new'})['bookmarks'][0]['ready'])

    def test_active_next_segment_is_not_required(self):
        self.segment()
        self.segment(number=1, locked=True)
        self.assertTrue(self.auto({'old'})['bookmarks'][0]['ready'])
        self.assertEqual(self.export({'old'})['bookmark_metadata']['included_segments'], ['old--0'])

    def test_missing_recorder_state_while_onroad_is_unknown(self):
        proc = self.root / 'proc'
        proc.mkdir()
        self.assertIsNone(b.recording_routes(self.root, True, proc_root=proc))
        self.assertEqual(b.recording_routes(self.root, False, proc_root=proc), set())
        self.assertEqual(b.recording_routes(self.root, True, 'new', proc_root=proc), {'new'})

    def test_open_old_recorder_files_remain_protected_even_on_new_drive(self):
        proc = self.root / 'proc'
        fd = proc / '42' / 'fd'
        fd.mkdir(parents=True)
        (fd.parent / 'comm').write_text('loggerd')
        (fd / '1').touch()
        with patch.object(b.os, 'readlink', return_value=str(self.root / 'old--0' / 'rlog.zst')):
            self.assertEqual(b.recording_routes(self.root, True, 'new', proc), {'old', 'new'})

    def test_recorder_fd_permission_failure_is_unknown(self):
        proc = self.root / 'proc'
        fd = proc / '42' / 'fd'
        fd.mkdir(parents=True)
        (fd.parent / 'comm').write_text('encoderd')
        (fd / '1').touch()
        with patch.object(b.os, 'readlink', side_effect=PermissionError):
            self.assertIsNone(b.recording_routes(self.root, False, proc_root=proc))

    def test_unreadable_proc_is_unknown(self):
        self.assertIsNone(b.recording_routes(self.root, False, proc_root=self.root / 'missing'))


class BookmarkHistoryTest(BookmarkFixture, unittest.TestCase):
    def export_at(self, offset, number=1, adjacent=1):
        with patch.object(b, 'segment_bookmark_offset', return_value=offset):
            return b.export_bookmarks(self.root, [f'old--{number}'], adjacent,
                                     preserved=lambda p: p.name == f'old--{number}', recording={'old'})

    def test_before_30_includes_previous_but_not_next(self):
        for number in range(3):
            self.segment(number=number)
        result = self.export_at(29.999)
        self.assertEqual(result['bookmark_metadata']['included_segments'], ['old--0', 'old--1'])
        self.assertEqual(result['bytes'], 48)

    def test_at_or_after_30_only_current_even_with_legacy_adjacent_two(self):
        for number in range(3):
            self.segment(number=number)
        for offset in (30., 45., 59.999):
            for adjacent in (1, 2):
                with self.subTest(offset=offset, adjacent=adjacent):
                    result = self.export_at(offset, adjacent=adjacent)
                    self.assertEqual(result['bookmark_metadata']['included_segments'], ['old--1'])
                    self.assertEqual(result['bookmark_metadata']['bookmark_offsets_seconds'], {'old--1': offset})

    def test_unknown_offset_keeps_previous(self):
        self.segment(number=0)
        self.segment(number=1)
        self.assertEqual(self.export_at(None)['bookmark_metadata']['included_segments'], ['old--0', 'old--1'])

    def test_first_segment_never_requests_negative_segment(self):
        self.segment()
        result = self.export_at(0., number=0)
        self.assertEqual(result['bookmark_metadata']['included_segments'], ['old--0'])
        self.assertEqual(result['excluded'], [])

    def test_missing_previous_is_reported_without_requesting_next(self):
        self.segment(number=1)
        result = self.export_at(10.)
        self.assertEqual(result['bookmark_metadata']['included_segments'], ['old--1'])
        self.assertEqual(len(result['excluded']), 1)
        self.assertTrue(result['excluded'][0].startswith('old--0'))

    def test_required_previous_must_be_closed(self):
        self.segment(number=0, locked=True)
        self.segment(number=1)
        for offset, ready in ((10., False), (45., True)):
            with self.subTest(offset=offset), patch.object(b, 'segment_bookmark_offset', return_value=offset):
                result = b.auto_bookmarks(self.root, True, lambda p: p.name == 'old--1', {'old'})
                self.assertEqual(result['bookmarks'][0]['ready'], ready)
        with self.assertRaises(ValueError):
            self.export_at(10.)
        self.assertEqual(self.export_at(45.)['bookmark_metadata']['included_segments'], ['old--1'])

    def test_explicit_current_only_is_preserved(self):
        self.segment(number=0)
        self.segment(number=1)
        self.assertEqual(self.export_at(5., adjacent=0)['bookmark_metadata']['included_segments'], ['old--1'])

    def event(self, kind, seconds, sentinel_type=None):
        return SimpleNamespace(which=lambda: kind, logMonoTime=int(seconds * 1e9),
                               sentinel=SimpleNamespace(type=sentinel_type))

    def test_uses_segment_sentinel_not_repeated_route_init_time(self):
        events = [self.event('initData', 100), self.event('sentinel', 700, 'startOfSegment'),
                  self.event('userBookmark', 729.999)]
        self.assertAlmostEqual(b.bookmark_offset(events), 29.999)

    def test_multiple_bookmarks_use_earliest_event_including_bookmark_button(self):
        events = [self.event('sentinel', 100, 'startOfRoute'), self.event('userBookmark', 145),
                  self.event('bookmarkButton', 110)]
        self.assertEqual(b.bookmark_offset(events), 10.)

    def test_missing_or_invalid_times_are_unknown(self):
        for events in ([self.event('userBookmark', 50)],
                       [self.event('sentinel', 100, 'startOfSegment')],
                       [self.event('sentinel', 100, 'startOfSegment'), self.event('userBookmark', 99)],
                       [self.event('sentinel', 100, 'startOfSegment'), self.event('userBookmark', 165)]):
            with self.subTest(events=events):
                self.assertIsNone(b.bookmark_offset(events))

    def test_qlog_is_used_and_decode_failure_falls_back_to_rlog(self):
        p = self.segment(number=1)
        (p / 'qlog.zst').write_bytes(b'qlog')
        item = b.snapshot(self.root, preserved=lambda p: True)['old--1']
        with patch.object(b, '_log_bookmark_offset', return_value=35.) as reader:
            self.assertEqual(b.segment_bookmark_offset(self.root, item), 35.)
            self.assertEqual(reader.call_count, 1)
            self.assertTrue(reader.call_args.args[0].endswith('qlog.zst'))
        with patch.object(b, '_log_bookmark_offset', side_effect=[ValueError('bad qlog'), 20.]):
            self.assertEqual(b.segment_bookmark_offset(self.root, item), 20.)

    def test_active_logs_are_never_decoded(self):
        self.segment(locked=True)
        item = b.snapshot(self.root, preserved=lambda p: True, recording={'old'})['old--0']
        with patch.object(b, '_log_bookmark_offset') as reader:
            self.assertIsNone(b.segment_bookmark_offset(self.root, item))
            reader.assert_not_called()


class BookmarkArchiveTest(BookmarkFixture, unittest.TestCase):
    def zipped(self, offset=40.):
        with patch.object(b, 'segment_bookmark_offset', return_value=offset):
            return b.export_bookmarks(self.root, ['old--1'], preserved=lambda p: p.name == 'old--1', archive=True)

    def test_zip_contains_only_original_segment_files(self):
        self.segment(number=0)
        segment = self.segment(number=1)
        self.segment(number=2, locked=True)
        (segment / 'notes.txt').write_text('test\n' * 100, encoding='utf8')
        result = self.zipped(10.)
        self.assertEqual(len(result['files']), 1)
        archive = Path(result['files'][0]['path'])
        # Latest KisaAgent requires exactly two safe components below realdata.
        self.assertEqual(len(archive.relative_to(self.root).parts), 2)
        self.assertTrue(archive.name.endswith('.zip'))
        self.assertEqual(result['bytes'], archive.stat().st_size)
        with zipfile.ZipFile(archive) as bundle:
            self.assertIsNone(bundle.testzip())
            self.assertEqual(set(bundle.namelist()), {'old--0/rlog.zst', 'old--0/fcamera.hevc',
                             'old--1/rlog.zst', 'old--1/fcamera.hevc', 'old--1/notes.txt'})
            self.assertEqual(bundle.read('old--1/rlog.zst'), (segment / 'rlog.zst').read_bytes())
            self.assertEqual(bundle.read('old--1/fcamera.hevc'), (segment / 'fcamera.hevc').read_bytes())
            self.assertEqual(bundle.getinfo('old--1/fcamera.hevc').compress_type, zipfile.ZIP_STORED)
            self.assertEqual(bundle.getinfo('old--1/rlog.zst').compress_type, zipfile.ZIP_STORED)
            self.assertEqual(bundle.getinfo('old--1/notes.txt').compress_type, zipfile.ZIP_DEFLATED)
            self.assertNotIn('KisaAgent_bookmarks.json', bundle.namelist())

    def test_after_30_zip_only_has_current_segment(self):
        for number in range(3):
            self.segment(number=number)
        result = self.zipped()
        with zipfile.ZipFile(result['files'][0]['path']) as bundle:
            self.assertEqual(set(bundle.namelist()), {'old--1/rlog.zst', 'old--1/fcamera.hevc'})

    def test_cache_reuses_archive_for_resume_and_expires_only_old_copies(self):
        self.segment(number=1)
        first = self.zipped()
        path = Path(first['files'][0]['path'])
        old = path.parent / 'KisaAgent_bookmarks_expired.zip'
        old.write_bytes(b'expired')
        os.utime(old, (time.time() - 90000,) * 2)
        with patch.object(b.zipfile.ZipFile, 'write', side_effect=AssertionError('cache was not reused')):
            self.assertEqual(self.zipped()['files'], first['files'])
        self.assertFalse(old.exists())
        self.assertEqual((self.root / 'old--1/rlog.zst').read_bytes(), b'partial log')

    def test_rebuild_is_byte_identical_for_resume(self):
        self.segment(number=1)
        result = self.zipped()
        path = Path(result['files'][0]['path'])
        original = path.read_bytes()
        path.unlink()
        rebuilt = self.zipped()
        self.assertEqual(Path(rebuilt['files'][0]['path']).read_bytes(), original)

    def test_source_change_gets_new_archive_name(self):
        segment = self.segment(number=1)
        first = self.zipped()
        (segment / 'rlog.zst').write_bytes(b'new log bytes')
        self.assertNotEqual(self.zipped()['files'][0]['path'], first['files'][0]['path'])

    def test_cache_keeps_at_most_three_archives_without_touching_sources(self):
        segment = self.segment(number=1)
        for number in range(5):
            (segment / 'rlog.zst').write_bytes(str(number).encode('ascii'))
            result = self.zipped()
            self.assertTrue(Path(result['files'][0]['path']).exists())
        self.assertEqual(len(list((self.root / 'bookmark_exports').glob('*.zip'))), 3)
        self.assertEqual((segment / 'rlog.zst').read_bytes(), b'4')

    def test_build_failure_does_not_publish_partial_archive(self):
        self.segment(number=1)
        with patch.object(b.zipfile.ZipFile, 'write', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.zipped()
        self.assertEqual(list((self.root / 'bookmark_exports').iterdir()), [])
        self.assertEqual((self.root / 'old--1/rlog.zst').read_bytes(), b'partial log')

    def test_invalid_archive_option_and_active_segment_are_rejected(self):
        self.segment(number=1, locked=True)
        with self.assertRaises(ValueError):
            self.zipped()
        with self.assertRaises(ValueError):
            b.export_bookmarks(self.root, ['old--1'], archive='true')

    def test_archive_filename_is_kst_date_and_time_only(self):
        # 2026-10-30 15:05:06 UTC -> next day in Korea, regardless of the C4 timezone.
        from datetime import datetime, timezone
        stamp = datetime(2026, 10, 30, 15, 5, 6, tzinfo=timezone.utc).timestamp()
        self.assertEqual(b.bookmark_archive_name(stamp), '2026-10-31_00-05-06.zip')
        self.segment(number=1)
        with patch.object(b.time, 'time', return_value=stamp):
            result = self.zipped()
        self.assertEqual(result['archive']['name'], '2026-10-31_00-05-06.zip')
        self.assertEqual(result['archive']['name_timezone'], 'Asia/Seoul')

    def test_same_second_exports_do_not_overwrite_each_other(self):
        self.segment(number=0)
        self.segment(number=1)
        with patch.object(b.time, 'time', return_value=time.time() + .125):
            first = self.zipped(40.)
            first_bytes = Path(first['files'][0]['path']).read_bytes()
            second = self.zipped(10.)
        first_path, second_path = (Path(r['files'][0]['path']) for r in (first, second))
        self.assertNotEqual(first_path, second_path)
        self.assertEqual(first_path.read_bytes(), first_bytes)
        self.assertRegex(second_path.name, r'^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.\d{6}\.zip$')

    def test_old_manifest_archive_is_not_reused(self):
        self.segment(number=1)
        cache = self.root / 'bookmark_exports'
        cache.mkdir()
        with zipfile.ZipFile(cache / ('KisaAgent_bookmarks_' + 'a' * 24 + '.zip'), 'w') as old:
            old.writestr('KisaAgent_bookmarks.json', '{}')
        result = self.zipped()
        with zipfile.ZipFile(result['files'][0]['path']) as bundle:
            self.assertFalse(any(name.endswith('.json') for name in bundle.namelist()))
        self.assertEqual(list(cache.glob('*.json')), [])

    def test_export_api_defaults_to_zip_and_can_request_original_files(self):
        # Compile the real endpoint independently of device-only imports/startup.
        import sys
        from types import ModuleType
        agent = SOURCE.parent / 'kisa_agent.py'
        tree = ast.parse(agent.read_text(encoding='utf8'))
        endpoint = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare_bookmark_export')
        module = ast.Module(body=[endpoint], type_ignores=[])
        payload = {'ids': ['old--1']}
        app = SimpleNamespace(route=lambda *args, **kwargs: lambda function: function)
        request = SimpleNamespace(get_json=lambda **kwargs: payload)
        namespace = {'app': app, 'request': request, 'jsonify': lambda value: value,
                     'bookmark_recording_context': lambda: (False, set())}
        exec(compile(module, str(agent), 'exec'), namespace)
        fake = ModuleType('openpilot.selfdrive.kisapilot.bookmark_export')
        self.segment(number=1)
        fake.export_bookmarks = lambda root, ids, adjacent, **kwargs: b.export_bookmarks(
            self.root, ids, adjacent, preserved=lambda p: p.name == 'old--1', **kwargs)
        with patch.dict(sys.modules, {'openpilot.selfdrive.kisapilot.bookmark_export': fake}):
            result = namespace['prepare_bookmark_export']()
            self.assertEqual(len(result['files']), 1)
            self.assertTrue(result['files'][0]['path'].endswith('.zip'))
            payload['archive'] = False
            result = namespace['prepare_bookmark_export']()
            self.assertEqual(len(result['files']), 2)
            self.assertNotIn('archive', result)
            payload['archive'] = 'true'
            result, status = namespace['prepare_bookmark_export']()
            self.assertEqual(status, 409)
            self.assertIn('error', result)


if __name__ == '__main__':
    unittest.main()
