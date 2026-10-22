import importlib.util
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

SOURCE = Path(__file__).parents[1] / 'bookmark_export.py'
spec = importlib.util.spec_from_file_location('bookmark_export', SOURCE)
b = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b)


class BookmarkRecoveryTest(unittest.TestCase):
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

    def test_reboot_with_old_lock_and_missing_next_exports_existing_files(self):
        p = self.segment(locked=True)
        self.assertTrue(self.auto(set(), False)['bookmarks'][0]['ready'])
        result = self.export(set())
        self.assertEqual(len(result['files']), 2)
        self.assertEqual(result['bookmark_metadata']['interrupted_segments'], ['old--0'])
        self.assertTrue(any('세그먼트 없음' in w for w in result['excluded']))
        self.assertTrue(any('미완성' in w for w in result['excluded']))
        self.assertTrue((p / 'rlog.lock').exists())
        self.assertEqual((p / 'rlog.zst').read_bytes(), b'partial log')

    def test_next_drive_does_not_wait_for_old_unfinished_next_segment(self):
        self.segment(locked=True)
        self.segment(number=1, locked=True)
        self.segment('new', locked=True)
        self.assertTrue(self.auto({'new'})['bookmarks'][0]['ready'])
        self.assertEqual(len(self.export({'new'})['files']), 4)

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

    def test_closed_current_bookmark_waits_for_future_segment(self):
        self.segment()
        self.assertFalse(self.auto({'old'})['bookmarks'][0]['ready'])

    def test_ended_closed_route_can_finish_without_next_segment(self):
        self.segment()
        self.assertTrue(self.auto({'new'})['bookmarks'][0]['ready'])

    def test_active_next_segment_keeps_bookmark_waiting(self):
        self.segment()
        self.segment(number=1, locked=True)
        self.assertFalse(self.auto({'old'})['bookmarks'][0]['ready'])

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


if __name__ == '__main__':
    unittest.main()
