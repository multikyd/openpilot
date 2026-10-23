"""Read loggerd bookmark xattrs and export closed, original replay segments."""
import os
import re
import time
from functools import lru_cache
from pathlib import Path


def split_segment(name):
    match = re.fullmatch(r"([^/\\]+)--(\d+)", name)
    if not match or match[1] in (".", ".."):
        return None
    return match[1], int(match[2])


def is_preserved(path):
    try:
        return os.getxattr(path, "user.preserve") == b"1"
    except (OSError, AttributeError):
        return False


def recording_routes(root, onroad, current_route='', proc_root='/proc'):
    """Return routes held by recorder processes; None means detection is uncertain.

    CurrentRoute also protects the gap before loggerd opens its first file. Never
    infer a finished route from file age alone or from another route's presence.
    """
    routes = set()
    root = Path(root).resolve()
    try:
        processes = list(Path(proc_root).iterdir())
        for process in processes:
            if not process.name.isdigit():
                continue
            try:
                name = (process / 'comm').read_text().strip()
                if name not in ('loggerd', 'encoderd', 'stream_encoderd'):
                    continue
                for fd in (process / 'fd').iterdir():
                    try:
                        target = Path(os.readlink(fd))
                    except FileNotFoundError:
                        continue
                    try:
                        relative = target.relative_to(root)
                    except ValueError:
                        continue
                    parsed = split_segment(relative.parts[0]) if relative.parts else None
                    if parsed:
                        routes.add(parsed[0])
            except FileNotFoundError:
                if process.exists():
                    return None
            except OSError:
                return None
    except OSError:
        return None
    if onroad:
        if current_route and split_segment(current_route + '--0'):
            routes.add(current_route)
        elif not routes:
            return None
    return routes


def snapshot(root, preserved=is_preserved, recording=None, now=None):
    root = Path(root)
    if not root.is_dir():
        raise FileNotFoundError("주행 로그 폴더가 없습니다.")
    result = {}
    now = time.time() if now is None else now
    for directory in root.iterdir():
        parsed = split_segment(directory.name)
        if not parsed or directory.is_symlink() or not directory.is_dir():
            continue
        try:
            files = []
            latest_change = directory.stat().st_mtime
            for path in directory.iterdir():
                if path.is_symlink() or not path.is_file():
                    continue
                try:
                    stat = path.stat()
                    latest_change = max(latest_change, stat.st_mtime, stat.st_ctime)
                    files.append({"path": str(path), "name": path.name, "size": stat.st_size, "is_dir": False})
                except FileNotFoundError:
                    continue
            locked = any(f["name"].endswith(".lock") for f in files)
            interrupted = (locked and recording is not None and parsed[0] not in recording
                           and now - latest_change >= 30)
            result[directory.name] = {"id": directory.name, "route": parsed[0], "segment": parsed[1],
                                      "active": locked and not interrupted, "interrupted": interrupted,
                                      "bookmarked": preserved(directory), "files": files}
        except FileNotFoundError:
            continue
    return result


def list_bookmarks(root, preserved=is_preserved, recording=None):
    segments = snapshot(root, preserved, recording)
    bookmarks = []
    for item in sorted(segments.values(), key=lambda s: (s["route"], s["segment"]), reverse=True):
        if item["bookmarked"]:
            bookmarks.append({k: item[k] for k in ("id", "route", "segment", "active")})
    return {"bookmarks": bookmarks, "precision": "segment", "segment_seconds": 60,
            "note": "북마크가 있는 1분 구간입니다. 정확한 이벤트 시각은 원본 로그의 userBookmark/bookmarkButton에 있습니다."}


def bookmark_offset(events):
    """Earliest bookmark relative to this segment's sentinel, not route initData."""
    start = None
    bookmarks = []
    for event in events:
        kind = event.which()
        if kind == 'sentinel' and str(event.sentinel.type) in ('startOfSegment', 'startOfRoute'):
            start = event.logMonoTime if start is None else min(start, event.logMonoTime)
        elif kind in ('userBookmark', 'bookmarkButton'):
            bookmarks.append(event.logMonoTime)
    if start is None or not bookmarks:
        return None
    offset = (min(bookmarks) - start) / 1e9
    return offset if 0 <= offset <= 60 else None


@lru_cache(maxsize=256)
def _log_bookmark_offset(path, size, modified_ns):
    # Cache only completed files, invalidating if a recovered file changes.
    from openpilot.tools.lib.logreader import LogReader
    return bookmark_offset(LogReader(path))


def segment_bookmark_offset(root, item):
    if item['active']:
        return None
    # qlog contains both bookmark services and the segment sentinel, and is small.
    for name in ('qlog.zst', 'qlog.bz2', 'qlog', 'rlog.zst', 'rlog.bz2', 'rlog'):
        path = Path(root, item['id'], name)
        try:
            if path.is_symlink() or not path.is_file():
                continue
            stat = path.stat()
            offset = _log_bookmark_offset(str(path), stat.st_size, stat.st_mtime_ns)
            if offset is not None:
                return offset
        except Exception:
            # Older/corrupt logs or unavailable decoders must retain history.
            continue
    return None


def bookmark_segments(root, item, adjacent=1):
    offset = segment_bookmark_offset(root, item)
    first = item['segment']
    # Legacy clients still send adjacent=1 (or 2); neither requests future data.
    if adjacent and (offset is None or offset < 30):
        first = max(0, first - 1)
    return [item['route'] + '--' + str(n) for n in range(first, item['segment'] + 1)], offset


def auto_bookmarks(root, onroad=False, preserved=is_preserved, recording=None):
    """Wait for the bookmark and required history to close, never its successor."""
    segments = snapshot(root, preserved, recording)
    bookmarks = []
    for item in sorted(segments.values(), key=lambda s: (s['route'], s['segment'])):
        if not item['bookmarked']:
            continue
        selected, _ = bookmark_segments(root, item)
        waiting = any(segments[n]['active'] for n in selected if n in segments)
        bookmarks.append({k: item[k] for k in ('id', 'route', 'segment', 'active')} | {'ready': not waiting})
    return {'bookmarks': bookmarks, 'onroad': bool(onroad), 'format': 'kisa-bookmark-auto-v1'}


def export_bookmarks(root, ids, adjacent=1, preserved=is_preserved, recording=None):
    if not isinstance(ids, list) or not ids or len(ids) > 100 or any(not isinstance(i, str) or not split_segment(i) for i in ids):
        raise ValueError("북마크를 1~100개 선택해주세요.")
    if type(adjacent) is not int or adjacent not in (0, 1, 2):
        raise ValueError("북마크 범위 값은 0~2여야 합니다.")
    segments = snapshot(root, preserved, recording)
    for name in ids:
        if name not in segments or not segments[name]["bookmarked"]:
            raise ValueError("북마크가 삭제되거나 변경되었습니다. 목록을 새로고침해주세요: " + name)
        if segments[name]["active"]:
            raise ValueError("북마크 구간이 아직 기록 중입니다. 약 1분 후 새로고침해주세요: " + name)
    included = set()
    excluded = []
    offsets = {}
    for name in dict.fromkeys(ids):
        item = segments[name]
        selected, offsets[name] = bookmark_segments(root, item, adjacent)
        for neighbor in selected:
            if neighbor not in segments:
                excluded.append(neighbor + " · 세그먼트 없음 (아직 생성 전 또는 삭제됨)")
            elif segments[neighbor]["active"]:
                raise ValueError("이전 북마크 분석 구간이 아직 기록 중입니다: " + neighbor)
            else:
                included.add(neighbor)
    files = []
    warnings = list(dict.fromkeys(excluded))
    for name in sorted(included, key=lambda n: (segments[n]["route"], segments[n]["segment"])):
        item = segments[name]
        if item["interrupted"]:
            warnings.append(name + " · 전원/기록 중단: 남아 있는 파일을 저장합니다. 마지막 로그·영상은 미완성일 수 있습니다.")
        names = {f["name"] for f in item["files"]}
        for required, candidates in (("주행 원본 로그", {"rlog", "rlog.zst", "rlog.bz2"}), ("전방 영상", {"fcamera.hevc"})):
            if not names.intersection(candidates):
                warnings.append(name + " · " + required + " 없음")
        files.extend({k: f[k] for k in ("path", "size", "is_dir")} for f in item["files"] if not f["name"].endswith(".lock"))
    if not files:
        raise ValueError("다운로드할 완료된 파일이 없습니다.")
    metadata = {"format": "kisa-bookmark-export-v1", "precision": "segment", "segment_seconds": 60,
                "adjacent_segments": adjacent, "selection_policy": "bookmark-history-30s",
                "bookmark_offsets_seconds": offsets,
                "bookmarks": [{k: segments[n][k] for k in ("id", "route", "segment")} for n in dict.fromkeys(ids)],
                "included_segments": sorted(included), "warnings": warnings,
                "interrupted_segments": sorted(n for n in included if segments[n]["interrupted"]),
                "replay_note": "원본 세그먼트 폴더와 로그·영상을 유지합니다. 정확한 북마크 시각은 로그의 userBookmark/bookmarkButton을 확인하세요. 영상/로그를 자르거나 세그먼트 번호를 바꾸지 마세요."}
    return {"files": files, "bytes": sum(f["size"] for f in files), "excluded": warnings, "bookmark_metadata": metadata}
