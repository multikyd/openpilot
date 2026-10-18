"""Read loggerd bookmark xattrs and export closed, original replay segments."""
import os
import re
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


def snapshot(root, preserved=is_preserved):
    root = Path(root)
    if not root.is_dir():
        raise FileNotFoundError("주행 로그 폴더가 없습니다.")
    result = {}
    for directory in root.iterdir():
        parsed = split_segment(directory.name)
        if not parsed or directory.is_symlink() or not directory.is_dir():
            continue
        try:
            files = []
            for path in directory.iterdir():
                if path.is_symlink() or not path.is_file():
                    continue
                try:
                    files.append({"path": str(path), "name": path.name, "size": path.stat().st_size, "is_dir": False})
                except FileNotFoundError:
                    continue
            result[directory.name] = {"id": directory.name, "route": parsed[0], "segment": parsed[1],
                                      "active": any(f["name"].endswith(".lock") for f in files),
                                      "bookmarked": preserved(directory), "files": files}
        except FileNotFoundError:
            continue
    return result


def list_bookmarks(root, preserved=is_preserved):
    segments = snapshot(root, preserved)
    bookmarks = []
    for item in sorted(segments.values(), key=lambda s: (s["route"], s["segment"]), reverse=True):
        if item["bookmarked"]:
            bookmarks.append({k: item[k] for k in ("id", "route", "segment", "active")})
    return {"bookmarks": bookmarks, "precision": "segment", "segment_seconds": 60,
            "note": "북마크가 있는 1분 구간입니다. 정확한 이벤트 시각은 원본 로그의 userBookmark/bookmarkButton에 있습니다."}


def export_bookmarks(root, ids, adjacent=1, preserved=is_preserved):
    if not isinstance(ids, list) or not ids or len(ids) > 100 or any(not isinstance(i, str) or not split_segment(i) for i in ids):
        raise ValueError("북마크를 1~100개 선택해주세요.")
    if type(adjacent) is not int or adjacent not in (0, 1, 2):
        raise ValueError("앞뒤 범위는 0~2개 세그먼트여야 합니다.")
    segments = snapshot(root, preserved)
    for name in ids:
        if name not in segments or not segments[name]["bookmarked"]:
            raise ValueError("북마크가 삭제되거나 변경되었습니다. 목록을 새로고침해주세요: " + name)
        if segments[name]["active"]:
            raise ValueError("북마크 구간이 아직 기록 중입니다. 약 1분 후 새로고침해주세요: " + name)
    included = set()
    excluded = []
    for name in dict.fromkeys(ids):
        item = segments[name]
        for number in range(max(0, item["segment"] - adjacent), item["segment"] + adjacent + 1):
            neighbor = item["route"] + "--" + str(number)
            if neighbor not in segments:
                excluded.append(neighbor + " · 세그먼트 없음 (아직 생성 전 또는 삭제됨)")
            elif segments[neighbor]["active"]:
                excluded.append(neighbor + " · 기록 중인 세그먼트")
            else:
                included.add(neighbor)
    files = []
    warnings = list(dict.fromkeys(excluded))
    for name in sorted(included, key=lambda n: (segments[n]["route"], segments[n]["segment"])):
        item = segments[name]
        names = {f["name"] for f in item["files"]}
        for required, candidates in (("주행 원본 로그", {"rlog", "rlog.zst", "rlog.bz2"}), ("전방 영상", {"fcamera.hevc"})):
            if not names.intersection(candidates):
                warnings.append(name + " · " + required + " 없음")
        files.extend({k: f[k] for k in ("path", "size", "is_dir")} for f in item["files"] if not f["name"].endswith(".lock"))
    if not files:
        raise ValueError("다운로드할 완료된 파일이 없습니다.")
    metadata = {"format": "kisa-bookmark-export-v1", "precision": "segment", "segment_seconds": 60,
                "adjacent_segments": adjacent, "bookmarks": [{k: segments[n][k] for k in ("id", "route", "segment")} for n in dict.fromkeys(ids)],
                "included_segments": sorted(included), "warnings": warnings,
                "replay_note": "원본 세그먼트 폴더와 로그·영상을 유지합니다. 정확한 북마크 시각은 로그의 userBookmark/bookmarkButton을 확인하세요. 영상/로그를 자르거나 세그먼트 번호를 바꾸지 마세요."}
    return {"files": files, "bytes": sum(f["size"] for f in files), "excluded": warnings, "bookmark_metadata": metadata}
