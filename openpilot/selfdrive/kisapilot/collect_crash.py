"""Run after parking: collect recent crash evidence locally, without uploading."""
import argparse
import io
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import tarfile
import time

MAX_FILE = 256 * 1024 * 1024
MAX_TOTAL = 512 * 1024 * 1024
ROOTS = (Path('/data/carrot_crash'), Path('/var/crash'), Path('/data/tombstones'), Path('/data/crashes'))


def select_files(roots, now, max_file=None, max_total=None):
  max_file = MAX_FILE if max_file is None else max_file
  max_total = MAX_TOTAL if max_total is None else max_total
  candidates, skipped = [], []
  for root in roots:
    try:
      for path in root.iterdir():
        if path.is_symlink() or not path.is_file():
          continue
        stat = path.stat()
        if not 0 <= now-stat.st_mtime <= 7*86400:
          continue
        if root.name == 'carrot_crash' or path.suffix in ('.crash', '.log', '.txt') or path.name.startswith('tombstone'):
          candidates.append((stat.st_mtime, stat.st_size, path))
    except OSError as error:
      skipped.append(dict(path=str(root), reason=str(error)))
  chosen, total = [], 0
  for _, size, path in sorted(candidates, key=lambda item: item[0], reverse=True):
    if len(chosen) >= 20 or size > max_file or total+size > max_total:
      skipped.append(dict(path=str(path), reason='collection size/count limit', size=size))
      continue
    chosen.append(path)
    total += size
  return chosen, skipped, total


def command(args):
  try:
    result = subprocess.run(args, capture_output=True, text=True, errors='replace', timeout=10)
    return dict(command=args, returncode=result.returncode, stdout=result.stdout[-2_000_000:], stderr=result.stderr[-10_000:])
  except (OSError, subprocess.TimeoutExpired) as error:
    return dict(command=args, error=str(error))


def nearby_segments(root, route):
  if not isinstance(route, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', route):
    return []
  candidates = []
  for path in Path(root).glob(route + '--*'):
    suffix = path.name[len(route)+2:]
    if suffix.isdigit() and path.is_dir() and not path.is_symlink():
      candidates.append((int(suffix), path.name))
  return [name for _, name in sorted(candidates)[-2:]]


def collect(destination, roots=ROOTS, event=None, automatic=False):
  destination = Path(destination)
  destination.mkdir(parents=True, exist_ok=True)
  max_file, max_total = (32*1024*1024, 64*1024*1024) if automatic else (MAX_FILE, MAX_TOTAL)
  chosen, skipped, total = select_files(roots, time.time(), max_file, max_total)
  if event:
    for segment in event.get('segments', [])[:2]:
      if not isinstance(segment, str) or not re.fullmatch(r'[A-Za-z0-9_-]+--[0-9]+', segment):
        continue
      directory = destination / segment
      if directory.is_symlink():
        continue
      for name in ('qlog.zst', 'rlog.zst', 'qlog.bz2', 'rlog.bz2'):
        path = directory / name
        try:
          if not path.is_file() or path.is_symlink():
            continue
          size = path.stat().st_size
          if size > max_file or total+size > max_total or len(chosen) >= 20:
            skipped.append(dict(path=str(path), reason='automatic collection limit'))
          elif path not in chosen:
            chosen.append(path)
            total += size
        except OSError as error:
          skipped.append(dict(path=str(path), reason=str(error)))
  if shutil.disk_usage(destination).free < total+128*1024*1024:
    raise OSError('Not enough space for crash collection; original files were preserved')
  report = dict(event=event, automatic=automatic, wall=time.time(), files=[str(p) for p in chosen], skipped=skipped,
                journal=command(['journalctl', '-u', 'comma.service', '--no-pager', '-n', '2000', '-o', 'short-iso']),
                cores=command(['coredumpctl', 'list', '--no-pager']))
  for name in ('core_pattern', 'core_uses_pid'):
    try:
      report[name] = Path('/proc/sys/kernel', name).read_text().strip()
    except OSError as error:
      report[name] = str(error)
  try:
    import resource
    report['collector_core_limit'] = resource.getrlimit(resource.RLIMIT_CORE)
  except (ImportError, OSError, ValueError):
    pass
  if automatic and event:
    filename = destination / f"process-crash-report-{int(event['wall']*1e6)}-{event['pid']}.tar.gz"
  else:
    filename = destination / f'carrot-crash-report-{time.strftime("%Y%m%d-%H%M%S")}-{os.getpid()}.tar.gz'
  partial = filename.with_suffix(filename.suffix + '.partial')
  with tarfile.open(partial, 'w:gz', compresslevel=1 if automatic else 9) as archive:
    for path in chosen:
      try:
        archive.add(path, arcname='evidence/'+str(path).lstrip('/'), recursive=False)
      except OSError as error:
        report['skipped'].append(dict(path=str(path), reason=str(error)))
    data = json.dumps(report, ensure_ascii=False, indent=2).encode()
    entry = tarfile.TarInfo('report.json')
    entry.size = len(data)
    archive.addfile(entry, io.BytesIO(data))
  with partial.open('r+b') as completed:
    os.fsync(completed.fileno())
  os.replace(partial, filename)
  try:
    directory_fd = os.open(destination, os.O_RDONLY)
    try:
      os.fsync(directory_fd)
    finally:
      os.close(directory_fd)
  except OSError:
    pass
  return filename


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', default='/data/media/0/realdata')
  args = parser.parse_args()
  print(collect(args.output))


if __name__ == '__main__':
  main()
