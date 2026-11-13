"""Offline optional ONNX runtime preparation. Safe to run before native builds."""
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import shutil
import stat
import sys
import tempfile
import uuid
import zipfile

BUNDLE = Path(__file__).resolve().parents[1] / 'selfdrive/ui/onroad/models/runtime'


def runtime_base():
  return Path(os.environ.get('LEAD_BRAKE_RUNTIME_DIR', '/data/extra/lead-brake-runtime'))


def bundle_info(bundle=BUNDLE):
  raw = (bundle/'manifest.json').read_bytes()
  return json.loads(raw), hashlib.sha256(raw).hexdigest()


def supported(manifest):
  return (platform.system() == manifest['system'] and
          platform.machine().lower() in ('aarch64', 'arm64') and
          f'{sys.version_info.major}.{sys.version_info.minor}' == manifest['python'])


def _inside(path, base):
  return path.resolve().is_relative_to(base.resolve())


def runtime_directory(bundle=BUNDLE, base=None, check_platform=True):
  base = runtime_base() if base is None else Path(base)
  try:
    manifest, fingerprint = bundle_info(bundle)
    if check_platform and not supported(manifest):
      return None
    pointer = json.loads((base/'active.json').read_text())
    name = pointer['directory']
    if Path(name).name != name or '/' in name or '\\' in name:
      return None
    target = base/name
    if not _inside(target, base) or (target/'.broken').exists():
      return None
    marker = json.loads((target/'installed.json').read_text())
    if marker['bundle'] != fingerprint or len(marker['checks']) < 3:
      return None
    for item in marker['checks']:
      path = target/item['file']
      if not _inside(path, target) or not path.is_file() or path.stat().st_size != item['size']:
        return None
    return target
  except (OSError, ValueError, KeyError, TypeError):
    return None


def _extract_wheel(wheel, target):
  with zipfile.ZipFile(wheel) as archive:
    for item in archive.infolist():
      name = PurePosixPath(item.filename)
      if (name.is_absolute() or '..' in name.parts or '\\' in item.filename or
          ':' in item.filename or stat.S_ISLNK(item.external_attr >> 16)):
        raise ValueError('Unsafe runtime wheel member')
      dest = target/str(name)
      if not _inside(dest, target):
        raise ValueError('Runtime member escapes target')
      if item.is_dir():
        dest.mkdir(parents=True, exist_ok=True)
      else:
        dest.parent.mkdir(parents=True, exist_ok=True)
        with archive.open(item) as source, dest.open('wb') as output:
          shutil.copyfileobj(source, output)


def prepare_runtime(bundle=BUNDLE, base=None, check_platform=True):
  bundle = Path(bundle)
  base = runtime_base() if base is None else Path(base)
  manifest, fingerprint = bundle_info(bundle)
  if check_platform and not supported(manifest):
    return 'unsupported'
  if runtime_directory(bundle, base, check_platform=check_platform) is not None:
    return 'ready'
  # Check all downloaded packages before creating an installation.
  for item in manifest['wheels']:
    name = item['file']
    if Path(name).name != name or not name.endswith('.whl'):
      raise ValueError('Invalid runtime wheel name')
    wheel = bundle/name
    if wheel.stat().st_size != item['size'] or hashlib.sha256(wheel.read_bytes()).hexdigest() != item['sha256']:
      raise ValueError('Runtime wheel checksum mismatch')
  base.mkdir(parents=True, exist_ok=True)
  staging = Path(tempfile.mkdtemp(prefix='.staging-', dir=base))
  try:
    for item in manifest['wheels']:
      _extract_wheel(bundle/item['file'], staging)
    critical = [staging/'onnxruntime/__init__.py', *sorted((staging/'onnxruntime/capi').glob('*.so*'))]
    if len(critical) < 3 or not all(p.is_file() for p in critical):
      raise ValueError('Incomplete ONNX runtime package')
    marker = dict(bundle=fingerprint, version=manifest['version'],
                  checks=[dict(file=str(p.relative_to(staging)), size=p.stat().st_size) for p in critical])
    (staging/'installed.json').write_text(json.dumps(marker), encoding='utf-8')
    final = base/f'ort-{fingerprint[:12]}-{uuid.uuid4().hex[:8]}'
    if not _inside(staging, base) or not _inside(final, base):
      raise ValueError('Runtime activation escapes target')
    staging.rename(final)
    pointer = base/f'.active-{uuid.uuid4().hex}.json'
    pointer.write_text(json.dumps(dict(directory=final.name)), encoding='utf-8')
    os.replace(pointer, base/'active.json')
    return 'installed'
  finally:
    # Delete only this invocation's checked, temporary staging directory.
    if staging.exists() and _inside(staging, base) and staging.name.startswith('.staging-'):
      shutil.rmtree(staging)


def use_runtime():
  """Only call in the optional worker, never inject packages into the UI."""
  directory = runtime_directory()
  if directory is not None:
    sys.path.insert(0, str(directory))
  return directory


def mark_broken(directory):
  if directory is not None and _inside(Path(directory), runtime_base()):
    try:
      (Path(directory)/'.broken').touch()
    except OSError:
      pass


def main():
  if os.environ.get('LEAD_BRAKE_MODEL', '1') == '0':
    return
  try:
    status = prepare_runtime()
    if status == 'installed':
      print('Brake vehicle runtime prepared from bundled files (offline).')
  except Exception as exc:
    # Optional perception must never prevent manager/UI startup.
    print(f'Brake vehicle runtime unavailable ({type(exc).__name__}); using existing detector.')


if __name__ == '__main__':
  main()
