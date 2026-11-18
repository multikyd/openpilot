"""Private C4-side SFTP configuration and resumable OpenSSH transfers."""
import base64
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import secrets
import shutil
import subprocess
import threading
import time

KEY_DIR = Path('/data/params/kisa_sftp')
FIELDS = {'host': 'KisaBookmarkSftpHost', 'port': 'KisaBookmarkSftpPort',
          'user': 'KisaBookmarkSftpUser', 'remote_dir': 'KisaBookmarkSftpRemoteDir',
          'key_name': 'KisaBookmarkSftpKeyName', 'rate_kbps': 'KisaBookmarkSftpRateKbps'}


class TransferError(Exception):
  pass


def atomic_write(path, data):
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name+'.'+secrets.token_hex(6)+'.tmp')
  fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
  try:
    with os.fdopen(fd, 'wb') as output:
      output.write(data)
      output.flush()
      os.fsync(output.fileno())
    os.replace(temporary, path)
    try:
      directory = os.open(path.parent, os.O_RDONLY)
      try: os.fsync(directory)
      finally: os.close(directory)
    except OSError:
      pass
  finally:
    temporary.unlink(missing_ok=True)


def text(value):
  return value.decode('utf8') if isinstance(value, bytes) else str(value or '')


def read_config(params):
  config = {field: text(params.get(key)) for field, key in FIELDS.items()}
  for name, default in (('port', 22), ('rate_kbps', 4096)):
    try: config[name] = int(config[name] or default)
    except ValueError: config[name] = default
  config['enabled'] = params.get_bool('KisaBookmarkUploadEnabled')
  return config


def validate_config(config, complete=True):
  result = {name: config.get(name, '') for name in FIELDS}
  for name in ('host', 'user', 'remote_dir', 'key_name'):
    value = result[name]
    if not isinstance(value, str) or len(value) > 512 or any(ord(c) < 32 for c in value):
      raise TransferError('INVALID_'+name.upper())
    if complete and not value:
      raise TransferError('MISSING_'+name.upper())
  if result['host'] and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.:-]*', result['host']):
    raise TransferError('INVALID_HOST')
  if result['user'] and not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*', result['user']):
    raise TransferError('INVALID_USER')
  directory = result['remote_dir']
  if directory and (not directory.startswith('/') or '..' in PurePosixPath(directory).parts or
                    any(c in directory for c in '*?[]\\')):
    raise TransferError('INVALID_REMOTE_DIR')
  if result['key_name'] and (Path(result['key_name']).name != result['key_name'] or
                            not re.fullmatch(r'[A-Za-z0-9_. -]+', result['key_name']) or
                            result['key_name'].startswith('.')):
    raise TransferError('INVALID_KEY_NAME')
  for name, default, low, high in (('port', 22, 1, 65535), ('rate_kbps', 4096, 64, 65536)):
    if isinstance(result[name], (bool, float)):raise TransferError('INVALID_'+name.upper())
    try: result[name] = int(result[name] or default)
    except (TypeError, ValueError): raise TransferError('INVALID_'+name.upper()) from None
    if not low <= result[name] <= high: raise TransferError('INVALID_'+name.upper())
  if type(config.get('enabled',False)) is not bool:raise TransferError('INVALID_ENABLED')
  result['enabled'] = config.get('enabled',False)
  return result


def destination_id(config):
  return hashlib.sha256(json.dumps([config[k] for k in ('host', 'port', 'user', 'remote_dir')]).encode()).hexdigest()[:20]


def quote(value):
  if any(ord(c) < 32 for c in value): raise TransferError('INVALID_PATH')
  return '"'+value.replace('\\', '\\\\').replace('"', '\\"')+'"'


def error_code(stderr):
  value = stderr.lower()
  if 'host key' in value or 'host identification' in value: return 'HOST_KEY'
  if 'permission denied' in value or 'load key' in value or 'invalid format' in value: return 'AUTH_OR_PERMISSION'
  if 'timed out' in value or 'connection' in value or 'resolve hostname' in value: return 'NETWORK'
  return 'SFTP_FAILED'


class SftpClient:
  def __init__(self, config, key_dir=KEY_DIR, cancel=None, run=None):
    self.config = validate_config(config)
    self.key_dir = Path(key_dir)
    self.cancel = cancel or threading.Event()
    self.runner = run

  def command(self):
    c = self.config
    key, hosts = self.key_dir/c['key_name'], self.key_dir/'known_hosts'
    if not key.is_file() or key.is_symlink(): raise TransferError('KEY_MISSING')
    if not hosts.is_file() or hosts.is_symlink(): raise TransferError('HOST_KEY_REQUIRED')
    binary = shutil.which('sftp')
    if not binary: raise TransferError('SFTP_CLIENT_MISSING')
    host = '['+c['host']+']' if ':' in c['host'] else c['host']
    command = [binary, '-F', os.devnull, '-b', '-', '-f', '-P', str(c['port']), '-l', str(c['rate_kbps']),
               '-i', str(key), '-oBatchMode=yes', '-oIdentitiesOnly=yes', '-oPasswordAuthentication=no',
               '-oKbdInteractiveAuthentication=no', '-oStrictHostKeyChecking=yes',
               '-oUserKnownHostsFile='+str(hosts), '-oConnectTimeout=12',
               '-oServerAliveInterval=10', '-oServerAliveCountMax=3', c['user']+'@'+host]
    nice = shutil.which('nice') if os.name != 'nt' else None
    return ([nice, '-n', '15'] if nice else [])+command

  def batch(self, commands, timeout=1800):
    payload='\n'.join(commands+['bye'])+'\n'
    if self.runner:
      return self.runner(self.command(), payload, timeout)
    env=dict(os.environ, LC_ALL='C')
    process=subprocess.Popen(self.command(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True, encoding='utf8', errors='replace', env=env)
    start=time.monotonic()
    try:
      data=payload
      while True:
        if self.cancel.is_set(): raise TransferError('CANCELLED')
        if time.monotonic()-start > timeout: raise TransferError('TIMEOUT')
        try:
          out, err=process.communicate(data, timeout=1)
          return process.returncode, out[-65536:], err[-65536:]
        except subprocess.TimeoutExpired:
          data=None
    finally:
      if process.poll() is None:
        process.kill()
        process.communicate()

  def stat(self, path):
    code, out, err=self.batch(['@ls -ln '+quote(path)], 30)
    if code:
      if 'no such file' in err.lower() or 'not found' in err.lower(): return None
      raise TransferError(error_code(err))
    for line in out.splitlines():
      fields=line.split()
      if len(fields)>=5 and fields[0].startswith('-') and fields[4].isdigit(): return int(fields[4])
    raise TransferError('REMOTE_NOT_FILE')

  def upload(self, local, sha256):
    local=Path(local)
    if not re.fullmatch(r'[0-9a-f]{64}',sha256):raise TransferError('INVALID_ARCHIVE_ID')
    final=self.config['remote_dir'].rstrip('/')+'/bookmark-'+sha256+'.zip'
    size=local.stat().st_size
    complete=self.stat(final)
    if complete is not None:
      if complete != size: raise TransferError('REMOTE_CONFLICT')
      return 'already_done'
    partial=self.stat(final+'.part')
    if partial is not None and partial>size: raise TransferError('REMOTE_CONFLICT')
    # reput resumes this immutable, content-addressed ZIP. rename executes only
    # after reput succeeds; a power loss never exposes a partial final ZIP.
    code, _, err=self.batch(['-@mkdir '+quote(self.config['remote_dir']),
                            '@reput '+quote(local.as_posix())+' '+quote(final+'.part'),
                            '@rename '+quote(final+'.part')+' '+quote(final)])
    if code: raise TransferError(error_code(err))
    if self.stat(final) != size: raise TransferError('REMOTE_SIZE')
    return 'saved'

  def test_connection(self):
    # Actually test write permission without writing log/video data.
    probe=self.config['remote_dir'].rstrip('/')+'/.kisa-test-'+secrets.token_hex(8)
    import tempfile
    fd, name=tempfile.mkstemp()
    try:
      with os.fdopen(fd,'wb') as f: f.write(b'kisa-sftp-test\n')
      code, _, err=self.batch(['-@mkdir '+quote(self.config['remote_dir']),
                              '@cd '+quote(self.config['remote_dir']),
                              '@put '+quote(Path(name).as_posix())+' '+quote(probe), '@rm '+quote(probe)], 30)
      if code: raise TransferError(error_code(err))
    finally:
      Path(name).unlink(missing_ok=True)
