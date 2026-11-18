"""Managed private keys; encrypted app upload and explicit SSH host-key trust."""
import base64
import hashlib
import os
from pathlib import Path
import re
import secrets
import shutil
import subprocess
import threading
import time

from openpilot.selfdrive.kisapilot.bookmark_sftp import KEY_DIR, TransferError, atomic_write, validate_config


class UploadKeys:
  def __init__(self, directory=KEY_DIR):
    self.directory=Path(directory)
    self.lock=threading.Lock()
    self.nonces={}
    self.trust={}

  def setup(self):
    if self.directory.is_symlink():raise TransferError('INVALID_KEY_DIRECTORY')
    self.directory.mkdir(parents=True,exist_ok=True)
    os.chmod(self.directory,0o700)

  def inventory(self):
    self.setup()
    keys=[]
    for path in self.directory.iterdir():
      if path.name.startswith(('transport.','known_hosts')) or path.suffix in ('.tmp','.pub') or path.is_symlink() or not path.is_file():continue
      with path.open('rb') as file:header=file.read(100)
      if b'PRIVATE KEY-----' in header:keys.append({'name':path.name})
    return sorted(keys,key=lambda item:item['name'])

  def public_key(self):
    self.setup()
    binary=shutil.which('openssl')
    if not binary:raise TransferError('KEY_CRYPTO_UNAVAILABLE')
    with self.lock:
      key=self.directory/'transport.pem'
      if not key.exists():
        result=subprocess.run([binary,'genpkey','-algorithm','RSA','-pkeyopt','rsa_keygen_bits:2048'],
                              stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,timeout=30)
        if result.returncode:raise TransferError('KEY_CRYPTO_UNAVAILABLE')
        atomic_write(key,result.stdout)
      result=subprocess.run([binary,'pkey','-in',str(key),'-pubout','-outform','DER'],
                            stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,timeout=10)
      if result.returncode:raise TransferError('KEY_CRYPTO_UNAVAILABLE')
      now=time.monotonic()
      self.nonces={k:v for k,v in self.nonces.items() if v>now}
      if len(self.nonces)>16:raise TransferError('BUSY')
      nonce=secrets.token_urlsafe(24)
      self.nonces[nonce]=now+120
      return {'public_key':base64.b64encode(result.stdout).decode(), 'nonce':nonce,
              'fingerprint':'SHA256:'+base64.b64encode(hashlib.sha256(result.stdout).digest()).decode().rstrip('=')}

  def receive(self, body):
    if not isinstance(body,dict):raise TransferError('INVALID_KEY_UPLOAD')
    with self.lock:
      expiry=self.nonces.pop(body.get('nonce',''),0)
    if expiry<time.monotonic():raise TransferError('KEY_UPLOAD_EXPIRED')
    name=body.get('name','key.pem')
    if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z0-9_. -]{1,100}',name) or name.startswith('.'):
      raise TransferError('INVALID_KEY_NAME')
    chunks=body.get('chunks')
    if not isinstance(chunks,list) or not 1<=len(chunks)<=90:raise TransferError('INVALID_KEY_UPLOAD')
    binary=shutil.which('openssl')
    if not binary:raise TransferError('KEY_CRYPTO_UNAVAILABLE')
    plain=bytearray()
    try:
      for chunk in chunks:
        try:cipher=base64.b64decode(chunk,validate=True)
        except (ValueError,TypeError):raise TransferError('INVALID_KEY_UPLOAD') from None
        if len(cipher)!=256:raise TransferError('INVALID_KEY_UPLOAD')
        result=subprocess.run([binary,'pkeyutl','-decrypt','-inkey',str(self.directory/'transport.pem'),
              '-pkeyopt','rsa_padding_mode:oaep','-pkeyopt','rsa_oaep_md:sha256','-pkeyopt','rsa_mgf1_md:sha256'],
              input=cipher,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,timeout=10)
        if result.returncode:raise TransferError('INVALID_KEY_UPLOAD')
        plain.extend(result.stdout)
      if len(plain)>16384 or b'PRIVATE KEY-----' not in plain[:100] or b'ENCRYPTED' in plain[:200]:
        raise TransferError('UNSUPPORTED_KEY')
      keyname=secrets.token_hex(4)+'-'+name
      path=self.directory/keyname
      atomic_write(path,plain)
      binary=shutil.which('ssh-keygen')
      if not binary:
        path.unlink(missing_ok=True)
        raise TransferError('SSH_CLIENT_MISSING')
      result=subprocess.run([binary,'-y','-P','','-f',str(path)],stdin=subprocess.DEVNULL,
                            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=10)
      if result.returncode:
        path.unlink(missing_ok=True)
        raise TransferError('UNSUPPORTED_OR_ENCRYPTED_KEY')
      return {'name':keyname}
    finally:
      plain[:]=b'\0'*len(plain)

  def scan_host(self, config):
    config=validate_config(config,complete=False)
    if not config['host']:raise TransferError('MISSING_HOST')
    binary=shutil.which('ssh-keyscan')
    if not binary:raise TransferError('SSH_CLIENT_MISSING')
    result=subprocess.run([binary,'-T','5','-p',str(config['port']),config['host']],
                          stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,timeout=20)
    candidates=[]
    for line in result.stdout.decode('ascii',errors='ignore').splitlines():
      fields=line.split()
      if len(fields)!=3 or fields[1] not in ('ssh-ed25519','ecdsa-sha2-nistp256','ssh-rsa'):continue
      try:raw=base64.b64decode(fields[2],validate=True)
      except ValueError:continue
      candidates.append((fields[1],line,'SHA256:'+base64.b64encode(hashlib.sha256(raw).digest()).decode().rstrip('=')))
    if not candidates:raise TransferError('HOST_UNREACHABLE')
    candidates.sort(key=lambda item:('ssh-ed25519','ecdsa-sha2-nistp256','ssh-rsa').index(item[0]))
    algorithm,line,fingerprint=candidates[0]
    token=secrets.token_urlsafe(24)
    with self.lock:
      self.trust[token]=(time.monotonic()+120,config['host'],config['port'],line)
    return {'challenge':token,'algorithm':algorithm,'fingerprint':fingerprint}

  def trust_host(self, challenge, config):
    self.setup()
    with self.lock:item=self.trust.pop(challenge,None)
    if not item or item[0]<time.monotonic() or (item[1],item[2])!=(config['host'],config['port']):
      raise TransferError('HOST_KEY_EXPIRED')
    atomic_write(self.directory/'known_hosts',(item[3]+'\n').encode('ascii'))
