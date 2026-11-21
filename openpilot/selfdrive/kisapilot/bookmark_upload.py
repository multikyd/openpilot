"""Persistent, single-transfer SFTP queue; discovery continues during uploads."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import threading
import time
import zipfile

from openpilot.selfdrive.kisapilot.bookmark_export import (
  bookmark_archive_name, export_bookmarks, is_preserved, segment_bookmark_timing, split_segment,
)
from openpilot.selfdrive.kisapilot.bookmark_sftp import (
  KEY_DIR, SftpClient, TransferError, atomic_write, destination_id, read_config, validate_config,
)

ALLOWED = {'qcamera.ts', 'qcamera.mp4', 'qlog', 'qlog.zst', 'qlog.bz2', 'rlog', 'rlog.zst', 'rlog.bz2'}


def pending_segments(root):
  """Small cross-process view used by loggerd's deleter, not its own queue API."""
  path=Path(root)/'bookmark_uploads/queue.json'
  try:
    if path.is_symlink() or path.stat().st_size > 8*1024*1024: return set()
    state=json.loads(path.read_text(encoding='utf8'))
    return {name for job in state.get('jobs',{}).values() if job.get('state') != 'done'
            for name in job.get('segments',[]) if isinstance(name,str) and split_segment(name)}
  except (OSError, ValueError, TypeError, AttributeError):
    return set()


class BookmarkUploader:
  def __init__(self, root, params, publish=lambda value: None, *, key_dir=KEY_DIR,
               preserved=is_preserved, client_factory=SftpClient, clock=time.time):
    self.root=Path(root)
    self.spool=self.root/'bookmark_uploads'
    self.params,self.publish,self.key_dir=params,publish,Path(key_dir)
    self.preserved,self.client_factory,self.clock=preserved,client_factory,clock
    self.lock=threading.RLock()
    self.stop_event=threading.Event()
    self.cancel=threading.Event()
    self.state={'version':1,'jobs':{}}
    self.observed={}
    self.error=''
    self.last_done=0.
    self.threads=[]
    self.loaded=False

  def load(self):
    if self.spool.is_symlink(): raise TransferError('INVALID_SPOOL')
    self.spool.mkdir(parents=True,exist_ok=True)
    path=self.spool/'queue.json'
    if path.exists():
      # Never silently reset a corrupt queue and re-upload everything.
      if path.is_symlink() or path.stat().st_size>8*1024*1024: raise TransferError('QUEUE_INVALID')
      try: state=json.loads(path.read_text(encoding='utf8'))
      except (OSError, ValueError): raise TransferError('QUEUE_INVALID') from None
      if state.get('version')!=1 or not isinstance(state.get('jobs'),dict): raise TransferError('QUEUE_INVALID')
      self.state=state
      for job in self.state['jobs'].values():
        if job['state']=='uploading': job.update(state='ready',retry_at=0)
    self.loaded=True

  def save(self):
    atomic_write(self.spool/'queue.json',json.dumps(self.state,ensure_ascii=False,sort_keys=True).encode('utf8'))

  def config(self):
    return validate_config(read_config(self.params))

  def summary(self):
    with self.lock:
      try:
        config=self.config()
        enabled=config['enabled']
        dest=destination_id(config)
      except TransferError as error:
        enabled=self.params.get_bool('KisaBookmarkUploadEnabled')
        dest=None
        if enabled:self.error=str(error)
      jobs=[j for j in self.state['jobs'].values() if j['destination']==dest]
      pending=[j for j in jobs if j['state']!='done']
      sending=any(j['state']=='uploading' for j in pending)
      retry=next((j.get('error') for j in pending if j.get('error')),None)
      error=self.error or retry or ''
      phase=('stopped' if not enabled else 'receiving' if sending else 'error' if error else
             'waiting' if pending else 'saved' if self.clock()-self.last_done<10 else 'stopped')
      return {'state':phase,'target':'server','pending':len(pending),
              'done':sum(j['state']=='done' for j in jobs),'error':error,
              'uploading':sending,'recording_wait':sum(j['state']=='waiting' for j in pending),
              'other_destination_pending':sum(j['state']!='done' and j['destination']!=dest for j in self.state['jobs'].values())}

  def discover(self, config):
    dest=destination_id(config)
    changed=False
    for directory in self.root.iterdir():
      parsed=split_segment(directory.name)
      if not parsed or directory.is_symlink() or not directory.is_dir() or not self.preserved(directory):continue
      identity=hashlib.sha256((dest+'\n'+directory.name).encode()).hexdigest()[:32]
      if identity not in self.state['jobs']:
        if len(self.state['jobs'])>=10000: raise TransferError('QUEUE_FULL')
        # Until the closed log reveals the event time, protect the previous
        # segment conservatively too. Never wait for/protect the successor.
        segments=[parsed[0]+'--'+str(n) for n in range(max(0,parsed[1]-1),parsed[1]+1)]
        self.state['jobs'][identity]={'id':identity,'bookmark':directory.name,'destination':dest,
            'segments':segments,'state':'waiting','attempts':0,'retry_at':0,'error':'','created':self.clock()}
        changed=True
    if changed:self.save()

  def prepare(self, identity):
    with self.lock:
      job=dict(self.state['jobs'][identity])
    # recording=None deliberately disallows stale-lock recovery for automatic
    # server uploads. Manual recovery/download remains a separate operation.
    try:
      export=export_bookmarks(self.root,[job['bookmark']],preserved=self.preserved,recording=None)
    except ValueError:
      return False
    included=export['bookmark_metadata']['included_segments']
    sources=[]
    for item in export['files']:
      path=Path(item['path'])
      if path.name not in ALLOWED:continue
      relative=path.resolve().relative_to(self.root.resolve())
      if path.is_symlink() or path.parent.is_symlink() or len(relative.parts)!=2:raise TransferError('INVALID_SOURCE')
      stat=path.stat()
      sources.append((path,relative.as_posix(),stat.st_size,stat.st_mtime_ns))
    for segment in included:
      names={p.name for p,_,_,_ in sources if p.parent.name==segment}
      if not names.intersection({'qcamera.ts','qcamera.mp4'}) or not any(n.startswith(('qlog','rlog')) for n in names):
        raise TransferError('SOURCE_MISSING')
    fingerprint=tuple((n,s,t) for _,n,s,t in sources)
    old=self.observed.get(identity)
    if old is None or old[0]!=fingerprint:
      self.observed[identity]=(fingerprint,self.clock())
      return False
    if self.clock()-old[1]<3.:return False
    if shutil.disk_usage(self.spool).free < sum(s for _,_,s,_ in sources)+128*1024*1024:
      raise TransferError('LOCAL_SPACE')
    temporary=self.spool/(identity+'.part')
    try:
      with zipfile.ZipFile(temporary,'w',compression=zipfile.ZIP_STORED,allowZip64=True,strict_timestamps=False) as bundle:
        for path,name,size,stamp in sources:
          if any(p.name.endswith('.lock') for p in path.parent.iterdir()):return False
          bundle.write(path,name)
          stat=path.stat()
          if (stat.st_size,stat.st_mtime_ns)!=(size,stamp):return False
        # Metadata in the ZIP comment keeps the segment directory layout intact.
        metadata=dict(export['bookmark_metadata'],format='kisa-bookmark-sftp-v1',media='qcamera',queue_id=identity)
        bundle.comment=json.dumps(metadata,ensure_ascii=False,sort_keys=True).encode('utf8')
      with temporary.open('r+b') as file:os.fsync(file.fileno())
      for path,_,size,stamp in sources:
        stat=path.stat()
        if (stat.st_size,stat.st_mtime_ns)!=(size,stamp) or any(p.name.endswith('.lock') for p in path.parent.iterdir()):return False
      digest=hash_file(temporary)
      archive=self.spool/(identity+'.'+digest+'.zip')
      os.replace(temporary,archive)
      remote_name=job.get('remote_name')
      name_sequence=job.get('name_sequence', 0)
      if job.get('sha256') != digest:
        _, timestamp=segment_bookmark_timing(self.root, job['bookmark'])
        if timestamp is None:
          timestamp=min(stamp for path,_,_,stamp in sources
                        if path.parent.name==job['bookmark'] and path.name.startswith(('qlog','rlog')))/1e9
        base=bookmark_archive_name(timestamp, fractional=True).removesuffix('.zip')
        # A rebuilt ZIP must never resume another archive's partial transfer.
        name_sequence += int(remote_name is not None)
      with self.lock:
        if job.get('sha256') != digest:
          used={j.get('remote_name') for j in self.state['jobs'].values()}
          while True:
            suffix=f'-{name_sequence+1}' if name_sequence else ''
            remote_name=f'{base}{suffix}.zip'
            if remote_name not in used:
              break
            name_sequence+=1
        self.state['jobs'][identity].update(state='ready',segments=included,archive=archive.name,
                                          sha256=digest,size=archive.stat().st_size,error='',retry_at=0,
                                          remote_name=remote_name,name_sequence=name_sequence)
        self.save()
      return True
    finally:
      temporary.unlink(missing_ok=True)

  def scan_step(self):
    if not self.loaded:self.load()
    if not self.params.get_bool('KisaBookmarkUploadEnabled'):
      self.cancel.set()
      return
    config=self.config()
    with self.lock:
      self.discover(config)
      self.error=''
      candidates=[j['id'] for j in self.state['jobs'].values()
                  if j['destination']==destination_id(config) and j['state']=='waiting' and self.clock()>=j['retry_at']]
    # Packaging is independent of the active SFTP transfer. Newly bookmarked
    # originals are recorded/protected before any file reading starts.
    for identity in candidates[:4]:
      if self.stop_event.is_set():break
      try:self.prepare(identity)
      except (OSError, TransferError):
        with self.lock:
          self.state['jobs'][identity].update(error='PREPARE_FAILED',retry_at=self.clock()+15)
          self.error='PREPARE_FAILED'
          self.save()

  def upload_step(self):
    if not self.loaded or not self.params.get_bool('KisaBookmarkUploadEnabled'):return False
    config=self.config()
    with self.lock:
      jobs=[j for j in self.state['jobs'].values() if j['destination']==destination_id(config) and
            j['state'] in ('ready','retry') and self.clock()>=j['retry_at']]
      if not jobs:return False
      job=min(jobs,key=lambda j:j['created'])
      identity=job['id']
      archive=self.spool/job['archive']
      if archive.parent!=self.spool or archive.is_symlink() or not archive.is_file() or hash_file(archive)!=job['sha256']:
        job.update(state='waiting',error='',retry_at=0)
        self.save()
        return False
      job.update(state='uploading',error='')
      self.cancel.clear()
      self.save()
    try:
      self.client_factory(config,key_dir=self.key_dir,cancel=self.cancel).upload(archive,job['sha256'],job.get('remote_name'))
      with self.lock:
        job.update(state='done',error='',completed=self.clock())
        self.save()  # Persist completion before discarding the resume source.
        self.last_done=self.clock()
      archive.unlink(missing_ok=True)
    except (OSError, TransferError) as error:
      with self.lock:
        job['attempts']+=1
        job.update(state='retry',error=str(error) if isinstance(error,TransferError) else 'LOCAL_IO',
                   retry_at=self.clock()+min(300,5*2**min(job['attempts'],6)))
        self.save()
    return True

  def start(self):
    def scan():
      while not self.stop_event.is_set():
        try:self.scan_step()
        except Exception:
          self.error='LOCAL_STATE'
        try:self.publish(self.summary())
        except Exception:self.error='STATUS_UNAVAILABLE'
        self.stop_event.wait(2.)
    def upload():
      while not self.stop_event.is_set():
        try:self.upload_step()
        except Exception:self.error='TRANSFER_FAILED'
        self.stop_event.wait(.5)
    for name,target in (('bookmark-scan',scan),('bookmark-sftp',upload)):
      thread=threading.Thread(target=target,name=name,daemon=True)
      self.threads.append(thread)
      thread.start()

  def stop(self):
    self.stop_event.set()
    self.cancel.set()
    for thread in self.threads:thread.join(timeout=2.)


def hash_file(path):
  digest=hashlib.sha256()
  with Path(path).open('rb') as stream:
    for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
  return digest.hexdigest()
