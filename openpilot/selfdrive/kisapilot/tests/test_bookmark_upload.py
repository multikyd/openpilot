import ast
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import zipfile

from openpilot.selfdrive.kisapilot import bookmark_export as export
from openpilot.selfdrive.kisapilot.bookmark_upload import BookmarkUploader, pending_segments
from openpilot.selfdrive.kisapilot.bookmark_sftp import FIELDS, SftpClient, TransferError, validate_config
from openpilot.selfdrive.kisapilot.bookmark_status import status_text


class Params:
  def __init__(self):
    self.values={FIELDS[k]:v for k,v in dict(host='test.invalid',port=22,user='tester',remote_dir='/incoming',key_name='key.pem',rate_kbps=4096).items()}
    self.values['KisaBookmarkUploadEnabled']=True
  def get(self,key):return self.values.get(key,'')
  def get_bool(self,key):return bool(self.values.get(key,False))
  def put(self,key,value,**kwargs):
    if key in (FIELDS['port'],FIELDS['rate_kbps']) and type(value) is not int:raise TypeError('numeric param')
    self.values[key]=value
  def put_bool(self,key,value,**kwargs):self.values[key]=value


class TestBookmarkUpload(unittest.TestCase):
  def setUp(self):
    self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
    self.root=Path(self.tmp.name);self.params=Params();self.now=100.;self.marks=set();self.sent=[]
    class Client:
      def __init__(client,*args,**kwargs):pass
      def upload(client,path,digest):self.sent.append((path.read_bytes(),digest));return 'saved'
    self.factory=Client
    self.worker=self.new_worker();self.worker.load()
    self.offset=patch.object(export,'segment_bookmark_offset',return_value=40.)
    self.offset.start();self.addCleanup(self.offset.stop)

  def new_worker(self):
    return BookmarkUploader(self.root,self.params,preserved=lambda p:p.name in self.marks,
                            client_factory=self.factory,clock=lambda:self.now)

  def segment(self,n,mark=False,locked=False):
    path=self.root/('route--'+str(n));path.mkdir()
    for name,data in [('rlog.zst',b'raw-rlog'),('qlog.zst',b'qlog'),('qcamera.ts',b'video'),('fcamera.hevc',b'large-video')]:
      (path/name).write_bytes(data)
    if locked:(path/'rlog.lock').touch()
    if mark:self.marks.add(path.name)
    return path

  def prepare(self):
    self.worker.scan_step();self.now+=4;self.worker.scan_step()

  def job(self):return next(iter(self.worker.state['jobs'].values()))

  def test_locked_segment_never_prepares_even_after_long_wait(self):
    path=self.segment(1,True,True)
    self.prepare();self.now+=1000;self.worker.scan_step()
    self.assertEqual(self.job()['state'],'waiting');self.assertFalse(self.worker.upload_step())
    (path/'rlog.lock').unlink();self.prepare()
    self.assertEqual(self.job()['state'],'ready')

  def test_zip_has_only_qcamera_and_original_logs_and_no_successor(self):
    self.segment(0);self.segment(1,True);self.segment(2,locked=True);self.prepare()
    job=self.job()
    with zipfile.ZipFile(self.worker.spool/job['archive']) as bundle:
      self.assertEqual(set(bundle.namelist()),{'route--1/qcamera.ts','route--1/qlog.zst','route--1/rlog.zst'})
      self.assertEqual(bundle.read('route--1/rlog.zst'),b'raw-rlog')
      self.assertTrue(all(info.compress_type==zipfile.ZIP_STORED for info in bundle.infolist()))
      self.assertEqual(json.loads(bundle.comment)['selection_policy'],'bookmark-history-30s')
    self.worker.upload_step()
    self.assertEqual(self.job()['state'],'done');self.assertEqual(len(self.sent),1)
    self.assertTrue((self.root/'route--1/fcamera.hevc').is_file())

  def test_early_bookmark_includes_previous_segment(self):
    self.segment(0);self.segment(1,True)
    with patch.object(export,'segment_bookmark_offset',return_value=10.):self.prepare()
    self.assertEqual(self.job()['segments'],['route--0','route--1'])

  def test_missing_video_is_retryable_not_falsely_completed(self):
    path=self.segment(0,True);(path/'qcamera.ts').unlink();self.prepare()
    self.assertEqual(self.job()['state'],'waiting');self.assertNotEqual(self.worker.summary()['state'],'saved')
    self.assertEqual(self.sent,[])

  def test_new_bookmark_queues_while_old_upload_is_blocked(self):
    self.segment(0,True);self.prepare();entered=threading.Event();release=threading.Event()
    class Client:
      def __init__(client,*a,**kw):pass
      def upload(client,*a):entered.set();release.wait(5)
    self.worker.client_factory=Client
    thread=threading.Thread(target=self.worker.upload_step)
    thread.start()
    try:
      self.assertTrue(entered.wait(2))
      self.segment(1,True,True);self.worker.scan_step()
      self.assertEqual(len(self.worker.state['jobs']),2)
      self.assertEqual(self.worker.summary()['pending'],2)
      self.assertEqual(len(json.loads((self.worker.spool/'queue.json').read_text())['jobs']),2)
      self.assertIn('route--1',pending_segments(self.root))
    finally:release.set();thread.join(5)

  def test_restart_preserves_inflight_zip_for_resume(self):
    self.segment(0,True);self.prepare()
    job=self.job();archive=self.worker.spool/job['archive'];original=archive.read_bytes()
    job['state']='uploading';self.worker.save()
    restarted=self.new_worker();restarted.load()
    self.assertEqual(next(iter(restarted.state['jobs'].values()))['state'],'ready')
    self.assertEqual(archive.read_bytes(),original)
    restarted.upload_step()
    self.assertEqual(len(self.sent),1)

  def test_failure_keeps_same_archive_and_persistent_retry(self):
    self.segment(0,True);self.prepare();original=self.job()['archive']
    class Client:
      def __init__(client,*a,**kw):pass
      def upload(client,*a):raise TransferError('NETWORK')
    self.worker.client_factory=Client;self.worker.upload_step()
    self.assertEqual(self.job()['state'],'retry');self.assertEqual(self.job()['archive'],original)
    self.assertTrue((self.worker.spool/original).is_file())
    self.assertEqual(self.worker.summary()['state'],'error')
    restarted=self.new_worker();restarted.load()
    self.assertEqual(next(iter(restarted.state['jobs'].values()))['error'],'NETWORK')

  def test_acknowledged_bookmark_is_not_queued_again(self):
    self.segment(0,True);self.prepare();self.worker.upload_step()
    restarted=self.new_worker();restarted.load();restarted.scan_step()
    self.assertFalse(restarted.upload_step());self.assertEqual(len(self.sent),1)

  def test_changed_archive_is_rebuilt_not_resumed_as_old_content(self):
    self.segment(0,True);self.prepare()
    (self.worker.spool/self.job()['archive']).write_bytes(b'changed')
    self.assertFalse(self.worker.upload_step());self.assertEqual(self.job()['state'],'waiting')
    self.assertEqual(self.sent,[])

  def test_corrupt_queue_is_not_silently_reset(self):
    (self.worker.spool/'queue.json').write_bytes(b'broken')
    with self.assertRaises(TransferError):self.new_worker().load()

  def test_waiting_jobs_protect_history_but_not_future(self):
    self.segment(2,True,True);self.worker.scan_step()
    self.assertEqual(pending_segments(self.root),{'route--1','route--2'})

  def test_private_configuration_is_not_in_queue_or_status(self):
    self.segment(0,True);self.worker.scan_step()
    data=(self.worker.spool/'queue.json').read_text()+json.dumps(self.worker.summary())
    self.assertNotIn('test.invalid',data);self.assertNotIn('/incoming',data);self.assertNotIn('tester',data)

  def test_prepare_failure_remains_visible_between_scans(self):
    self.segment(0,True)
    with patch.object(self.worker,'prepare',side_effect=TransferError('SOURCE_MISSING')):
      self.worker.scan_step()
    self.worker.scan_step()
    self.assertEqual(self.worker.summary()['state'],'error')

  def test_actual_deleter_skips_pending_segments_and_upload_spool(self):
    self.segment(2,True,True);self.segment(1);self.segment(0)
    self.worker.scan_step()
    source=Path(__file__).resolve().parents[3]/'system/loggerd/deleter.py'
    tree=ast.parse(source.read_text())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='deleter_step')
    class Paths:
      @staticmethod
      def log_root():return str(self.root)
    deleted=[]
    from types import SimpleNamespace
    scope=dict(Paths=Paths,MIN_BYTES=10,MIN_PERCENT=10,DELETE_LAST=[],os=os,
               get_available_bytes=lambda **kw:0,get_available_percent=lambda **kw:0,
               listdir_by_creation=lambda root:['bookmark_uploads','route--1','route--2','route--0'],
               get_preserved_segments=lambda dirs:set(),
               cloudlog=SimpleNamespace(info=lambda *a:None,exception=lambda *a:None),
               shutil=SimpleNamespace(rmtree=lambda p:deleted.append(Path(p).name)))
    exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),scope)
    self.assertEqual(scope['deleter_step']()[0],True)
    self.assertEqual(deleted,['route--0'])


class TestSftpContract(unittest.TestCase):
  def test_upload_uses_resume_only_for_a_shorter_existing_temporary_file(self):
    from openpilot.selfdrive.kisapilot.bookmark_sftp import read_config
    with tempfile.TemporaryDirectory() as directory:
      archive = Path(directory) / 'bookmark.zip'
      archive.write_bytes(b'data')
      client = SftpClient(read_config(Params()))
      for partial, command in ((None, '@put'), (0, '@put'), (2, '@reput'), (4, '@put')):
        with self.subTest(partial=partial), patch.object(client, 'stat', side_effect=[None, partial, 4]), \
             patch.object(client, 'batch', return_value=(0, '', '')) as batch:
          self.assertEqual(client.upload(archive, 'a'*64), 'saved')
          transfer, rename = batch.call_args.args[0][-2:]
          self.assertTrue(transfer.startswith(command+' '))
          self.assertTrue(rename.startswith('@rename '))

  def test_upload_status_distinguishes_stat_upload_and_rename_failures(self):
    from openpilot.selfdrive.kisapilot.bookmark_sftp import read_config
    with tempfile.TemporaryDirectory() as directory:
      archive = Path(directory) / 'bookmark.zip'
      archive.write_bytes(b'data')
      client = SftpClient(read_config(Params()))
      missing = (1, '', 'Can\'t ls: No such file or directory')
      for responses, expected in (
          ([(1, '', 'private path: unexpected failure')], 'SFTP_STAT_FAILED'),
          ([(1, '', 'subsystem not found')], 'SFTP_SUBSYSTEM'),
          ([missing, missing, (1, '', 'private path: unexpected failure')], 'SFTP_UPLOAD_FAILED'),
          ([missing, missing, (1, '', 'remote rename /private/a to /private/b: Failure')], 'SFTP_RENAME_FAILED'),
          ([missing, missing, (1, '', 'remote write: Disk quota exceeded')], 'SFTP_SPACE_OR_QUOTA')):
        with self.subTest(expected=expected), patch.object(client, 'batch', side_effect=responses):
          with self.assertRaises(TransferError) as error:
            client.upload(archive, 'a'*64)
          self.assertEqual(str(error.exception), expected)

  def test_config_rejects_batch_and_option_injection(self):
    for field,value in [('host','-Fbad'),('host','user@host'),('user','x\nput bad'),
                        ('remote_dir','/x\nrm y'),('remote_dir','/x/../y'),('key_name','../key')]:
      config={k:Params().get(v) for k,v in FIELDS.items()};config[field]=value
      with self.assertRaises(TransferError):validate_config(config)

  def test_server_banner_distinguishes_actual_complete_from_waiting(self):
    for state,expected in [('waiting','UPLOAD 2 WAIT'),('receiving','UPLOADING / 1 WAIT'),
                           ('saved','UPLOAD DONE'),('error','UPLOAD RETRY')]:
      self.assertEqual(status_text(dict(state=state,target='server',pending=2,ts=10.),10.)[0],expected)



class TestPasswordTransport(unittest.TestCase):
  def test_password_batchmode_precedes_batch_flag_and_secret_not_in_command(self):
    from openpilot.selfdrive.kisapilot.bookmark_sftp import read_config,password_path
    with tempfile.TemporaryDirectory() as directory:
      keys=Path(directory);(keys/'known_hosts').write_text('trusted')
      config=read_config(Params());config.update(auth_mode='password',key_name='')
      password_path(config,keys).write_text('synthetic-secret')
      with patch('openpilot.selfdrive.kisapilot.bookmark_sftp.shutil.which',side_effect=lambda name:name):
        args=SftpClient(config,keys).command()
      self.assertLess(args.index('-oBatchMode=no'),args.index('-b'))
      self.assertNotIn('synthetic-secret',' '.join(args))
      self.assertNotIn('-i',args)

if __name__=='__main__':unittest.main()
