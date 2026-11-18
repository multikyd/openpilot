"""Real OpenSSH client against a loopback SSH/SFTP server (test-only Paramiko)."""
import base64
import hashlib
import os
from pathlib import Path
import posixpath
import shutil
import socket
import subprocess
import tempfile
import threading
import unittest

from openpilot.selfdrive.kisapilot.bookmark_sftp import SftpClient, TransferError

try:
  import paramiko
except ImportError:
  paramiko=None


@unittest.skipUnless(paramiko and shutil.which('sftp') and shutil.which('ssh-keygen'), 'test SSH tools unavailable')
class TestNativeSftp(unittest.TestCase):
  def setUp(self):
    self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
    self.root=Path(self.temp.name);self.remote=self.root/'remote';self.remote.mkdir()
    self.keys=self.root/'keys';self.keys.mkdir()
    subprocess.run([shutil.which('ssh-keygen'),'-q','-t','ed25519','-N','','-f',str(self.keys/'id')],check=True,capture_output=True)
    pub=(self.keys/'id.pub').read_text().split()[1]
    allowed=paramiko.Ed25519Key(data=base64.b64decode(pub))
    class Auth(paramiko.ServerInterface):
      def check_auth_publickey(self,user,key):
        return paramiko.AUTH_SUCCESSFUL if user=='tester' and key==allowed else paramiko.AUTH_FAILED
      def get_allowed_auths(self,user):return 'publickey'
      def check_channel_request(self,kind,channel):return paramiko.OPEN_SUCCEEDED if kind=='session' else paramiko.OPEN_FAILED_ADMINISTRATIVELY_PROHIBITED
    root=self.remote
    class Handle(paramiko.SFTPHandle):
      def stat(self):return paramiko.SFTPAttributes.from_stat(os.fstat(self.writefile.fileno() if self.writefile else self.readfile.fileno()))
    class FS(paramiko.SFTPServerInterface):
      def local(self,path):
        relative=posixpath.normpath('/'+path).lstrip('/')
        target=(root/relative).resolve()
        if root.resolve() not in target.parents and target!=root.resolve():raise OSError(13,'denied')
        return target
      def canonicalize(self,path):return posixpath.normpath('/'+path)
      def stat(self,path):
        try:return paramiko.SFTPAttributes.from_stat(self.local(path).stat())
        except OSError as error:return paramiko.SFTPServer.convert_errno(error.errno)
      lstat=stat
      def list_folder(self,path):
        try:
          out=[]
          for item in self.local(path).iterdir():
            attr=paramiko.SFTPAttributes.from_stat(item.stat());attr.filename=item.name;out.append(attr)
          return out
        except OSError as error:return paramiko.SFTPServer.convert_errno(error.errno)
      def open(self,path,flags,attr):
        try:
          fd=os.open(self.local(path),flags,0o600)
          stream=os.fdopen(fd,'r+b' if flags & os.O_RDWR else 'wb' if flags & os.O_WRONLY else 'rb')
          handle=Handle(flags);handle.readfile=stream;handle.writefile=stream
          return handle
        except OSError as error:return paramiko.SFTPServer.convert_errno(error.errno)
      def mkdir(self,path,attr):
        try:self.local(path).mkdir();return paramiko.SFTP_OK
        except OSError as error:return paramiko.SFTPServer.convert_errno(error.errno)
      def rename(self,old,new):
        try:os.rename(self.local(old),self.local(new));return paramiko.SFTP_OK
        except OSError as error:return paramiko.SFTPServer.convert_errno(error.errno)
      def remove(self,path):
        try:self.local(path).unlink();return paramiko.SFTP_OK
        except OSError as error:return paramiko.SFTPServer.convert_errno(error.errno)
    self.hostkey=paramiko.RSAKey.generate(2048)
    self.socket=socket.socket();self.socket.bind(('127.0.0.1',0));self.socket.listen(10);self.socket.settimeout(.2)
    self.port=self.socket.getsockname()[1];self.closed=threading.Event();self.transports=[]
    def serve():
      while not self.closed.is_set():
        try:client,_=self.socket.accept()
        except socket.timeout:continue
        except OSError:break
        transport=paramiko.Transport(client);self.transports.append(transport)
        transport.add_server_key(self.hostkey);transport.set_subsystem_handler('sftp',paramiko.SFTPServer,FS)
        try:transport.start_server(server=Auth())
        except (EOFError,OSError,paramiko.SSHException):transport.close()
    self.thread=threading.Thread(target=serve,daemon=True);self.thread.start()
    def close():
      self.closed.set();self.socket.close()
      for t in self.transports:t.close()
      self.thread.join(2)
    self.addCleanup(close)
    self.hosts=self.keys/'known_hosts'
    self.hosts.write_text(f'[127.0.0.1]:{self.port} {self.hostkey.get_name()} {self.hostkey.get_base64()}\n')
    self.config=dict(host='127.0.0.1',port=self.port,user='tester',remote_dir='/incoming',key_name='id',rate_kbps=65536)

  def test_real_reput_resumes_same_zip_then_renames_and_deduplicates(self):
    data=b'original-qcamera-and-logs'*5000
    local=self.root/'bookmark.zip';local.write_bytes(data);digest=hashlib.sha256(data).hexdigest()
    destination=self.remote/'incoming';destination.mkdir()
    partial=destination/('bookmark-'+digest+'.zip.part');partial.write_bytes(data[:8192])
    client=SftpClient(self.config,self.keys)
    self.assertEqual(client.upload(local,digest),'saved')
    final=partial.with_suffix('')
    self.assertEqual(final.read_bytes(),data);self.assertFalse(partial.exists())
    self.assertEqual(client.upload(local,digest),'already_done')

  def test_real_unknown_host_is_rejected(self):
    self.hosts.write_text('')
    with self.assertRaisesRegex(TransferError,'HOST_KEY'):
      SftpClient(self.config,self.keys).test_connection()

  def test_real_test_creates_target_folder_and_removes_only_probe(self):
    self.config['remote_dir']='/home/kyd/realdata'
    SftpClient(self.config,self.keys).test_connection()
    self.assertTrue((self.remote/'home/kyd/realdata').is_dir())
    self.assertEqual(list((self.remote/'home/kyd/realdata').iterdir()),[])


if __name__=='__main__':unittest.main()
