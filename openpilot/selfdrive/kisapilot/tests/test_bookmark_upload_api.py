import ast
import base64
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from flask import Flask, jsonify
from openpilot.selfdrive.kisapilot.bookmark_upload_api import UploadApi, private_path
from openpilot.selfdrive.kisapilot.bookmark_upload_keys import UploadKeys
from openpilot.selfdrive.kisapilot.bookmark_sftp import FIELDS, TransferError
from openpilot.selfdrive.kisapilot.tests.test_bookmark_upload import Params


class TestUploadApi(unittest.TestCase):
  def setUp(self):
    self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
    self.root=Path(self.temp.name);self.keys=self.root/'private';self.keys.mkdir()
    self.params=Params();self.params.put_bool('KisaBookmarkUploadEnabled',False)
    self.app=Flask(__name__);self.api=UploadApi(self.app,self.params,Params(),self.root/'logs',self.keys)
    self.client=self.app.test_client()

  def test_config_uses_correct_native_parameter_types_and_no_embedded_server(self):
    body=self.client.get('/bookmarks/server/config').json
    body.update(host='unit.invalid',port=2222,enabled=False)
    response=self.client.post('/bookmarks/server/config',json=body)
    self.assertEqual(response.status_code,200)
    self.assertIs(type(self.params.get(FIELDS['port'])),int)
    self.assertIs(type(self.params.get(FIELDS['rate_kbps'])),int)
    self.assertEqual(self.client.get('/bookmarks/server/config').json['host'],'unit.invalid')
    self.assertNotIn('unit.invalid',json.dumps(response.json))

  def test_enable_requires_uploaded_key_and_host_trust(self):
    body=self.client.get('/bookmarks/server/config').json;body['enabled']=True
    self.assertEqual(self.client.post('/bookmarks/server/config',json=body).json['error'],'KEY_MISSING')
    (self.keys/'key.pem').write_bytes(b'-----BEGIN PRIVATE KEY-----\n')
    self.assertEqual(self.client.post('/bookmarks/server/config',json=body).json['error'],'HOST_KEY_REQUIRED')
    (self.keys/'known_hosts').write_text('trusted')
    self.assertEqual(self.client.post('/bookmarks/server/config',json=body).status_code,200)

  def test_status_response_never_contains_server_fields_or_key_contents(self):
    response=self.client.get('/bookmarks/server/status')
    self.assertEqual(response.status_code,200)
    for value in ('test.invalid','tester','/incoming','key.pem'):
      self.assertNotIn(value,response.get_data(as_text=True))

  def test_key_listing_only_returns_name_and_excludes_transport_key(self):
    (self.keys/'key.pem').write_bytes(b'-----BEGIN PRIVATE KEY-----\nnot-returned-private-bytes')
    (self.keys/'transport.pem').write_bytes(b'-----BEGIN PRIVATE KEY-----\n')
    (self.keys/'known_hosts').write_text('private-hostname')
    self.assertEqual(self.client.get('/bookmarks/server/keys').json,{'keys':[{'name':'key.pem'}]})

  def test_plaintext_key_body_is_rejected(self):
    response=self.client.post('/bookmarks/server/key-upload',data=b'-----BEGIN PRIVATE KEY-----\n',content_type='application/octet-stream')
    self.assertEqual(response.status_code,409)
    self.assertFalse(list(self.keys.glob('*.pem')))

  def test_connection_test_reports_success_only_after_test_finishes(self):
    entered=threading.Event();release=threading.Event()
    def run():entered.set();release.wait(2)
    with patch('openpilot.selfdrive.kisapilot.bookmark_upload_api.SftpClient') as client:
      client.return_value.test_connection.side_effect=run
      response=self.client.post('/bookmarks/server/test',json={})
      self.assertEqual(response.status_code,202);self.assertTrue(entered.wait(1))
      self.assertEqual(self.client.get('/bookmarks/server/status').json['connection_test']['state'],'testing')
      release.set()
      for _ in range(100):
        if self.api.test_result['state']!='testing':break
        time.sleep(.005)
      self.assertEqual(self.api.test_result['state'],'success')

  def test_private_paths_are_denied_even_through_a_symlink_or_parent_copy(self):
    with patch('openpilot.selfdrive.kisapilot.bookmark_upload_api.KEY_DIR',self.keys):
      self.assertTrue(private_path(self.keys/'key.pem'))
      self.assertFalse(private_path(self.root))
      self.assertTrue(private_path(self.root,ancestors=True))

  def test_legacy_phone_download_and_status_routes_cannot_take_over_server_queue(self):
    source=(Path(__file__).parents[1]/'kisa_agent.py').read_text(encoding='utf8')
    tree=ast.parse(source)
    names={'automatic_bookmark_status','automatic_bookmark_transfer_status'}
    self.params.put_bool("KisaBookmarkUploadEnabled",True)
    env=dict(app=self.app,jsonify=jsonify,params=self.params)
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[]),'actual_legacy_routes','exec'),env)
    self.assertEqual(self.client.get('/bookmarks/auto').json['bookmarks'],[])
    self.assertIn('ignored',self.client.post('/bookmarks/auto/status',json={'state':'saved'}).json)

  def test_agent_main_registers_real_api_and_stops_threads_on_exit(self):
    source=(Path(__file__).parents[1]/'kisa_agent.py').read_text(encoding='utf8')
    main=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='main')
    app=Flask('agent_main_test')
    created=[]
    def factory(application,params,shared):
      api=UploadApi(application,params,shared,self.root/'main-logs',self.keys);created.append(api);return api
    env=dict(params=self.params,Params=lambda *a:Params(),app=app,threading=threading,udp_broadcast_loop=lambda:None,HTTP_PORT=5005)
    with patch('openpilot.selfdrive.kisapilot.bookmark_upload_api.UploadApi',side_effect=factory),patch.object(app,'run'):
      exec(compile(ast.Module(body=[main],type_ignores=[]),'actual_agent_main','exec'),env)
      env['main']()
    self.assertTrue(created[0].worker.stop_event.is_set())
    self.assertTrue(all(not t.is_alive() for t in created[0].worker.threads))
    self.assertEqual(app.test_client().get('/bookmarks/server/config').status_code,200)


  def test_switch_to_phone_retains_failed_server_jobs_and_disables_upload(self):
    from openpilot.selfdrive.kisapilot.bookmark_sftp import destination_id, read_config
    job={'bookmark':'route--4','state':'retry','destination':destination_id(read_config(self.params))}
    self.api.worker.state['jobs']['pending']=job.copy()
    result=self.client.post('/bookmarks/server/target',json={'target':'phone'})
    self.assertEqual(result.status_code,200)
    self.assertEqual(result.json['pending_bookmarks'],['route--4'])
    self.assertFalse(self.params.get_bool('KisaBookmarkUploadEnabled'))
    self.assertEqual(self.api.worker.state['jobs']['pending'],job)
    self.assertEqual(self.client.get('/bookmarks/server/config').json['save_target'],'phone')
    self.assertTrue(self.api.worker.cancel.is_set())

  def test_password_mode_needs_bound_password_but_no_private_key(self):
    from openpilot.selfdrive.kisapilot.bookmark_sftp import password_path
    body=self.client.get('/bookmarks/server/config').json
    body.update(auth_mode='password',key_name='',enabled=True)
    self.assertEqual(self.client.post('/bookmarks/server/config',json=body).json['error'],'PASSWORD_REQUIRED')
    password_path(body,self.keys).write_text('synthetic-password')
    (self.keys/'known_hosts').write_text('trusted')
    self.assertEqual(self.client.post('/bookmarks/server/config',json=body).status_code,200)
    self.assertNotIn('synthetic-password',self.client.get('/bookmarks/server/config').get_data(as_text=True))
    self.assertTrue(self.client.get('/bookmarks/server/config').json['password_configured'])
    body['host']='another.invalid'
    self.assertEqual(self.client.post('/bookmarks/server/config',json=body).json['error'],'PASSWORD_REQUIRED')

  def test_key_nonce_cannot_be_used_for_password(self):
    self.api.keys.nonces['test-nonce']=(time.monotonic()+120,'key','')
    response=self.client.post('/bookmarks/server/password',json={'nonce':'test-nonce','chunks':['unused']})
    self.assertEqual(response.json['error'],'KEY_UPLOAD_EXPIRED')


class TestEncryptedKeyUpload(unittest.TestCase):
  def test_encrypted_password_is_account_bound_one_time_and_not_listed(self):
    from cryptography.hazmat.primitives import serialization,hashes
    from cryptography.hazmat.primitives.asymmetric import padding
    from openpilot.selfdrive.kisapilot.bookmark_sftp import read_config,password_path
    binary=str(Path('C:/Program Files/Git/mingw64/bin/openssl.exe')) if os.name=='nt' else shutil.which('openssl')
    with tempfile.TemporaryDirectory() as folder:
      manager=UploadKeys(folder);config=read_config(Params());binding=password_path(config,folder).name
      original=shutil.which
      with patch('openpilot.selfdrive.kisapilot.bookmark_upload_keys.shutil.which',side_effect=lambda name:binary if name=='openssl' else original(name)):
        public=manager.public_key('password',binding)
        key=serialization.load_der_public_key(base64.b64decode(public['public_key']))
        secret=b'fixture-password-not-for-use'
        cipher=key.encrypt(secret,padding.OAEP(mgf=padding.MGF1(hashes.SHA256()),algorithm=hashes.SHA256(),label=None))
        body={'nonce':public['nonce'],'chunks':[base64.b64encode(cipher).decode()]}
        self.assertEqual(manager.receive_password(body,config),{'configured':True})
        self.assertEqual(password_path(config,folder).read_bytes(),secret)
        self.assertEqual(manager.inventory(),[])
        with self.assertRaisesRegex(TransferError,'KEY_UPLOAD_EXPIRED'):manager.receive_password(body,config)
        public=manager.public_key('password',binding);body['nonce']=public['nonce']
        config['user']='another'
        with self.assertRaisesRegex(TransferError,'KEY_UPLOAD_EXPIRED'):manager.receive_password(body,config)

  def test_real_openssl_oaep_upload_is_one_time_and_does_not_return_private_key(self):
    binary=shutil.which('openssl')
    if not binary:
      candidate=Path('C:/Program Files/Git/mingw64/bin/openssl.exe')
      binary=str(candidate) if candidate.exists() else None
    if not binary or not shutil.which('ssh-keygen'):self.skipTest('test crypto tools unavailable')
    with tempfile.TemporaryDirectory() as folder:
      root=Path(folder);key=root/'client-key'
      subprocess.run([shutil.which('ssh-keygen'),'-q','-t','ed25519','-N','','-f',str(key)],check=True,capture_output=True)
      plain=key.read_bytes();manager=UploadKeys(root/'managed')
      from openpilot.selfdrive.kisapilot.bookmark_upload_keys import atomic_write as real_write
      def write_key(path,data):
        if os.name=='nt' and Path(path).name!='transport.pem':
          # Windows OpenSSH requires an ACL, whereas C4 uses chmod 0600. Let
          # ssh-keygen create the test ACL before replacing just its contents.
          subprocess.run([shutil.which('ssh-keygen'),'-q','-t','ed25519','-N','','-f',str(path)],check=True,capture_output=True)
          Path(path).write_bytes(data)
          Path(str(path)+'.pub').unlink()
        else:real_write(path,data)
      original=shutil.which
      with patch('openpilot.selfdrive.kisapilot.bookmark_upload_keys.shutil.which',side_effect=lambda name:binary if name=='openssl' else original(name)),patch('openpilot.selfdrive.kisapilot.bookmark_upload_keys.atomic_write',side_effect=write_key):
        public=manager.public_key();der=root/'public.der';der.write_bytes(base64.b64decode(public['public_key']))
        chunks=[]
        for i in range(0,len(plain),190):
          result=subprocess.run([binary,'pkeyutl','-encrypt','-pubin','-keyform','DER','-inkey',str(der),
            '-pkeyopt','rsa_padding_mode:oaep','-pkeyopt','rsa_oaep_md:sha256','-pkeyopt','rsa_mgf1_md:sha256'],
            input=plain[i:i+190],capture_output=True,check=True)
          chunks.append(base64.b64encode(result.stdout).decode())
        body=dict(nonce=public['nonce'],name='server.key',chunks=chunks)
        result=manager.receive(body)
        self.assertEqual((root/'managed'/result['name']).read_bytes(),plain)
        self.assertNotIn(plain.decode(),json.dumps(result))
        self.assertEqual(manager.inventory(),[result])
        with self.assertRaises(TransferError):manager.receive(body)


if __name__=='__main__':unittest.main()
