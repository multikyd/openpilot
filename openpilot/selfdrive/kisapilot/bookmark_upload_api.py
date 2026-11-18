"""Agent settings/key APIs. No private key or subprocess diagnostic is returned."""
import json
from pathlib import Path
import threading
import time

from flask import jsonify, request

from openpilot.selfdrive.kisapilot.bookmark_sftp import FIELDS, KEY_DIR, SftpClient, TransferError, read_config, validate_config
from openpilot.selfdrive.kisapilot.bookmark_upload import BookmarkUploader
from openpilot.selfdrive.kisapilot.bookmark_upload_keys import UploadKeys


def private_path(path, ancestors=False):
  if not path:return False
  try:
    path=Path(path).resolve()
    key=KEY_DIR.resolve()
    return path==key or key in path.parents or (ancestors and path in key.parents)
  except (OSError,ValueError,TypeError):return True


class UploadApi:
  def __init__(self, app, params, shared_params, root='/data/media/0/realdata', key_dir=KEY_DIR):
    self.params=params
    self.keys=UploadKeys(key_dir)
    self.test_result={'state':'idle'}
    self.test_lock=threading.Lock()
    def publish(report):
      if shared_params is not None:
        shared_params.put('KisaBookmarkTransferStatus',json.dumps(dict(report,ts=time.monotonic())))
    self.worker=BookmarkUploader(root,params,publish,key_dir=key_dir)
    def guarded(function):
      def call(*args,**kwargs):
        try:return function(*args,**kwargs)
        except TransferError as error:return jsonify({'error':str(error)}),409
        except Exception:return jsonify({'error':'LOCAL_IO'}),500
      call.__name__=function.__name__
      return call

    @app.route('/bookmarks/server/config',methods=['GET','POST'])
    @guarded
    def server_config():
      if request.method=='GET':return jsonify(read_config(params))
      if request.content_length is None or request.content_length>8192:raise TransferError('INVALID_CONFIG')
      body=request.get_json(silent=True)
      if not isinstance(body,dict):raise TransferError('INVALID_CONFIG')
      if self.worker.summary()['uploading']:raise TransferError('TRANSFER_BUSY')
      config=validate_config(body,complete=bool(body.get('enabled')))
      if config['enabled']:
        names={key['name'] for key in self.keys.inventory()}
        if config['key_name'] not in names:raise TransferError('KEY_MISSING')
        if not (self.keys.directory/'known_hosts').is_file():raise TransferError('HOST_KEY_REQUIRED')
      # Disable before updating fields, so no worker can use a mixed destination.
      params.put_bool('KisaBookmarkUploadEnabled',False,block=True)
      for field,key in FIELDS.items():params.put(key,config[field],block=True)
      params.put_bool('KisaBookmarkUploadEnabled',config['enabled'],block=True)
      return jsonify({'ok':True})

    @app.route('/bookmarks/server/status',methods=['GET'])
    @guarded
    def server_status():
      return jsonify(dict(self.worker.summary(),connection_test=self.test_result))

    @app.route('/bookmarks/server/keys',methods=['GET'])
    @guarded
    def server_keys():return jsonify({'keys':self.keys.inventory()})

    @app.route('/bookmarks/server/key-upload/public',methods=['GET'])
    @guarded
    def key_upload_public():return jsonify(self.keys.public_key())

    @app.route('/bookmarks/server/key-upload',methods=['POST'])
    @guarded
    def key_upload():
      if request.content_length is None or request.content_length>65536:raise TransferError('INVALID_KEY_UPLOAD')
      return jsonify(self.keys.receive(request.get_json(silent=True)))

    @app.route('/bookmarks/server/host-key',methods=['POST'])
    @guarded
    def host_key():return jsonify(self.keys.scan_host(read_config(params)))

    @app.route('/bookmarks/server/trust-host',methods=['POST'])
    @guarded
    def trust_host():
      body=request.get_json(silent=True) or {}
      self.keys.trust_host(body.get('challenge',''),read_config(params))
      return jsonify({'ok':True})

    @app.route('/bookmarks/server/test',methods=['POST'])
    @guarded
    def server_test():
      config=validate_config(read_config(params))
      with self.test_lock:
        if self.test_result['state']=='testing':raise TransferError('BUSY')
        self.test_result={'state':'testing'}
      def run():
        try:
          SftpClient(config,self.keys.directory).test_connection()
          self.test_result={'state':'success'}
        except TransferError as error:self.test_result={'state':'error','error':str(error)}
        except Exception:self.test_result={'state':'error','error':'SFTP_FAILED'}
      threading.Thread(target=run,name='sftp-test',daemon=True).start()
      return jsonify({'state':'testing'}),202

  def start(self):self.worker.start()

  def stop(self):self.worker.stop()
