"""TCP adapter for the navigation app wire protocol."""
import json
import socket
import threading

from openpilot.selfdrive.kisapilot.nav_source import NaviSourceMux as NaviSourceMux

DISCOVERY_PORTS = (7705, 17705, 17706)
READ_TIMEOUT = 10.0
MAX_LINE = 2 * 1024 * 1024


class NaviTcpServer:
  def __init__(self,mux,port=7712,host='0.0.0.0'):
    self.mux,self.port,self.host = mux,port,host
    self.running = threading.Event()
    self.running.set()
    self.slots = threading.BoundedSemaphore(8)
    self.connections,self.connection_lock,self.server = set(),threading.Lock(),None

  def stop(self):
    self.running.clear()
    if self.server:
      self.server.close()
    with self.connection_lock:
      connections = list(self.connections)
    for conn in connections:
      try:
        conn.shutdown(socket.SHUT_RDWR)
      except OSError:
        pass
      conn.close()

  def _client(self,conn,addr):
    key = object()
    self.mux.open(key,addr)
    try:
      conn.settimeout(READ_TIMEOUT)
      conn.setsockopt(socket.SOL_SOCKET,socket.SO_KEEPALIVE,1)
      with conn,conn.makefile('r',encoding='utf-8',errors='replace') as stream:
        while self.running.is_set():
          line = stream.readline(MAX_LINE+1)
          if not line or len(line)>MAX_LINE:
            break
          try:
            self.mux.feed(key,json.loads(line))
          except (ValueError,TypeError):
            continue
    except (OSError,TimeoutError):
      pass
    finally:
      conn.close()
      with self.connection_lock:
        self.connections.discard(conn)
      self.mux.close(key)
      self.slots.release()

  def serve(self):
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as server:
      self.server = server
      server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
      server.bind((self.host,self.port))
      self.port = server.getsockname()[1]
      server.listen(8)
      server.settimeout(.5)
      while self.running.is_set():
        self.mux.tick()
        try:
          conn,addr = server.accept()
        except socket.timeout:
          continue
        except OSError:
          if not self.running.is_set():
            break
          raise
        if not self.slots.acquire(blocking=False):
          conn.close()
          continue
        with self.connection_lock:
          self.connections.add(conn)
        threading.Thread(target=self._client,args=(conn,addr),daemon=True).start()
