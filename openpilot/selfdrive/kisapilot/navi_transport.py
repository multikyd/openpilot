"""Concurrent navigation connections with one coherent selected source."""
import json
import math
import socket
import threading
import time

DISCOVERY_PORTS = (7705, 17705, 17706)
FRESH_SECONDS = 4.0
READ_TIMEOUT = 10.0
MAX_LINE = 2 * 1024 * 1024

class NaviSourceMux:
  def __init__(self, dispatch, changed, received=lambda *a: None, clock=time.monotonic):
    self.dispatch, self.changed, self.received, self.clock = dispatch, changed, received, clock
    self.clients, self.selected, self.lock = {}, None, threading.RLock()

  def open(self, key, addr):
    with self.lock:
      self.clients[key] = dict(addr=addr, source='legacy', instance='', foreground=None,
        foreground_epoch=0, active=True, rg_at=None, seq=-1, cache={})

  def _eligible(self, c, now):
    return c['active'] and c['rg_at'] is not None and now-c['rg_at'] <= FRESH_SECONDS

  def _select(self, now):
    candidates = {k:c for k,c in self.clients.items() if self._eligible(c, now)}
    foreground = {k:c for k,c in candidates.items() if c['foreground'] is True}
    if foreground:
      target = max(foreground, key=lambda k:(foreground[k]['foreground_epoch'],
                                             k == self.selected, foreground[k]['rg_at']))
    elif self.selected in candidates:
      target = self.selected
    else:
      target = max(candidates, key=lambda k:candidates[k]['rg_at'], default=None)
    if target == self.selected:
      return False
    self.selected = target
    c = self.clients.get(target)
    self.changed(c['source'] if c else None, c['addr'] if c else None)
    if c:
      # Replay only this app, position before signals; never reuse stale signals.
      for kind in ('rgdata', 'route', 'ssinf', 'sinf', 'complexCrossroad'):
        cached = c['cache'].get(kind)
        if cached and (kind == 'route' or now-cached[0] <= FRESH_SECONDS):
          self.dispatch(cached[1])
    return True

  @staticmethod
  def _valid_rg(rg):
    try:
      lat, lon = float(rg['vpPosPointLat']), float(rg['vpPosPointLon'])
      return math.isfinite(lat) and math.isfinite(lon) and abs(lat)<=90 and abs(lon)<=180 and (lat!=0 or lon!=0)
    except (KeyError, TypeError, ValueError, OverflowError):
      return False

  def feed(self, key, obj):
    if not isinstance(obj, dict):
      return
    with self.lock:
      c = self.clients.get(key)
      if c is None:
        return
      now = self.clock()
      state = obj.get('navi_state')
      if isinstance(state, dict):
        seq, instance = state.get('sequence'), state.get('instance_id')
        if not isinstance(seq,int) or isinstance(seq,bool) or not isinstance(instance,str):
          return
        if instance == c['instance'] and seq <= c['seq']:
          return
        if instance != c['instance']:
          c.update(seq=-1, rg_at=None, cache={})
        c.update(instance=instance, seq=seq, foreground=state.get('foreground') is True,
                 active=state.get('active') is True)
        epoch = state.get('foreground_since_elapsed_ms',0)
        c['foreground_epoch'] = epoch if isinstance(epoch,(int,float)) and math.isfinite(epoch) else 0
      if obj.get('source') in ('naver-map','tmap'):
        c['source'] = obj['source']
      outgoing = {}
      for kind in ('rgdata','route','ssinf','sinf','complexCrossroad'):
        alias = 'vrtx' if kind=='route' and 'vrtx' in obj else kind
        if alias not in obj:
          continue
        if kind=='rgdata':
          if not isinstance(obj[alias],dict) or not self._valid_rg(obj[alias]):
            continue
          c['rg_at'] = now
        packet = {k:v for k,v in obj.items() if k in ('source','bridge_version','timestamp_ms','raw_rgdata','raw_naver','navi_state')}
        packet[kind] = obj[alias]
        c['cache'][kind] = (now,packet)
        outgoing[kind] = packet
      switched = self._select(now)
      if key == self.selected:
        if not switched:
          for packet in outgoing.values():
            self.dispatch(packet)
        if 'rgdata' in outgoing:
          self.received(c['source'],outgoing['rgdata'])

  def close(self,key):
    with self.lock:
      self.clients.pop(key,None)
      self._select(self.clock())

  def tick(self):
    with self.lock:
      self._select(self.clock())

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
