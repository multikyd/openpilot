"""Concurrent navigation connections with one coherent selected source."""
import json
import math
import socket
import threading
import time
from openpilot.selfdrive.kisapilot.signal_follow import directional_signal, signal_from_sinf, usable_signal

DISCOVERY_PORTS = (7705, 17705, 17706)
FRESH_SECONDS = 4.0
READ_TIMEOUT = 10.0
MAX_LINE = 2 * 1024 * 1024
EVENT_TYPES = ('rgdata', 'route', 'ssinf', 'sinf', 'complexCrossroad')
PACKET_METADATA = frozenset((
  'source', 'bridge_version', 'timestamp_ms', 'raw_rgdata', 'raw_naver',
  'raw_ssinf', 'raw_sinf', 'signal_update', 'navi_state',
))

class NaviSourceMux:
  def __init__(self, dispatch, changed, received=lambda *a: None, clock=time.monotonic):
    self.dispatch, self.changed, self.received, self.clock = dispatch, changed, received, clock
    self.clients, self.selected, self.lock = {}, None, threading.RLock()
    self.signal_source, self.signal_original_at = None, float('-inf')

  def open(self, key, addr):
    with self.lock:
      self.clients[key] = dict(addr=addr, source='legacy', instance='', foreground=None,
        foreground_epoch=0, active=True, rg_at=None, seq=-1, cache={})

  def _eligible(self, c, now):
    return c['active'] and c['rg_at'] is not None and now-c['rg_at'] <= FRESH_SECONDS

  def _select(self, now):
    target = None
    best_priority = None
    for key, client in self.clients.items():
      if not self._eligible(client, now):
        continue
      foreground = client['foreground'] is True
      # Foreground ownership wins first. Otherwise keep the current eligible app.
      priority = (foreground, client['foreground_epoch'] if foreground else 0,
                  key == self.selected, client['rg_at'])
      if best_priority is None or priority > best_priority:
        target, best_priority = key, priority
    if target == self.selected:
      return False
    self.selected = target
    c = self.clients.get(target)
    self.changed(c['source'] if c else None, c['addr'] if c else None)
    if c:
      # Signals have their own live arbitration; a UI/app switch must not replay old phases.
      for kind in ('rgdata', 'route', 'complexCrossroad'):
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

  def _signal(self, key, packet, kind, now):
    value = packet[kind]
    if not isinstance(value, dict):
      return
    data = signal_from_sinf(value) if kind == 'sinf' else value
    if not usable_signal(directional_signal(data, now, {}, {})):
      return
    update = packet.get('signal_update', {})
    update = update if isinstance(update, dict) else {}
    if update.get('status') == 'phase_expired':
      return
    local = update.get('kind') == 'local_countdown'
    # An older extrapolated phase from the other provider must not overwrite a
    # phase that has just arrived. A new original observation can always replace it.
    if local and key != self.signal_source and now-self.signal_original_at <= FRESH_SECONDS:
      return
    self.signal_source = key
    if not local:
      self.signal_original_at = now
    self.dispatch(packet)

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
      metadata = None
      outgoing = {}
      for kind in EVENT_TYPES:
        alias = 'vrtx' if kind == 'route' and 'vrtx' in obj else kind
        if alias not in obj:
          continue
        value = obj[alias]
        if kind == 'rgdata':
          if not isinstance(value, dict) or not self._valid_rg(value):
            continue
          c['rg_at'] = now
        if metadata is None:
          metadata = {name: field for name, field in obj.items() if name in PACKET_METADATA}
        packet = metadata.copy()
        packet[kind] = value
        c['cache'][kind] = (now, packet)
        outgoing[kind] = packet
      switched = self._select(now)
      for kind in ('ssinf', 'sinf'):
        if kind in outgoing:
          self._signal(key, outgoing[kind], kind, now)
      if key == self.selected:
        if not switched:
          for kind, packet in outgoing.items():
            if kind not in ('ssinf', 'sinf'):
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
