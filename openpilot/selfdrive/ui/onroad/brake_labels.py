"""Bounded, asynchronous human labels. Collects data; never changes a detector/control."""
import json
import os
from pathlib import Path
import time
import uuid
from concurrent.futures import ThreadPoolExecutor


class BrakeLabelWriter:
  def __init__(self, directory='/data/brake_labels'):
    self.directory = Path(directory)
    self.pool = None
    self.pending = None

  def submit(self, sample, label, now, wall):
    if label not in ('ON', 'OFF') or not sample or not 0 <= now-sample['mono'] <= .35:
      return 'NO FRAME'
    if self.pending is not None and not self.pending.done():
      return 'BUSY'
    metadata = {k: v for k, v in sample.items() if k != 'rgb'}
    rgb = sample['rgb']
    if rgb.dtype.name != 'uint8' or rgb.ndim != 3 or rgb.shape[2] != 3 or max(rgb.shape[:2]) > 192:
      return 'NO FRAME'
    metadata.update(label=label, label_mono=now, label_wall=wall, display_only=True, format_version=1)
    if self.pool is None:
      self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='brake_labels')
    # This crop owns its memory; no shared camera buffer is retained or read here.
    self.pending = self.pool.submit(self._save, metadata, rgb.copy())
    return 'SAVING'

  def result(self):
    if self.pending is None or not self.pending.done():
      return None
    pending, self.pending = self.pending, None
    try:
      return pending.result()
    except Exception:
      return 'SAVE ERROR'

  def _save(self, metadata, rgb):
    directory = self.directory
    directory.mkdir(parents=True, exist_ok=True)
    # Refuse further writes instead of filling storage or deleting earlier labels.
    files = list(directory.iterdir())
    if len(files) >= 4000 or sum(p.stat().st_size for p in files if p.is_file()) >= 64*1024*1024:
      return 'STORAGE FULL'
    try:
      metadata['route'] = Path('/data/params/d/CurrentRoute').read_text().strip()
    except OSError:
      metadata['route'] = ''
    name = f"{int(metadata['label_wall']*1000)}-{uuid.uuid4().hex[:10]}"
    image, record = directory/(name+'.ppm'), directory/(name+'.json')
    height, width = rgb.shape[:2]
    metadata['image'] = image.name
    metadata['width'], metadata['height'] = width, height
    def atomic(path, data):
      temporary = path.with_suffix(path.suffix+'.tmp')
      with temporary.open('wb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
      os.replace(temporary, path)
    atomic(image, f'P6\n{width} {height}\n255\n'.encode()+rgb.tobytes())
    atomic(record, json.dumps(metadata, allow_nan=False).encode())
    return 'SAVED'

  def close(self):
    if self.pool is not None:
      self.pool.shutdown(wait=False)
      self.pool = None
