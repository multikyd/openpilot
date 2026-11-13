"""Small live-only status banner; no reads on the 60 Hz drawing path."""
import json
import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.kisapilot.bookmark_status import status_text
from openpilot.system.ui.lib.text_measure import measure_text_cached


class BookmarkTransferBanner:
  def __init__(self):
    self.params = Params('/dev/shm/params')
    self.next_read = 0.
    self.report = {}

  def draw(self, rect, font):
    now = time.monotonic()
    if now >= self.next_read:
      self.next_read = now+.5
      try:
        self.report = json.loads(self.params.get('KisaBookmarkTransferStatus') or '{}')
      except (ValueError, TypeError, OSError, RuntimeError):
        self.report = {}
    text, state = status_text(self.report, now)
    if not text:
      return
    size = 15
    measured = measure_text_cached(font, text, size)
    size *= min(1., 250/max(1., measured.x))
    measured = measure_text_cached(font, text, size)
    center = rect.x+rect.width/2
    # Below the existing camera-offset badge; clear of DM, speed and torque bar.
    box = rl.Rectangle(center-measured.x/2-8, rect.y+49, measured.x+16, 25)
    color = rl.Color(80, 255, 130, 255) if state == 'saved' else rl.Color(255, 190, 70, 255) if state == 'error' else rl.WHITE
    rl.draw_rectangle_rounded(box, .2, 8, rl.Color(0, 0, 0, 190))
    rl.draw_text_ex(font, text, rl.Vector2(center-measured.x/2, box.y+(box.height-measured.y)/2), size, 0, color)
