"""Two-step brake-lamp labelling in the right strip, with no control side effects."""
import time
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.mici.onroad.hud_layout import SPEED_HEADER_HEIGHT
from openpilot.selfdrive.ui.onroad.brake_labels import BrakeLabelWriter
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached


class BrakeLabelControls:
  def __init__(self):
    self.writer = BrakeLabelWriter()
    self.buttons = []
    self.opened = None
    self.message = ''
    self.until = 0.

  def tap(self, pos, detector):
    now = time.monotonic()
    for rect, action in self.buttons:
      if rect.x <= pos.x <= rect.x+rect.width and rect.y <= pos.y <= rect.y+rect.height:
        if action == 'LABEL':
          self.opened = (detector.epoch, now)
        elif action == 'CANCEL':
          self.opened = None
        else:
          sample = detector.label_sample(now)
          same_target = self.opened and self.opened[0] == detector.epoch and now-self.opened[1] < 5.
          self.message = self.writer.submit(sample if same_target else None, action, now, time.time())
          self.until = now+2.
          self.opened = None
        return True
    return False

  def render(self, rect, detector):
    self.buttons = []
    now = time.monotonic()
    result = self.writer.result()
    if result:
      self.message, self.until = result, now+2.
    if not detector.enabled:
      self.opened = None
      return
    if self.opened and (now-self.opened[1] >= 5. or self.opened[0] != detector.epoch):
      self.opened = None
    font = gui_app.font(FontWeight.BOLD)
    x, y, width = rect.x+rect.width-SIDE_PANEL_WIDTH+3, rect.y+SPEED_HEADER_HEIGHT, SIDE_PANEL_WIDTH-6
    def button(text, by, height, action, color):
      box = rl.Rectangle(x, by, width, height)
      if action:
        self.buttons.append((box, action))
      rl.draw_rectangle_rounded(box, .15, 4, color)
      size = 16.
      measured = measure_text_cached(font, text, size)
      size *= min(1., (width-6)/max(1., measured.x))
      measured = measure_text_cached(font, text, size)
      rl.draw_text_ex(font, text, rl.Vector2(x+(width-measured.x)/2, by+(height-measured.y)/2), size, 0, rl.WHITE)
    if self.opened:
      rl.draw_rectangle_rec(rl.Rectangle(x-3,y,SIDE_PANEL_WIDTH,rect.height-SPEED_HEADER_HEIGHT), rl.BLACK)
      for index, (label, color) in enumerate((('ON', rl.Color(170,80,15,255)),
                                             ('OFF', rl.Color(25,110,140,255)),
                                             ('CANCEL', rl.Color(50,55,65,255)))):
        button(label, y+6+index*52, 44, label, color)
    else:
      # The icon itself remains visible; its large 74x68 touch area has a small title.
      self.buttons.append((rl.Rectangle(x,y,width,68), 'LABEL'))
      button(self.message if now < self.until else 'LABEL', y, 20, None, rl.Color(40,48,58,255))

  def close(self):
    self.writer.close()
