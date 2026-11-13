"""Large frozen-frame human feedback; no changes to driving control."""
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
    self.sample = None
    self.texture = None
    self.texture_attempted = False
    self.label = self.feedback = None
    self.message = ''
    self.until = 0.

  def dismiss(self):
    self.opened = self.sample = None
    self.label = self.feedback = None
    self.texture_attempted = False
    if self.texture is not None:
      rl.unload_texture(self.texture)
      self.texture = None

  def tap(self, pos, detector):
    now = time.monotonic()
    for rect, action in self.buttons:
      if rect.x <= pos.x <= rect.x+rect.width and rect.y <= pos.y <= rect.y+rect.height:
        if action == 'LABEL':
          sample = detector.label_sample(now)
          if sample is None:
            self.message, self.until = 'NO FRAME', now+2.
          else:
            self.dismiss()
            self.sample = dict(sample, rgb=sample['rgb'].copy(), review_started_mono=now)
            self.opened = now
        elif action == 'CANCEL':
          self.dismiss()
        elif self.opened is not None and now-self.opened <= 10.:
          if action in ('ON','OFF'):
            self.label = action
          elif action in ('correct','incorrect','missing'):
            self.feedback = action
          elif action == 'SAVE':
            if self.label is None or self.feedback is None:
              self.message, self.until = 'SELECT BOTH', now+2.
            else:
              self.message = self.writer.submit(self.sample,self.label,now,time.time(),box_feedback=self.feedback)
              self.until = now+2.
              self.dismiss()
        return True
    # Modal touches never leak through to camera or other controls.
    return self.opened is not None

  def render(self, rect, detector):
    self.buttons = []
    now = time.monotonic()
    result = self.writer.result()
    if result:
      self.message, self.until = result, now+2.
    if not detector.enabled:
      self.dismiss()
      return
    if self.opened is not None and now-self.opened > 10.:
      self.dismiss()
    font = gui_app.font(FontWeight.BOLD)
    def text(value,x,y,w,size,color=rl.WHITE):
      measured=measure_text_cached(font,value,size)
      size*=min(1.,max(1.,w-6)/max(1.,measured.x))
      measured=measure_text_cached(font,value,size)
      rl.draw_text_ex(font,value,rl.Vector2(x+(w-measured.x)/2,y),size,0,color)
    if self.opened is None:
      x=rect.x+rect.width-SIDE_PANEL_WIDTH-10
      self.buttons.append((rl.Rectangle(x,rect.y+SPEED_HEADER_HEIGHT,SIDE_PANEL_WIDTH+10,88),'LABEL'))
      if now < self.until:
        text(self.message,x+10,rect.y+SPEED_HEADER_HEIGHT,SIDE_PANEL_WIDTH,14)
      return
    # Scale the 536x240 modal as a whole, preserving large touch targets.
    scale=min(rect.width/536.,rect.height/240.)
    ox=rect.x+(rect.width-536*scale)/2;oy=rect.y+(rect.height-240*scale)/2
    def box(x,y,w,h):return rl.Rectangle(ox+x*scale,oy+y*scale,w*scale,h*scale)
    rl.draw_rectangle_rec(rect,rl.Color(9,13,18,255))
    text('BRAKE / BOX FEEDBACK',ox,oy+4*scale,536*scale,22*scale)
    if not self.texture_attempted:
      self.texture_attempted=True
      try:
        rgb=self.sample['rgb']
        image=rl.Image(rl.ffi.from_buffer(rgb),rgb.shape[1],rgb.shape[0],1,
                       rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8)
        self.texture=rl.load_texture_from_image(image)
        if self.texture.id <= 0:self.texture=None
      except Exception:
        self.texture=None
    if self.texture is not None:
      w,h=self.sample['rgb'].shape[1],self.sample['rgb'].shape[0]
      zoom=min(212./w,140./h)
      dest=box(12+(212-w*zoom)/2,38+(140-h*zoom)/2,w*zoom,h*zoom)
      rl.draw_texture_pro(self.texture,rl.Rectangle(0,0,w,h),dest,rl.Vector2(0,0),0,rl.WHITE)
      for x,y,bw,bh in self.sample.get('lamps',()):
        rl.draw_rectangle_lines_ex(rl.Rectangle(dest.x+x*zoom*scale,dest.y+y*zoom*scale,
                                               bw*zoom*scale,bh*zoom*scale),2,rl.Color(255,190,65,255))
    else:
      text('PREVIEW UNAVAILABLE',ox+12*scale,oy+90*scale,212*scale,14*scale)
    text('FROZEN FRAME',ox+12*scale,oy+185*scale,212*scale,14*scale)
    status=self.message if now<self.until else 'SELECT BRAKE + BOX'
    text(status,ox+12*scale,oy+207*scale,212*scale,13*scale)
    missing=not self.sample.get('lamps')
    actions=[('ON','ON',238,38),('OFF','OFF',384,38),
             ('BOX OK','correct',238,104),('NO BOX' if missing else 'WRONG','missing' if missing else 'incorrect',384,104),
             ('SAVE','SAVE',238,178),('CANCEL','CANCEL',384,178)]
    for caption,action,x,y in actions:
      r=box(x,y,140,54 if y<178 else 48)
      selected=action in (self.label,self.feedback)
      color=rl.Color(35,125,155,255) if selected else rl.Color(45,53,66,255)
      if action=='SAVE' and self.label is not None and self.feedback is not None:color=rl.Color(35,125,75,255)
      rl.draw_rectangle_rounded(r,.15,4,color)
      text(caption,r.x,r.y+10*scale,r.width,(28 if y==38 else 21)*scale)
      self.buttons.append((r,action))

  def close(self):
    self.dismiss()
    self.writer.close()
