"""Horizontal C4 signal bar; large timer and explicit estimated-time styling."""
import math
import pyray as rl
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.mici.onroad.hud_layout import signal_box
from openpilot.selfdrive.ui.mici.onroad.signal_presentation import signal_lamps, countdown_rows, signal_panel_visible


def draw_signal_panel(rect, font, display):
  if rect.width < 440 or not signal_panel_visible(display):
    return
  x, y, width, height = signal_box(rect.x, rect.y, rect.width)
  # Reuse the established 232x74 drawing in a smaller top-aligned viewport.
  scale = width/232
  width, height = 232, 74
  def rectangle(px, py, w, h):
    return rl.Rectangle(x+(px-x)*scale, y+(py-y)*scale, w*scale, h*scale)
  amber = rl.Color(255,205,80,255)
  red, green = rl.Color(255,82,88,255), rl.Color(100,240,78,255)
  rl.draw_rectangle_rounded(rectangle(x,y,width,height), .23, 10, rl.Color(43,47,55,245))
  rl.draw_rectangle_rounded_lines_ex(rectangle(x,y,width,height), .23, 10, 2, rl.Color(135,143,153,220))

  scale_view = scale

  def text(value, left, top, available, size, color, max_height=None):
    measured=measure_text_cached(font,value,size)
    scale=min(1.,available/max(1.,measured.x))
    if max_height is not None:
      scale=min(scale,max_height/max(1.,measured.y))
    size*=scale
    measured=measure_text_cached(font,value,size)
    ty=top+(max_height-measured.y)/2 if max_height is not None else top
    rl.draw_text_ex(font,value,rl.Vector2(x+(left+(available-measured.x)/2-x)*scale_view,
                                        y+(ty-y)*scale_view),size*scale_view,0,color)

  source=display.source
  live=source in ('TMAP','VISION')
  arrow_right=display.movement=='right'
  def state_color(state):
    return red if state==1 else green if state==2 else amber if state==3 else rl.WHITE

  lamps=signal_lamps(display)
  for i,state in enumerate(lamps):
    cx,cy=x+21+i*36,y+height/2
    background=state_color(state) if state and i!=1 else rl.Color(28,33,41,255)
    rl.draw_circle(int(x+(cx-x)*scale),int(y+(cy-y)*scale),16*scale,background)
    if i==1:
      # A turn stop state leaves the arrow dark, matching the physical signal.
      ink=state_color(state) if state else rl.Color(65,74,85,255)
      for a,b in (((9,0),(-9,0)),((-9,0),(-1,-8)),((-9,0),(-1,8))):
        flip=-1 if arrow_right else 1
        rl.draw_line_ex(rl.Vector2(x+(cx+flip*a[0]-x)*scale,y+(cy+a[1]-y)*scale),
                        rl.Vector2(x+(cx+flip*b[0]-x)*scale,y+(cy+b[1]-y)*scale),4*scale,ink)

  direction={'straight':'STRAIGHT','left':'LEFT','right':'RIGHT','observed':'OBS','other':'DIR ?'}.get(display.movement,'DIR ?')
  mode=display.operatingMode
  rows=countdown_rows(display)
  state=rows[0][1]
  color=state_color(state) if live else amber
  value=rows[0][2]
  location=getattr(display,'locationKind','')
  distance='NO RANGE' if location=='unknown' else f'{"QUEUE" if location=="queue" else "PT"} ~{display.distance:.0f}m' if location in ('stop','queue') else f'~{display.distance:.0f}m' if location=='candidate' else f'{display.distance:.0f}m'
  detail=f'{source} / {direction} / {distance}'
  if mode in ('OFF','FLASH'):
    value=mode;color=amber
  elif source=='EST':
    value=f'~{round((display.estimateLow+display.estimateHigh)/2)}'
    detail=f'EST {"WAIT" if mode=="WAIT" else "RED" if display.predictionState==1 else "GREEN"} / {direction} / {distance}'
    text(f'{math.floor(display.estimateLow)}-{math.ceil(display.estimateHigh)}s',x,y+height+23,width,22,amber)
  if live and mode not in ('OFF','FLASH') and rows[0][0]:
    row_height=(height-10)/len(rows)
    for i,(label,state,value) in enumerate(rows):
      text(f'{label} {value}',x+113,y+5+i*row_height,width-121,
           min(48,row_height),state_color(state),max_height=row_height)
  else:
    text(value,x+113,y+5,width-121,64,color,max_height=height-10)
  text(detail,x+10,y+height+4,width-20,20,amber if source=='EST' else rl.WHITE)
