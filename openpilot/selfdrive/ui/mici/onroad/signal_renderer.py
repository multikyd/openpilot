"""Horizontal C4 signal bar; large timer and explicit estimated-time styling."""
import math
import pyray as rl
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.mici.onroad.signal_presentation import signal_lamps, countdown_rows


def draw_signal_panel(rect, font, display):
  if not display.source or rect.width < 440:
    return
  width, height = 232, 74
  x, y = rect.x+rect.width-width-8, rect.y+92
  amber = rl.Color(255,205,80,255)
  red, green = rl.Color(255,82,88,255), rl.Color(100,240,78,255)
  rl.draw_rectangle_rounded(rl.Rectangle(x,y,width,height), .23, 10, rl.Color(43,47,55,245))
  rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(x,y,width,height), .23, 10, 2, rl.Color(135,143,153,220))

  def text(value, left, top, available, size, color, max_height=None):
    measured=measure_text_cached(font,value,size)
    scale=min(1.,available/max(1.,measured.x))
    if max_height is not None:
      scale=min(scale,max_height/max(1.,measured.y))
    size*=scale
    measured=measure_text_cached(font,value,size)
    ty=top+(max_height-measured.y)/2 if max_height is not None else top
    rl.draw_text_ex(font,value,rl.Vector2(left+(available-measured.x)/2,ty),size,0,color)

  source=display.source
  live=source in ('TMAP','VISION')
  arrow_right=display.movement=='right'
  def state_color(state):
    return red if state==1 else green if state==2 else amber if state==3 else rl.WHITE

  lamps=signal_lamps(display)
  for i,state in enumerate(lamps):
    cx,cy=x+21+i*36,y+height/2
    background=state_color(state) if state and i!=1 else rl.Color(28,33,41,255)
    rl.draw_circle(int(cx),int(cy),16,background)
    if i==1:
      # A turn stop state leaves the arrow dark, matching the physical signal.
      ink=state_color(state) if state else rl.Color(65,74,85,255)
      for a,b in (((9,0),(-9,0)),((-9,0),(-1,-8)),((-9,0),(-1,8))):
        flip=-1 if arrow_right else 1
        rl.draw_line_ex(rl.Vector2(cx+flip*a[0],cy+a[1]),rl.Vector2(cx+flip*b[0],cy+b[1]),4,ink)

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
  elif source=='LEARN':
    detail+=f' / {display.samples} OBS'
  elif source=='MODEL':
    detail=f'MODEL {"WAIT" if mode=="MODEL_STOP" else "MOVE"} / {direction} / {distance}'
  if live and mode not in ('OFF','FLASH') and rows[0][0]:
    row_height=(height-10)/len(rows)
    for i,(label,state,value) in enumerate(rows):
      text(f'{label} {value}',x+113,y+5+i*row_height,width-121,
           min(48,row_height),state_color(state),max_height=row_height)
  else:
    text(value,x+113,y+5,width-121,64,color,max_height=height-10)
  text(detail,x,y+height+4,width,13,amber if source=='EST' else rl.WHITE)
