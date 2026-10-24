"""C4 mici's former confidence-ball space, showing observation-only lead status."""
import time
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.mici.onroad.lead_brake_state import lead_brake_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, FONT_SCALE, font_fallback
from openpilot.selfdrive.ui.mici.onroad.hud_layout import HEADER_TOP, SPEED_HEADER_HEIGHT
from openpilot.system.ui.lib.text_measure import measure_text_cached


class LeadBrakePanel:
  def render(self, rect, sm, detector):
    if not detector.enabled:
      return
    now = time.monotonic()
    fresh = (sm.valid['radarState'] and sm.alive['radarState'] and
             0 <= now-sm.recv_time['radarState'] <= .35)
    state = lead_brake_state(sm['radarState'].leadOne, fresh, detector.latest_observation(now))
    if state is None:
      return
    sidebar = rl.Rectangle(rect.x+rect.width-SIDE_PANEL_WIDTH, rect.y+SPEED_HEADER_HEIGHT,
                           SIDE_PANEL_WIDTH, max(0, rect.height-SPEED_HEADER_HEIGHT))
    self.draw(sidebar, state)

  @staticmethod
  def draw_speed(rect, speed, metric, color):
    font = gui_app.font(FontWeight.BOLD)
    value = str(round(max(0., speed)))
    available = max(1., rect.width-6)
    size = 54.
    measured = measure_text_cached(font, value, size)
    size *= min(1., available/max(1., measured.x))
    effective = font_fallback(font)
    top_offset = 0.
    # Raylib positions text by its line box. Compensate glyph bearing so the
    # visible digit tops align with the signal/offset box, including 3 digits.
    if getattr(effective, 'baseSize', 0) > 0:
      glyphs = [rl.get_glyph_info(effective, ord(ch)) for ch in value]
      top = min(g.offsetY for g in glyphs)
      bottom = max(g.offsetY+g.image.height for g in glyphs)
      ink_height = (bottom-top)*size*FONT_SCALE/effective.baseSize
      size *= min(1., 48/max(1., ink_height))
      top_offset = top*size*FONT_SCALE/effective.baseSize
    measured = measure_text_cached(font, value, size)
    rl.draw_text_ex(font, value, rl.Vector2(rect.x+(rect.width-measured.x)/2,
                                         rect.y+HEADER_TOP-top_offset), size, 0, color)
    unit = 'km/h' if metric else 'mph'
    size = 12.
    measured = measure_text_cached(font, unit, size)
    rl.draw_text_ex(font, unit, rl.Vector2(rect.x+(rect.width-measured.x)/2, rect.y+55),
                    size, 0, rl.Color(220, 226, 233, 255))

  @staticmethod
  def draw(rect, state):
    # Wider strip gives larger speed digits, lamps and brake-state text.
    scale = min(1., rect.width/80., rect.height/170.)
    if scale <= 0:
      return
    cx = rect.x+rect.width/2
    top = rect.y+(rect.height-170*scale)*.35
    width = rect.width-6*scale
    font = gui_app.font(FontWeight.MEDIUM)
    muted = rl.Color(155, 165, 177, 255)
    color = {"ON": rl.Color(255, 174, 54, 255), "OFF": rl.Color(112, 206, 225, 255),
             "UNKNOWN": muted}[state.state]
    outline = rl.Color(100, 111, 124, 255)
    white = rl.Color(232, 237, 242, 255)

    def text(value, y, size, ink):
      size *= scale
      measure = measure_text_cached(font, value, size)
      size *= min(1., width/max(1., measure.x))
      measure = measure_text_cached(font, value, size)
      rl.draw_text_ex(font, value, rl.Vector2(cx-measure.x/2, top+y*scale), size, 0, ink)

    rl.begin_scissor_mode(int(rect.x), int(rect.y), int(rect.width), int(rect.height))
    try:
      text("LEAD", 0, 15, muted)
      w, h = 55*scale, 44*scale
      y = top+23*scale
      def point(x, yy):
        return rl.Vector2(cx+x*w, y+yy*h)
      # Rear-car silhouette, with recognizable left/right and high-center lamps.
      points = [(-.47, 1.), (-.47, .42), (-.34, .08), (-.24, 0.), (.24, 0.),
                (.34, .08), (.47, .42), (.47, 1.), (.30, 1.), (.30, .86),
                (-.30, .86), (-.30, 1.), (-.47, 1.)]
      for a, b in zip(points, points[1:]):
        rl.draw_line_ex(point(*a), point(*b), 1.6*scale, outline)
      rl.draw_line_ex(point(-.37, .39), point(.37, .39), scale, outline)
      lamps = [(-.43, .50, .22, .14), (.21, .50, .22, .14), (-.14, .26, .28, .06)]
      for x, yy, ww, hh in lamps:
        p = point(x, yy)
        if state.state == "ON":
          rl.draw_rectangle_rounded(rl.Rectangle(p.x-2*scale, p.y-2*scale, ww*w+4*scale, hh*h+4*scale),
                                    .4, 4, rl.Color(255, 174, 54, 38))
        rl.draw_rectangle_rounded(rl.Rectangle(p.x, p.y, ww*w, max(1., hh*h)), .35, 4, color)
      text("BRAKE", 71, 16, color)
      text(state.status_text, 91, 16, color)
      text(f"{state.distance:.1f} m", 115, 18, white)
      arrow_y = top+143*scale
      arrow_color = color if state.motion else muted
      if state.motion:
        direction = 1 if state.motion < 0 else -1
        for sign in (-1, 1):
          rl.draw_line_ex(rl.Vector2(cx, arrow_y-3*scale*direction),
                          rl.Vector2(cx+sign*6*scale, arrow_y+3*scale*direction), 1.8*scale, arrow_color)
      else:
        rl.draw_line_ex(rl.Vector2(cx-5*scale, arrow_y), rl.Vector2(cx+5*scale, arrow_y), scale, muted)
      text(state.motion_text, 150, 14, muted)
    finally:
      rl.end_scissor_mode()
