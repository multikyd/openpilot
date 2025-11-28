import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

# kisa
from openpilot.selfdrive.ui.onroad.kisa_button import KisaButton
import math
from collections import deque

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'


@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)  # Added
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK
  # kisa
  green_translucent: rl.Color = rl.Color(0, 200, 0, 100)
  blue_translucent: rl.Color = rl.Color(0, 140, 255, 120)
  ochre_translucent: rl.Color = rl.Color(204, 153, 0, 128)
  orange_translucent: rl.Color = rl.Color(204, 120, 0, 128)


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)

    self._kisa_button: KisaButton = KisaButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)
    self.img_width = 200
    self.img_car_width, self.img_car_height = 120, 200
    self.img_speed_bump = gui_app.texture("addon/img/img_speed_bump.png", self.img_width, self.img_width)
    self.img_car = gui_app.texture("addon/img/car.png", self.img_car_width, self.img_car_height)

    self._draw_plot = DrawPlot()

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = v_cruise_cluster
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    if self.is_cruise_set and not ui_state.is_metric:
      self.set_speed *= KM_TO_MILE

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_current_speed(rect)
    self._draw_blinkers(rect)
    self._draw_speed_limit_sign(rect)
    self._draw_standstill_timer(rect)
    self._draw_car_stat(rect)
    self._draw_debug_msg(rect)
    
    self._draw_plot.draw(self)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    self._kisa_button.render(rl.Rectangle(button_x, button_y + 960 - UI_CONFIG.button_size, UI_CONFIG.button_size, UI_CONFIG.button_size))

  def user_interacting(self) -> bool:
    return self._exp_button.is_pressed or self._kisa_button.is_pressed

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    s = ui_state

    set_speed_width = UI_CONFIG.set_speed_width_metric if s.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y - 45 + 1020 - UI_CONFIG.set_speed_height - 190

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height + 20) # 2

    if s.xSpdLimit > 19 and self.speed > s.desiredSpeed and s.xSpdType != 22:
      bg_brush = COLORS.ochre_translucent
    elif s.xSpdLimit > 19:
      bg_brush = COLORS.green_translucent
    elif s.enabled:
      bg_brush = COLORS.blue_translucent
    else:
      bg_brush = COLORS.BLACK_TRANSLUCENT

    # Draw rounded rect background + border
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 32, bg_brush)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 32, 6, COLORS.WHITE_TRANSLUCENT)

    # mid line
    line_y = y + UI_CONFIG.set_speed_height // 2 - 7
    start = rl.Vector2(x + 35, line_y)
    end = rl.Vector2(x + set_speed_width - 35, line_y)
    try:
      rl.draw_line(start, end, 6)
    except Exception:
      rl.draw_rectangle_rounded(rl.Rectangle(start.x, start.y - 3, end.x - start.x, 6), 0.1, 3, COLORS.WHITE)

    setSpeedStr = str(round(self.set_speed)) if 0 < self.set_speed < 254 and s.enabled else CRUISE_DISABLED_CHAR
    # Draw top text
    top_font_size = 80
    setSpeedStr_w = measure_text_cached(self._font_semi_bold, setSpeedStr, top_font_size).x
    rl.draw_text_ex(self._font_semi_bold, setSpeedStr, rl.Vector2(x + (set_speed_width - setSpeedStr_w) / 2, y), top_font_size, 0, COLORS.WHITE)

    # Draw bottom text
    if not (s.has_longitudinal_control or s.camera_scc > 0):
      bottom_text = str(int(s.vSetDis)) if s.enabled else CRUISE_DISABLED_CHAR
    else:
      bottom_text = str(int(min(s.desiredSpeed, self.set_speed))) if s.enabled else CRUISE_DISABLED_CHAR
    bottom_font_size = FONT_SIZES.set_speed + 3
    bottom_text_w = measure_text_cached(self._font_bold, bottom_text, bottom_font_size).x
    rl.draw_text_ex(self._font_bold, bottom_text, rl.Vector2(x + (set_speed_width - bottom_text_w) / 2, y + 90), bottom_font_size, 0, COLORS.WHITE)

    if s.desiredSpeed > self.set_speed:
      source_text = ""
    else:
      source_text = s.desiredSource
    source_font_size = FONT_SIZES.set_speed - 47
    source_text_w = measure_text_cached(self._font_semi_bold, source_text, source_font_size).x
    rl.draw_text_ex(self._font_semi_bold, source_text, rl.Vector2(x + (set_speed_width - source_text_w) / 2, y + 172), source_font_size, 0, rl.Color(230, 200, 0, 255))

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    s = ui_state

    # speed text
    speed_text = str(round(self.speed))
    act_accel = s.aReqValue if not (s.has_longitudinal_control or s.camera_scc > 0) else s.accel

    def clamp(v, lo, hi):
      return lo if v < lo else (hi if v > hi else v)

    gas_opacity = clamp(act_accel * 255, 0, 255)
    brake_opacity = clamp(abs(act_accel * 175), 0, 255)

    if s.brakePress:
      speed_color = rl.Color(255, 0, 0, 255)
    elif s.brakeLights and speed_text == "0":
      speed_color = rl.Color(201, 34, 49, 100)
    elif s.gasPress:
      speed_color = rl.Color(0, 240, 0, 255)
    elif (act_accel < 0 and act_accel > -5.0):
      r = clamp(255 - int(abs(act_accel * 8)), 0, 255)
      g = clamp(255 - int(brake_opacity), 0, 255)
      b = clamp(255 - int(brake_opacity), 0, 255)
      speed_color = rl.Color(r, g, b, 255)
    elif (act_accel > 0 and act_accel < 3.0):
      r = clamp(255 - int(gas_opacity), 0, 255)
      g = clamp(255 - int(act_accel * 10), 0, 255)
      b = clamp(255 - int(gas_opacity), 0, 255)
      speed_color = rl.Color(r, g, b, 255)
    else:
      speed_color = COLORS.WHITE

    set_speed_width = UI_CONFIG.set_speed_width_metric if s.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 50 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 1020 - 225
    speed_pos = rl.Vector2(x, y)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed + 10, 0, speed_color)

    # unit_text = "KPH" if ui_state.is_metric else "MPH"
    # unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    # unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    # rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)

  def _draw_blinkers(self, rect: rl.Rectangle) -> None:
    """Draw KisaPilot-style blinkers."""
    if not ui_state.leftBlinker and not ui_state.rightBlinker:
      return

    t = rl.get_time()
    center_x = rect.x + rect.width // 2
    center_y = rect.y + 200
    size, thickness, freq, sway_amp = 100, 50, 3, 20
    sway = sway_amp * math.sin(t * freq)
    count, spacing = 3, 110
    base_color = rl.Color(230, 165, 0, 230)
    overlap = 0.25

    img = self.img_car
    x_center = rect.x + UI_CONFIG.border_size + 57 + img.width // 2
    y_center = rect.y + 450
    blinker_width = 20
    blinker_height = 10
    blinker_spacing = 45
    alpha = int(((math.sin(t * freq) + 1) / 2) * 230)

    def draw_chevron(x, y, direction="right", alpha=230):
      delta = size * overlap
      if direction == "right":
        points = [(x - size + delta, y - size + delta), (x, y), (x - size + delta, y + size - delta)]
      else:
        points = [(x + size - delta, y - size + delta), (x, y), (x + size - delta, y + size - delta)]
      color = rl.Color(base_color.r, base_color.g, base_color.b, alpha)
      for i in range(2):
        rl.draw_line_ex(points[i], points[i+1], thickness, color)
        rl.draw_circle(int(points[i][0]), int(points[i][1]), thickness/2, color)
        rl.draw_circle(int(points[i+1][0]), int(points[i+1][1]), thickness/2, color)

    def draw_sequence(x, y, direction="right"):
      for i in range(count):
        offset = i * spacing
        alpha = int(((math.sin(t * freq - (count - 1 - i) * 0.5) + 1) / 2) * base_color.a)
        pos_x = x - offset if direction == "right" else x + offset
        draw_chevron(pos_x, y, direction, alpha)

    if ui_state.leftBlinker:
      blinker_left = rl.Rectangle(x_center - blinker_spacing - blinker_width + 30 + 14, y_center - img.height // 2 + 8, blinker_width, blinker_height)
      rl.draw_rectangle_pro(blinker_left, rl.Vector2(blinker_left.width / 2, blinker_left.height / 2), -23, rl.Color(230, 165, 0, alpha))
      draw_sequence(center_x - 700 - sway, center_y, "left")
    if ui_state.rightBlinker:
      blinker_right = rl.Rectangle(x_center + blinker_spacing - 2 + 5, y_center - img.height // 2 + 8, blinker_width, blinker_height)
      rl.draw_rectangle_pro(blinker_right, rl.Vector2(blinker_right.width / 2, blinker_right.height / 2), 23, rl.Color(230, 165, 0, alpha))
      draw_sequence(center_x + 700 + sway, center_y, "right")

  def _draw_speed_limit_sign(self, rect: rl.Rectangle) -> None:
    """Draw KisaPilot-style speed limit sign."""
    s_center_x = rect.x + UI_CONFIG.border_size + 340
    s_center_y = rect.y + 1020 - 333
    d_center_y = s_center_y - 160

    diameters = (220, 180, 202)
    rects = {
      "inner": rl.Rectangle(s_center_x - diameters[1]//2, s_center_y - diameters[1]//2, diameters[1], diameters[1]),
      "dist":  rl.Rectangle(s_center_x - 110, d_center_y - 35, 220, 70),
    }

    type, limit_spd, dist = ui_state.xSpdType, ui_state.xSpdLimitOrg, ui_state.xSpdDist

    if limit_spd <= 20 and dist == 0:
      return

    visual_offset = 1.2

    if type == 22:
      img = self.img_speed_bump
      icon_x = s_center_x - self.img_width // 2
      icon_y = s_center_y - self.img_width // 2
      icon_rect = rl.Rectangle(icon_x, icon_y, self.img_width, self.img_width)
      source_rect = rl.Rectangle(0, 0, img.width, img.height)
      rl.draw_texture_pro(img, source_rect, icon_rect, rl.Vector2(0, 0), 0, rl.WHITE)
    else:
      cx, cy = int(rects["inner"].x + rects["inner"].width / 2), int(rects["inner"].y + rects["inner"].height / 2)
      rl.draw_circle(cx, cy, int(diameters[0] / 2), rl.RED)
      rl.draw_circle(cx, cy, int(diameters[1] / 2), rl.WHITE)
      text = str(int(limit_spd))
      font_size = 110 if limit_spd < 100 else 90
      text_size = rl.measure_text_ex(self._font_bold, text, font_size, 0)
      text_x = rects["inner"].x + (rects["inner"].width - text_size.x*visual_offset) / 2
      text_y = rects["inner"].y + (rects["inner"].height - text_size.y*visual_offset) / 2
      rl.draw_text_ex(self._font_bold, text, rl.Vector2(text_x, text_y), font_size, 0, rl.BLACK)

    if dist == 0:
      return

    opacity = max(0, min(255, int((600 - dist) * 0.425))) if dist <= 600 else 0
    rl.draw_rectangle_rounded(rects["dist"], 0.35, 32, rl.Color(255, 0, 0, opacity))
    rl.draw_rectangle_rounded_lines_ex(rects["dist"], 0.35, 32, 6, COLORS.WHITE_TRANSLUCENT)

    dist_text = (
      f"{dist:.0f}m" if dist < 1000 else
      f"{dist/1000:.2f}km" if dist < 10000 else
      f"{dist/1000:.1f}km"
    )

    font_size = 55
    text_size = rl.measure_text_ex(self._font_bold, dist_text, font_size, 0)
    text_x = rects["dist"].x + (rects["dist"].width - text_size.x*visual_offset) / 2
    text_y = rects["dist"].y + (rects["dist"].height - text_size.y*visual_offset) / 2
    rl.draw_text_ex(self._font_bold, dist_text, rl.Vector2(text_x, text_y), font_size, 0, rl.WHITE)

  def _draw_standstill_timer(self, rect: rl.Rectangle) -> None:
    """Draw KisaPilot-style standstill timer."""
    if ui_state.standstillElapsedTimer:
      minute = int(ui_state.standstillElapsedTimer // 60)
      second = int(ui_state.standstillElapsedTimer % 60)
      time_text = f"{minute:02d}:{second:02d}"

      stop_x = rect.x + rect.width - UI_CONFIG.border_size - 745
      stop_y = rect.y + UI_CONFIG.border_size + 320

      time_x = stop_x
      time_y = rect.y + UI_CONFIG.border_size + 450

      stop_color = rl.Color(204, 119, 34, 220)
      time_color = rl.Color(255, 255, 255, 220)

      rl.draw_text_ex(self._font_bold, "STOP", rl.Vector2(stop_x, stop_y),
                      135, 0, stop_color)

      rl.draw_text_ex(self._font_bold, time_text, rl.Vector2(time_x, time_y),
                      140, 0, time_color)

  def _draw_car_stat(self, rect: rl.Rectangle) -> None:
    """Draw KisaPilot-style CAR Status."""
    s = ui_state

    img = self.img_car
    x_center = rect.x + UI_CONFIG.border_size + 56 + img.width // 2
    y_center = rect.y + 460
    icon_x = x_center - img.width // 2
    icon_y = y_center - img.height // 2
    icon_rect = rl.Rectangle(icon_x, icon_y, self.img_car_width, self.img_car_height)
    source_rect = rl.Rectangle(0, 0, img.width, img.height)
    rl.draw_texture_pro(img, source_rect, icon_rect, rl.Vector2(0, 0), 0, rl.Color(255, 255, 255, 150))

    fl = s.tpmsPressureFl
    fr = s.tpmsPressureFr
    rl_p = s.tpmsPressureRl
    rr = s.tpmsPressureRr
    unit = s.tpmsUnit  # 0: psi, 1: kpa, 2: bar

    font_size = 36 if unit == 2 else (32 if unit != 0 else 37)

    def fmt_val(v):
      if v is None or v == 0 or v == 255:
        return ""
      return f"{int(round(v))}" if unit != 2 else f"{v:.1f}"

    def draw_value(offset_x, offset_y, value):
      offset = 0.52 if unit == 2 else (0.51 if unit != 0 else 0.465)
      if value is None:
        col = rl.Color(255, 255, 255, 210)
        text = ""
      else:
        if (value < 32 and unit != 2) or (value < 2.2 and unit == 2):
          col = rl.Color(255, 200, 0, 210)  # yellow
        elif (value > 45 and unit != 2) or (value > 2.8 and unit == 2):
          col = rl.Color(255, 0, 0, 210)    # red
        else:
          col = rl.Color(0, 255, 0, 210)    # green
        text = fmt_val(value)
      tsz = measure_text_cached(self._font_bold, text, font_size).x
      rl.draw_text_ex(self._font_bold, text,
                      rl.Vector2(offset_x - (tsz / 2)*offset, offset_y),
                      font_size, 0, col)

    offset_add = 1.1 if unit == 2 else (0.7 if unit != 0 else 1.2)
    x_offset = img.width // offset_add  # left_right wheel distance
    y_offset_front = -img.height // 3.3  # front
    y_offset_rear = img.height // 4.2    # rear

    # offset based on tire loc
    y_text_adjust = -10

    draw_value(x_center - x_offset, y_center + y_offset_front + y_text_adjust, fl)  # Front-left
    draw_value(x_center + x_offset, y_center + y_offset_front + y_text_adjust, fr)  # Front-right
    draw_value(x_center - x_offset, y_center + y_offset_rear + y_text_adjust, rl_p)  # Rear-left
    draw_value(x_center + x_offset, y_center + y_offset_rear + y_text_adjust, rr)  # Rear-right

    if s.brakeLights:
      brake_width = 20
      brake_height = 10
      brake_spacing = 35

      brake_left = rl.Rectangle(
        x_center - brake_spacing - brake_width + 33,
        y_center + img.height // 2 - 12,
        brake_width,
        brake_height
      )
      rl.draw_rectangle_rounded(brake_left, 0.9, 8, rl.Color(255, 0, 0, 180))

      brake_right = rl.Rectangle(
        x_center + brake_spacing - 6,
        y_center + img.height // 2 - 12,
        brake_width,
        brake_height
      )
      rl.draw_rectangle_rounded(brake_right, 0.9, 8, rl.Color(255, 0, 0, 180))

    if s.autoHold:
      center_text = "AUTO\nHOLD"
      font = self._font_bold
      font_size = 27
      color = rl.Color(0, 255, 0, 230)
      lines = center_text.split("\n")
      line_height = font_size + 3
      total_height = line_height * len(lines)

      for i, line in enumerate(lines):
        tsz = measure_text_cached(font, line, font_size).x
        rl.draw_text_ex(font, line, rl.Vector2(x_center - tsz / 2 + 14, y_center - total_height / 2 + i * line_height), font_size, 0, color)

    if s.cruise_gap:
      gap_count = int(s.cruise_gap)
      gap_w, gap_h, spacing, radius = 50, 12, 6, 0
      color = rl.Color(0, 150, 255, 255)
      border = rl.WHITE
      x = x_center - gap_w // 2 + 15
      base_y = y_center - img.height // 2 - 20

      for i in range(gap_count):
        y = base_y - i * (gap_h + spacing)
        rect = rl.Rectangle(x, y, gap_w, gap_h)
        rl.draw_rectangle_rounded(rect, 0.9, radius, color)
        rl.draw_rectangle_rounded_lines(rect, radius, 1, border)

      lead_dist = s.radarDRel

      if lead_dist is not None and 0 < lead_dist < 150:
        dist_text = f"{lead_dist:.1f} m"

        text_font = self._font_bold
        text_size = 40
        tsz = measure_text_cached(text_font, dist_text, text_size).x

        top_y = base_y - (gap_count * (gap_h + spacing)) - 35

        if lead_dist < 5:
          color = rl.Color(255, 0, 0, 255)
        elif lead_dist < 10:
          color = rl.Color(255, 140, 0, 255)
        else:
          color = rl.Color(255, 255, 255, 230)

        rl.draw_text_ex(text_font, dist_text, rl.Vector2(x_center - tsz / 2 + 14, top_y), text_size, 0, color)

  def _draw_debug_msg(self, rect: rl.Rectangle) -> None:
    if ui_state.debug_msg <= 0:
      return

    s = ui_state

    right_debug_lines = [
      ("CPU", f"{s.cpuUsage:.0f}%"),
      ("MaxTemp", f"{s.maxTemp:.0f}°C"),
      ("Storage", f"{s.storageUsage:.0f}%"),
      ("FAN", f"{s.fanSpeedRpm}"),
      ("Altitude", f"{s.altitude:.0f}m"),
      ("Bearing", f"{s.bearing:.0f}°"),
      ("Voltage", f"{s.voltage:.1f}V"),
    ]

    left_debug_lines = []
    if ui_state.debug_msg > 1:
      left_debug_lines = [
        ("AngOffset", f"{s.angleOffsetDeg:.1f}°"),
        ("SteerRatio", f"{s.steerRatio:.2f}"),
        ("Accel", f"{s.accel:.2f}"),
      ]

    small_font_size = 30
    large_font_size = 40
    line_spacing = 6
    value_spacing = 0
    line_height = small_font_size + large_font_size + line_spacing

    def draw_debug_box(box_x, box_y, debug_lines, box_width=160, bg_alpha=10):
      total_lines = len(debug_lines)
      box_height = total_lines * line_height + 15
      # Draw semi-transparent background
      bg_color = rl.Color(0, 0, 0, bg_alpha)
      rl.draw_rectangle(int(box_x), int(box_y), int(box_width), int(box_height), bg_color)

      # Draw top and bottom lines safely
      line_color = rl.Color(255, 255, 255, 180)
      line_thickness = 5

      # top line
      top_start = rl.Vector2(box_x, box_y)
      top_end = rl.Vector2(box_x + box_width, box_y)
      try:
        rl.draw_line_ex(top_start, top_end, line_thickness, line_color)
      except Exception:
        rl.draw_rectangle_rounded(rl.Rectangle(top_start.x, top_start.y - line_thickness//2, box_width, line_thickness), 0.1, 3, line_color)

      # bottom line
      bottom_start = rl.Vector2(box_x, box_y + box_height)
      bottom_end = rl.Vector2(box_x + box_width, box_y + box_height)
      try:
        rl.draw_line_ex(bottom_start, bottom_end, line_thickness, line_color)
      except Exception:
        rl.draw_rectangle_rounded(rl.Rectangle(bottom_start.x, bottom_start.y - line_thickness//2, box_width, line_thickness), 0.1, 3, line_color)

      # Draw text
      for i, (label, value) in enumerate(debug_lines):
        item_y = box_y + 10 + i * line_height

        # Label
        label_size = rl.measure_text_ex(self._font_bold, label, small_font_size, 0)
        label_x = box_x + (box_width - label_size.x) / 2 - 5
        rl.draw_text_ex(
          self._font_bold,
          label,
          rl.Vector2(label_x, item_y),
          small_font_size,
          0,
          rl.Color(200, 200, 200, 220)
        )

        # Value
        value_size = rl.measure_text_ex(self._font_bold, value, large_font_size, 0)
        value_x = box_x + (box_width - value_size.x) / 2 - 5
        value_y = item_y + small_font_size + value_spacing
        rl.draw_text_ex(
          self._font_bold,
          value,
          rl.Vector2(value_x, value_y),
          large_font_size,
          0,
          rl.Color(255, 255, 255, 220)
        )

    # Draw right box
    right_box_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size + 15
    right_box_y = rect.y + UI_CONFIG.border_size + 210
    draw_debug_box(right_box_x, right_box_y, right_debug_lines)

    if left_debug_lines:
      left_box_x = rect.x + rect.width + UI_CONFIG.border_size - UI_CONFIG.button_size + 15 - 160 - 65
      left_box_y = rect.y + UI_CONFIG.border_size + 210
      draw_debug_box(left_box_x, left_box_y, left_debug_lines)

class DrawPlot:
  PLOT_MAX = 400

  def __init__(self):
    self.plotSize = 0
    self.plotIndex = 0
    self.plotQueue = [[0.0]*self.PLOT_MAX for _ in range(3)]
    self.plotMin = -2.0
    self.plotMax = 2.0
    self.plotX = 350.0
    self.plotY = 70.0
    self.plotHeight = 300.0
    self.plotDx = 2.0
    self.plotRatio = 1.0
    self.show_plot_mode_prev = -1
    self.minDeque = [deque() for _ in range(3)]
    self.maxDeque = [deque() for _ in range(3)]

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)

  def _draw_plotting(self, renderer, index, start, x, y_list, size, color, stroke=2):
    span = (self.plotMax - self.plotMin)
    self.plotRatio = self.plotHeight if span < 1.0 else (self.plotHeight / span)
    dx = self.plotDx

    if size <= 0:
      size = 1

    prev_x = None
    prev_y = None
    for i in range(size):
      data = y_list[(start - i) % self.PLOT_MAX]
      plot_y = self.plotY + self.plotHeight - (data - self.plotMin) * self.plotRatio
      x_pos = x + (size - i) * dx

      if prev_x is not None:
        # pyray wrapper uses rl.draw_line
        try:
          rl.draw_line(int(prev_x), int(prev_y), int(x_pos), int(plot_y), color)
          if stroke > 1:
            rl.draw_line(int(prev_x), int(prev_y)+1, int(x_pos), int(plot_y)+1, color)
        except Exception:
          pass
      else:
        txt = "{:.2f}".format(data)
        y_offset = 40 if index > 0 else 0
        try:
          rl.draw_text_ex(self._font_semi_bold, txt, rl.Vector2(float(x_pos + 50), float(plot_y + y_offset)), 25, 0, rl.WHITE)
        except Exception:
          pass

      prev_x = x_pos
      prev_y = plot_y

  def make_plot_data(self, renderer):
    sm = ui_state.sm
    try:
      car_state = sm["carState"]
      lp = sm["longitudinalPlan"]
      car_control = sm["carControl"]
      controls_state = sm["controlsState"]
      lateral = controls_state.lateralControlState
      if hasattr(lateral, "which"):
        if lateral.which() == "torqueState":
          torque_state = lateral.torqueState
        else:
          torque_state = None
      else:
        torque_state = getattr(lateral, "torqueState", None)
      
      a_ego = car_state.aEgo
      v_ego = car_state.vEgo
      
      accel = lp.accels[0] if len(lp.accels) > 0 else 0.0
      speeds_0 = lp.speeds[0] if len(lp.speeds) > 0 else 0.0
      
      accel_out = car_control.actuators.accel

      model_data = sm["modelV2"]
      model = model_data.modelV2 if hasattr(model_data, "modelV2") else None

      position = model.position if model and len(model.position) > 0 else [0.0]
      velocity = model.velocity if model and len(model.velocity) > 0 else [0.0]

      live_params = sm["liveParameters"]
    except Exception as e:
      print(f"[UI Error] make_plot_data error: {e}")
      return [0.0, 0.0, 0.0], "no data"

    m = ui_state.show_plot_mode
    data = [0.0, 0.0, 0.0]
    title = "no data"

    if m in (0, 1):
      data[0] = a_ego
      data[1] = accel
      data[2] = accel_out
      title = "1.Accel (Y:a_ego, G:a_target, O:a_out)"
    elif m == 2:
      data[0] = speeds_0
      data[1] = v_ego
      data[2] = a_ego
      title = "2.Speed/Accel(Y:speed_0, G:v_ego, O:a_ego)"
    elif m == 3:
      try:
        data[0] = position.x[32]
      except Exception:
        data[0] = 0.0
      try:
        data[1] = velocity.x[32]
        data[2] = velocity.x[0]
      except Exception:
        data[1] = data[2] = 0.0
      title = "3.Model(Y:pos_32, G:vel_32, O:vel_0)"
    elif m == 4:
      data[0] = accel
      if sm.valid['radarState']:
        lead = sm['radarState'].leadOne
        data[1] = lead.aLeadK if lead is not None else 0.0
        data[2] = lead.vRel if lead is not None else 0.0
      title = "4.Lead(Y:accel, G:a_lead, O:v_rel)"
    elif m == 5:
      data[0] = a_ego
      if sm.valid['radarState']:
        lead = sm['radarState'].leadOne
        data[1] = lead.aLead if lead else 0.0
        data[2] = lead.jLead if lead else 0.0
      title = "5.Lead(Y:a_ego, G:a_lead, O:j_lead)"
    elif m == 6 and torque_state is not None:
      data[0] = torque_state.actualLateralAccel * 10.0
      data[1] = torque_state.desiredLateralAccel * 10.0
      data[2] = torque_state.output * 10.0
      title = "6.Steer(Y:actual, G:desire, O:output)"
    elif m == 7:
      data[0] = car_state.steeringAngleDeg
      data[1] = car_control.actuators.steeringAngleDeg
      data[2] = live_params.angleOffsetDeg * 10.0
      title = "7.SteerA (Y:Actual, G:Target, O:Offset*10)"
    elif m == 8:
      try:
        curv = car_control.actuators.curvature * 10000.0
      except Exception:
        curv = 0.0
      data = [curv, curv, curv]
      title = "8.SteerA (Y:Actual, G:Target, O:Offset*10)"
    else:
      data = [0.0, 0.0, 0.0]
      title = "no data"

    if ui_state.show_plot_mode != self.show_plot_mode_prev:
      self.plotSize = 0
      self.plotIndex = 0
      self.plotMin = 0.0
      self.plotMax = 0.0
      for i in range(3):
        self.minDeque[i].clear()
        self.maxDeque[i].clear()
      self.show_plot_mode_prev = ui_state.show_plot_mode

    return data, title

  def update_plot_queue(self, plot_data):
    self.plotIndex = (self.plotIndex + 1) % self.PLOT_MAX
    for i in range(3):
      if self.plotSize == self.PLOT_MAX:
        if self.minDeque[i] and self.minDeque[i][0] == self.plotQueue[i][self.plotIndex]:
          self.minDeque[i].popleft()
        if self.maxDeque[i] and self.maxDeque[i][0] == self.plotQueue[i][self.plotIndex]:
          self.maxDeque[i].popleft()

      self.plotQueue[i][self.plotIndex] = plot_data[i]

      while self.minDeque[i] and self.minDeque[i][-1] > plot_data[i]:
        self.minDeque[i].pop()
      self.minDeque[i].append(plot_data[i])

      while self.maxDeque[i] and self.maxDeque[i][-1] < plot_data[i]:
        self.maxDeque[i].pop()
      self.maxDeque[i].append(plot_data[i])

    if self.plotSize < self.PLOT_MAX:
      self.plotSize += 1

    self.plotMin = float('inf')
    self.plotMax = -float('inf')
    for i in range(3):
      if self.minDeque[i]:
        self.plotMin = min(self.plotMin, self.minDeque[i][0])
      if self.maxDeque[i]:
        self.plotMax = max(self.plotMax, self.maxDeque[i][0])

    # 최소/최대 범위 고정
    if self.plotMin > -2.0:
      self.plotMin = -2.0
    if self.plotMax < 2.0:
      self.plotMax = 2.0

  def draw(self, renderer):
    if ui_state.show_plot_mode == 0:
      return

    # sm = ui_state.sm
    # if not (sm.alive('carState') and sm.alive('longitudinalPlan')):
    #   print("carState alive:", sm.alive('carState'))
    #   print("longitudinalPlan alive:", sm.alive('longitudinalPlan'))
    #   return

    plot_data, title = self.make_plot_data(renderer)
    self.update_plot_queue(plot_data)

    if getattr(renderer, "_rect", None) is None:
      return
    if renderer._rect.width < 1200:
      return

    COLOR_YELLOW = rl.Color(240, 200, 0, 255)
    COLOR_GREEN  = rl.Color(0, 200, 100, 255)
    COLOR_ORANGE = rl.Color(255, 128, 0, 255)
    COLOR_WHITE  = rl.Color(255,255,255,255)
    colors = [COLOR_YELLOW, COLOR_GREEN, COLOR_ORANGE]

    # Each Channel Plot
    for i in range(3):
      self._draw_plotting(renderer, i, self.plotIndex, self.plotX, self.plotQueue[i], self.plotSize, colors[i], stroke=2)

    # title
    try:
      rl.draw_text_ex(self._font_semi_bold, title, rl.Vector2(float(self.plotX + 150), float(self.plotY - 20)), 25, 0, COLOR_WHITE)
    except Exception:
      pass