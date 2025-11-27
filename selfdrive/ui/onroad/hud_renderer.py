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
  white: rl.Color = rl.WHITE
  disengaged: rl.Color = rl.Color(145, 155, 149, 255)
  override: rl.Color = rl.Color(145, 155, 149, 255)  # Added
  engaged: rl.Color = rl.Color(128, 216, 166, 255)
  disengaged_bg: rl.Color = rl.Color(0, 0, 0, 153)
  override_bg: rl.Color = rl.Color(145, 155, 149, 204)
  engaged_bg: rl.Color = rl.Color(128, 216, 166, 204)
  grey: rl.Color = rl.Color(166, 166, 166, 255)
  dark_grey: rl.Color = rl.Color(114, 114, 114, 255)
  black_translucent: rl.Color = rl.Color(0, 0, 0, 166)
  white_translucent: rl.Color = rl.Color(255, 255, 255, 200)
  border_translucent: rl.Color = rl.Color(255, 255, 255, 75)
  header_gradient_start: rl.Color = rl.Color(0, 0, 0, 114)
  header_gradient_end: rl.Color = rl.BLANK
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
      COLORS.header_gradient_start,
      COLORS.header_gradient_end,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_current_speed(rect)
    self._draw_blinkers(rect)
    self._draw_speed_limit_sign(rect)
    self._draw_standstill_timer(rect)
    self._draw_car_stat(rect)
    self._draw_debug_msg(rect)

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
      bg_brush = COLORS.black_translucent

    # Draw rounded rect background + border
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 32, bg_brush)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 32, 6, COLORS.white_translucent)

    # mid line
    line_y = y + UI_CONFIG.set_speed_height // 2 - 7
    start = rl.Vector2(x + 35, line_y)
    end = rl.Vector2(x + set_speed_width - 35, line_y)
    try:
      rl.draw_line(start, end, 6)
    except Exception:
      rl.draw_rectangle_rounded(rl.Rectangle(start.x, start.y - 3, end.x - start.x, 6), 0.1, 3, COLORS.white)

    setSpeedStr = str(round(self.set_speed)) if 0 < self.set_speed < 254 and s.enabled else CRUISE_DISABLED_CHAR
    # Draw top text
    top_font_size = 80
    setSpeedStr_w = measure_text_cached(self._font_semi_bold, setSpeedStr, top_font_size).x
    rl.draw_text_ex(self._font_semi_bold, setSpeedStr, rl.Vector2(x + (set_speed_width - setSpeedStr_w) / 2, y), top_font_size, 0, COLORS.white)

    # Draw bottom text
    if not (s.has_longitudinal_control or s.camera_scc > 0):
      bottom_text = str(int(s.vSetDis)) if s.enabled else CRUISE_DISABLED_CHAR
    else:
      bottom_text = str(int(min(s.desiredSpeed, self.set_speed))) if s.enabled else CRUISE_DISABLED_CHAR
    bottom_font_size = FONT_SIZES.set_speed + 3
    bottom_text_w = measure_text_cached(self._font_bold, bottom_text, bottom_font_size).x
    rl.draw_text_ex(self._font_bold, bottom_text, rl.Vector2(x + (set_speed_width - bottom_text_w) / 2, y + 90), bottom_font_size, 0, COLORS.white)

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
      speed_color = COLORS.white

    set_speed_width = UI_CONFIG.set_speed_width_metric if s.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 50 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 1020 - 225
    speed_pos = rl.Vector2(x, y)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed + 10, 0, speed_color)

    # unit_text = "KPH" if ui_state.is_metric else "MPH"
    # unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    # unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    # rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.white_translucent)

  def _draw_blinkers(self, rect: rl.Rectangle) -> None:
    """Draw KisaPilot-style blinkers."""
    if not ui_state.leftBlinker and not ui_state.rightBlinker:
      return

    t = rl.get_time()
    center_x = rect.x + rect.width // 2
    center_y = rect.y + 200
    size, thickness, freq, sway_amp = 100, 40, 6, 20
    sway = sway_amp * math.sin(t * freq)
    count, spacing = 5, 90
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
    rl.draw_rectangle_rounded_lines_ex(rects["dist"], 0.35, 32, 6, COLORS.white_translucent)

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

      stop_x = rect.x + rect.width - UI_CONFIG.border_size - 645
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
    y_center = rect.y + 450
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

  def _draw_debug_msg(self, rect: rl.Rectangle) -> None:
    if ui_state.debug_msg > 0:
      s = ui_state
      x = rect.x + UI_CONFIG.border_size + 350
      y = rect.y + UI_CONFIG.border_size + 50
      line_height = 50

      debug_lines = [
        f"Storage Usage: {s.storageUsage:.0f}%",
        f"Memory Usage: {s.memoryUsage:.0f}%",
        f"CPU Usage: {s.cpuUsage:.0f}%",
        f"CPU Temp: {s.cpuTemp:.0f}°C",
        f"GPU Temp: {s.gpuTemp:.0f}°C",
        f"DSP Temp: {s.dspTemp:.0f}°C",
        f"Memory Temp: {s.memoryTemp:.0f}°C",
        f"Modem Temp: {s.modemTemp:.0f}°C",
        f"PMIC Temp: {s.pmicTemp:.0f}°C",
        f"Max Temp: {s.maxTemp:.0f}°C",
        f"Fan Speed: {s.fanSpeedRpm}",
        f"Voltage: {s.voltage:.1f}V",
        f"GPS Acc: {s.gpsAccuracy:.1f}m",
        f"Altitude: {s.altitude:.0f}m",
        f"Bearing: {s.bearing:.0f}°",
      ]

      for i, msg in enumerate(debug_lines):
        rl.draw_text_ex(
          self._font_bold,
          msg,
          rl.Vector2(x, y + i * line_height),
          40,  #font size
          0,   #spacing
          rl.Color(255, 255, 255, 220),
      )

    if ui_state.debug_msg > 1:
      x = rect.x + UI_CONFIG.border_size + 850
      y = rect.y + UI_CONFIG.border_size + 50
      line_height = 50

      debug_lines = [
        f"Panda: {s.pandaSafetyModel}",
        f"Interface: {s.interfaceSafetyModel}",
        f"RX Checks: {'PASS' if s.rxChecks else 'FAIL'}",
        f"MissCnt Check: {'PASS' if s.mismatchCounter else 'FAIL'}",
        f"Controls Allowed: {'YES' if s.controlAllowed else 'NO'}",
        f"Enabled: {'Lat' if s.latEnabled else 'NO'}/{'Yes' if s.enabled else 'NO'}",
        f"AngleOffset: {s.angleOffsetDeg:.1f}°",
        f"AngleOffsetAvg: {s.angleOffsetAverageDeg:.1f}°",
        f"steerRatio: {s.steerRatio:.2f}",
      ]

      for i, msg in enumerate(debug_lines):
        rl.draw_text_ex(
          self._font_bold,
          msg,
          rl.Vector2(x, y + i * line_height),
          40,  #font size
          0,   #spacing
          rl.Color(255, 255, 255, 220),
      )