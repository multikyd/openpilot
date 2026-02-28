import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from cereal import log

EventName = log.OnroadEvent.EventName

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 36
  set_speed: int = 112


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


class TurnIntent(Widget):
  FADE_IN_ANGLE = 30  # degrees

  def __init__(self):
    super().__init__()
    self._pre = False
    self._turn_intent_direction: int = 0

    self._turn_intent_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._turn_intent_rotation_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._txt_turn_intent_left: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20)
    self._txt_turn_intent_right: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20, flip_x=True)

  def _render(self, _):
    if self._turn_intent_alpha_filter.x > 1e-2:
      turn_intent_texture = self._txt_turn_intent_right if self._turn_intent_direction == 1 else self._txt_turn_intent_left
      src_rect = rl.Rectangle(0, 0, turn_intent_texture.width, turn_intent_texture.height)
      dest_rect = rl.Rectangle(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2,
                               turn_intent_texture.width, turn_intent_texture.height)

      origin = (turn_intent_texture.width / 2, self._rect.height / 2)
      color = rl.Color(255, 255, 255, int(255 * self._turn_intent_alpha_filter.x))
      rl.draw_texture_pro(turn_intent_texture, src_rect, dest_rect, origin, self._turn_intent_rotation_filter.x, color)

  def _update_state(self) -> None:
    sm = ui_state.sm

    left = any(e.name == EventName.preLaneChangeLeft for e in sm['onroadEvents'])
    right = any(e.name == EventName.preLaneChangeRight for e in sm['onroadEvents'])
    if left or right:
      # pre lane change
      if not self._pre:
        self._turn_intent_rotation_filter.x = self.FADE_IN_ANGLE if left else -self.FADE_IN_ANGLE

      self._pre = True
      self._turn_intent_direction = -1 if left else 1
      self._turn_intent_alpha_filter.update(1)
      self._turn_intent_rotation_filter.update(0)
    elif any(e.name == EventName.laneChange for e in sm['onroadEvents']):
      # fade out and rotate away
      self._pre = False
      self._turn_intent_alpha_filter.update(0)

      if self._turn_intent_direction == 0:
        # unknown. missed pre frame?
        self._turn_intent_rotation_filter.update(0)
      else:
        self._turn_intent_rotation_filter.update(self._turn_intent_direction * self.FADE_IN_ANGLE)
    else:
      # didn't complete lane change, just hide
      self._pre = False
      self._turn_intent_direction = 0
      self._turn_intent_alpha_filter.update(0)
      self._turn_intent_rotation_filter.update(0)


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._turn_intent = TurnIntent()
    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', 50, 50)
    self._txt_wheel_critical: rl.Texture = gui_app.texture('icons_mici/wheel_critical.png', 50, 50)
    self._txt_exclamation_point: rl.Texture = gui_app.texture('icons_mici/exclamation_point.png', 44, 44)

    self.img_width = 90
    self.img_car_width, self.img_car_height = 40, 70
    self.img_speed_bump: rl.Texture = gui_app.texture("addon/img/img_speed_bump.png", self.img_width, self.img_width)
    self.img_car: rl.Texture = gui_app.texture("addon/img/car.png", self.img_car_width, self.img_car_height)

    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    self.setspeed_changed = False
    self.source_text = ""

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha_filter.x > 1e-2)

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    set_speed = (
      controls_state.vCruiseDEPRECATED if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    engaged = sm['selfdriveState'].enabled or ui_state.latEnabled
    source_text = ui_state.desiredSource
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged) or (source_text != self.source_text):
      self._set_speed_changed_time = rl.get_time()
      self.source_text = source_text
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""

    self._torque_bar.render(rect)

    if self.is_cruise_set:
      self._draw_set_speed(rect)

    self._draw_steering_wheel(rect)
    self._draw_current_speed(rect)
    self._draw_standstill_timer(rect)
    if self._can_draw_top_icons and not self.setspeed_changed:
      self._draw_car_stat(rect)
      self._draw_speed_limit_sign(rect)

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else self._txt_wheel

    if self._show_wheel_critical:
      self._wheel_alpha_filter.update(255)
      self._wheel_y_filter.update(0)
    else:
      if ui_state.status == UIStatus.DISENGAGED:
        self._wheel_alpha_filter.update(0)
        self._wheel_y_filter.update(wheel_txt.height / 2)
      else:
        self._wheel_alpha_filter.update(255 * 0.9)
        self._wheel_y_filter.update(0)

    # pos
    pos_x = int(rect.x + 21 + wheel_txt.width / 2)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + self._wheel_y_filter.x)
    rotation = -ui_state.sm['carState'].steeringAngleDeg

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(
      pos_x - wheel_txt.width / 2 - turn_intent_margin,
      pos_y - wheel_txt.height / 2 - turn_intent_margin,
      wheel_txt.width + turn_intent_margin * 2,
      wheel_txt.height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height)
    origin = (wheel_txt.width / 2, wheel_txt.height / 2)

    # color and draw
    color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, color)

    if self._show_wheel_critical:
      # Draw exclamation point icon
      EXCLAMATION_POINT_SPACING = 10
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 2
      rl.draw_texture(self._txt_exclamation_point, int(exclamation_pos_x), int(exclamation_pos_y), rl.WHITE)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    alpha = self._set_speed_alpha_filter.update(0 < rl.get_time() - self._set_speed_changed_time < SET_SPEED_PERSISTENCE and
                                                self._can_draw_top_icons and self._engaged)
    if alpha < 1e-2:
      self.setspeed_changed = False
      return
    else:
      self.setspeed_changed = True

    x = rect.x
    y = rect.y

    # draw drop shadow
    circle_radius = 162 // 2
    rl.draw_circle_gradient(int(x + circle_radius), int(y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(x + 7, y - 10),
      FONT_SIZES.set_speed - 30,
      0,
      set_speed_color,
    )

    #max_text = tr("MAX")
    if not (ui_state.has_longitudinal_control or ui_state.camera_scc > 0):
      max_text = str(int(ui_state.vSetDis)) if ui_state.enabled else CRUISE_DISABLED_CHAR
    else:
      max_text = str(int(min(ui_state.desiredSpeed, self.set_speed))) if ui_state.enabled else CRUISE_DISABLED_CHAR
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + 7, y + FONT_SIZES.set_speed - 55),
      FONT_SIZES.max_speed + 35,
      0,
      max_color,
    )

    # source text
    if ui_state.desiredSpeed <= self.set_speed:
      source_color = rl.Color(255, 255, 0, int(255 * alpha))
      rl.draw_text_ex(
        self._font_semi_bold,
        ui_state.desiredSource,
        rl.Vector2(x + 7, y + FONT_SIZES.set_speed),
        FONT_SIZES.max_speed + 20,
        0,
        source_color,
      )

  # def _draw_current_speed(self, rect: rl.Rectangle) -> None:
  #   """Draw the current vehicle speed and unit."""
  #   speed_text = str(round(self.speed))
  #   speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
  #   speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
  #   rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

  #   unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
  #   unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
  #   unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
  #   rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)

  # 536 X 240
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

    # speed_pos = rl.Vector2(rect.x + rect.width - 100, rect.y)
    # rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed - 100, 0, speed_color)

    font_size = FONT_SIZES.current_speed - 100
    text_size = rl.measure_text_ex(self._font_bold, speed_text, font_size, 0)
    right_x = rect.x + rect.width - 20
    speed_pos = rl.Vector2(right_x - text_size.x, rect.y)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, font_size, 0, speed_color)

  def _draw_speed_limit_sign(self, rect: rl.Rectangle) -> None:
    """Draw KisaPilot-style speed limit sign."""
    s_center_x = rect.x + 170
    s_center_y = rect.y + 60
    d_center_y = s_center_y + 60

    diameters = (100, 80, 91)
    rects = {
      "inner": rl.Rectangle(s_center_x - diameters[1]//2, s_center_y - diameters[1]//2, diameters[1], diameters[1]),
      "dist":  rl.Rectangle(s_center_x - 55, d_center_y, 115, 35),
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
      font_size = 60 if limit_spd < 100 else 50
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

    font_size = 33
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

      time_x = rect.x + rect.width - 150
      time_y = rect.y + 100

      time_color = rl.Color(204, 119, 34, 220)

      rl.draw_text_ex(self._font_bold, time_text, rl.Vector2(time_x, time_y),
                      50, 0, time_color)

  def _draw_car_stat(self, rect: rl.Rectangle) -> None:
    """Draw KisaPilot-style CAR Status."""
    s = ui_state

    img = self.img_car
    x_center = rect.x + 26 + img.width // 2
    y_center = rect.y + 135
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

    font_size = 20 if unit == 2 else (18 if unit != 0 else 21)

    def fmt_val(v):
      if v is None or v == 0 or v == 255:
        return ""
      return f"{int(round(v))}" if unit != 2 else f"{v:.1f}"

    def draw_value(offset_x, offset_y, value):
      offset = 0.6 if unit == 2 else (0.6 if unit != 0 else 0.7)
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

    offset_add = 1.5 if unit == 2 else (1.2 if unit != 0 else 1.7)
    x_offset = img.width // offset_add  # left_right wheel distance
    y_offset_front = -img.height // 3.4  # front
    y_offset_rear = img.height // 4.2    # rear

    # offset based on tire loc
    y_text_adjust = -10

    draw_value(x_center - x_offset, y_center + y_offset_front + y_text_adjust, fl)  # Front-left
    draw_value(x_center + x_offset, y_center + y_offset_front + y_text_adjust, fr)  # Front-right
    draw_value(x_center - x_offset, y_center + y_offset_rear + y_text_adjust, rl_p)  # Rear-left
    draw_value(x_center + x_offset, y_center + y_offset_rear + y_text_adjust, rr)  # Rear-right

    if s.brakeLights:
      brake_width = 8
      brake_height = 6
      brake_spacing = 14

      brake_left = rl.Rectangle(
        x_center - brake_spacing - brake_width + 14,
        y_center + img.height // 2 - 7,
        brake_width,
        brake_height
      )
      rl.draw_rectangle_rounded(brake_left, 0.9, 8, rl.Color(255, 0, 0, 180))

      brake_right = rl.Rectangle(
        x_center + brake_spacing - 6,
        y_center + img.height // 2 - 7,
        brake_width,
        brake_height
      )
      rl.draw_rectangle_rounded(brake_right, 0.9, 8, rl.Color(255, 0, 0, 180))

    if s.autoHold:
      center_text = "AT\nHD"
      font = self._font_bold
      font_size = 13
      color = rl.Color(0, 255, 0, 230)
      lines = center_text.split("\n")
      line_height = font_size
      total_height = line_height * len(lines)

      for i, line in enumerate(lines):
        tsz = measure_text_cached(font, line, font_size).x
        rl.draw_text_ex(font, line, rl.Vector2(x_center - tsz / 2 + 4, y_center - total_height / 2 + i * line_height - 2), font_size, 0, color)

    if s.cruise_gap:
      gap_count = int(s.cruise_gap)
      gap_w, gap_h, spacing, radius = 18, 2, 2, 0
      if gap_count == 1:
        color = rl.Color(220, 60, 60, 255)    # Red
      elif gap_count == 2:
        color = rl.Color(230, 200, 60, 255)   # Yellow
      elif gap_count == 3:
        color = rl.Color(60, 200, 120, 255)   # Green
      else:
        color = rl.Color(230, 230, 230, 255)  # White
      border = rl.WHITE
      x = x_center - gap_w // 2 + 4
      base_y = y_center - img.height // 2 + 2

      for i in range(gap_count):
        y = base_y - i * (gap_h + spacing)
        rect = rl.Rectangle(x, y, gap_w, gap_h)
        rl.draw_rectangle_rounded(rect, 0.9, radius, color)
        rl.draw_rectangle_rounded_lines(rect, radius, 1, border)

      lead_dist = s.radarDRel

      if lead_dist is not None and 0 < lead_dist < 150:
        dist_text = f"{lead_dist:.1f} m"

        text_font = self._font_bold
        text_size = 20
        tsz = measure_text_cached(text_font, dist_text, text_size).x

        top_y = base_y - (gap_count * (gap_h + spacing)) - 18

        if lead_dist < 5:
          color = rl.Color(255, 0, 0, 255)
        elif lead_dist < 10:
          color = rl.Color(255, 140, 0, 255)
        else:
          color = rl.Color(255, 255, 255, 230)

        rl.draw_text_ex(text_font, dist_text, rl.Vector2(x_center - tsz / 2 + 5, top_y), text_size, 0, color)