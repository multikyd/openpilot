import math
import re
import time
import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.mici.onroad.lead_brake_panel import LeadBrakePanel
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.mici.onroad.hud_layout import (LEFT_CENTER, HEADER_TOP, left_car_box,
                                                        lower_notice_box, gap_bar_boxes, select_road_notice)
from openpilot.selfdrive.ui.mici.onroad.signal_renderer import draw_signal_panel
from openpilot.selfdrive.ui.mici.onroad.bookmark_transfer import BookmarkTransferBanner
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus, ChestnutState
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.cereal import log

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
    self._chestnut_fade_time: float = 0

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)

    self._turn_intent = TurnIntent()
    self._bookmark_transfer = BookmarkTransferBanner()
    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', 50, 50)
    self._txt_wheel_critical: rl.Texture = gui_app.texture('icons_mici/wheel_critical.png', 50, 50)
    self._txt_exclamation_point: rl.Texture = gui_app.texture('icons_mici/exclamation_point.png', 9, 44)
    self._txt_chestnut: rl.Texture = gui_app.texture('icons_mici/chestnut.png', 60, 44)
    self._txt_chestnut_green: rl.Texture = gui_app.texture('icons_mici/chestnut_green.png', 60, 44)
    self._txt_chestnut_orange: rl.Texture = gui_app.texture('icons_mici/chestnut_orange.png', 75, 44)
    self._chestnut_icon: rl.Texture | None = None


    self.img_width = 90
    self.img_car_width, self.img_car_height = 40, 70
    self.img_speed_bump: rl.Texture = gui_app.texture("addon/img/img_speed_bump.png", self.img_width, self.img_width)
    self.img_car: rl.Texture = gui_app.texture("addon/img/car.png", self.img_car_width, self.img_car_height)

    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._chestnut_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

    self.setspeed_changed = False
    self.source_text = ""
    self._camera_offset_text = "--"
    self._camera_offset_frame = -1

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
    self._update_camera_offset(sm)
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    engaged = sm['selfdriveState'].enabled or ui_state.latEnabled
    source_text = ui_state.desiredSource
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged) or (source_text != self.source_text):
      self._set_speed_changed_time = rl.get_time()
      self.source_text = source_text
    if engaged != self._engaged:
      self._chestnut_fade_time = rl.get_time() if engaged else 0
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

    self._draw_model_source(rect)

    self._draw_lower_left(rect)
    self._draw_camera_offset(rect)
    if ui_state.started:
      self._bookmark_transfer.draw(rect, self._font_bold)
    if (self._can_draw_top_icons and ui_state.started and
        ui_state.sm.valid['carrotMan'] and ui_state.sm.alive['carrotMan'] and
        0 <= time.monotonic()-ui_state.sm.recv_time['carrotMan'] < 1):
      draw_signal_panel(rect, self._font_bold, ui_state.sm['carrotMan'].signalDisplay)
    if self._can_draw_top_icons and not self.setspeed_changed:
      self._draw_car_stat(rect)
      if ui_state.bumpTarget <= 0 and ui_state.bumpLearningText:
        # A brief result strip, confined to the middle; retain all left-side status icons.
        text = ui_state.bumpLearningText
        for old, new in (("큰 충격 · 다음", "Hard bump: next"), ("부드러움 · 다음", "Smooth: next"),
                         ("목표 유지", "Keep target"), ("노면 충격 저장", "Impact saved")):
          text = text.replace(old, new)
        if not text.isascii():
          text = "Impact saved"
        if not ui_state.is_metric and ('next ' in text or 'target ' in text):
          prefix, speed = text.rsplit(' ', 1)
          text = f"{prefix} {round(float(speed) * KM_TO_MILE)}"
        size = 22
        width = measure_text_cached(self._font_bold, text, size).x
        rl.draw_rectangle_rounded(rl.Rectangle(rect.x+108, rect.y+188, 288, 46), .2, 8, rl.Color(0, 0, 0, 220))
        rl.draw_text_ex(self._font_bold, text, rl.Vector2(rect.x+252-width/2, rect.y+198), size, 0,
                        rl.Color(255, 220, 70, 255))

  def _update_camera_offset(self, sm) -> None:
    # Parse planner telemetry only on a new plan, never once per rendered frame.
    frame = sm.recv_frame['lateralPlan']
    fresh = (sm.valid['lateralPlan'] and sm.alive['lateralPlan'] and
             frame >= ui_state.started_frame and 0 <= time.monotonic()-sm.recv_time['lateralPlan'] < 1)
    if not fresh:
      self._camera_offset_text = "--"
      self._camera_offset_frame = -1
    elif frame != self._camera_offset_frame:
      self._camera_offset_frame = frame
      match = re.search(r'(?:^|\s)camEq=([^\s|]+)', sm['lateralPlan'].latDebugText)
      try:
        value = float(match.group(1)) if match else math.nan
        # Suppress negative zero while retaining signed manual calibration values.
        self._camera_offset_text = f"{0.0 if abs(value) < .005 else value:.2f}" if math.isfinite(value) else "--"
      except ValueError:
        self._camera_offset_text = "--"

  def _draw_camera_offset(self, rect: rl.Rectangle) -> None:
    if not self._can_draw_top_icons or not ui_state.started or rect.width < 440:
      return
    text = self._camera_offset_text
    size = 32
    measured = measure_text_cached(self._font_bold, text, size)
    size *= min(1., 72 / max(1., measured.x))
    width = measure_text_cached(self._font_bold, text, size).x
    center = rect.x + rect.width / 2
    rl.draw_rectangle_rounded(rl.Rectangle(center-40, rect.y+HEADER_TOP, 80, 40), .2, 8, rl.Color(0, 0, 0, 190))
    rl.draw_text_ex(self._font_bold, text, rl.Vector2(center-width/2, rect.y+7), size, 0, rl.WHITE)

  def _draw_model_source(self, rect: rl.Rectangle) -> None:
    loading = ui_state.chestnut_state == ChestnutState.LOADING
    if loading:
      icon = self._txt_chestnut
      opacity = 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
    elif ui_state.chestnut_state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED):
      icon = self._txt_chestnut_orange
      opacity = 1.0
    elif ui_state.chestnut_state == ChestnutState.ACTIVE:
      icon = self._txt_chestnut_green
      opacity = 1.0
    else:
      return

    if icon is not self._chestnut_icon:
      self._chestnut_fade_time = rl.get_time()
      self._chestnut_icon = icon
    visible = loading or rl.get_time() - self._chestnut_fade_time < SET_SPEED_PERSISTENCE
    alpha = self._chestnut_alpha_filter.update(visible)
    if alpha < 1e-2:
      return

    pos = rl.Vector2(rect.x + rect.width - 10 - icon.width,
                     rect.y + rect.height - 14 - (self._txt_wheel.height + icon.height) / 2)
    rl.draw_texture_ex(icon, pos, 0.0, 1.0, rl.Color(255, 255, 255, int(255 * opacity * alpha)))

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
      rl.draw_texture_ex(self._txt_exclamation_point, rl.Vector2(exclamation_pos_x, exclamation_pos_y), 0.0, 1.0, rl.WHITE)

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
    rl.draw_circle_gradient(rl.Vector2(x + circle_radius, y + circle_radius), circle_radius,
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

  # 536 X 240
  def draw_current_speed_sidebar(self, rect: rl.Rectangle) -> None:
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

    sidebar = rl.Rectangle(rect.x+rect.width-SIDE_PANEL_WIDTH, rect.y, SIDE_PANEL_WIDTH, rect.height)
    LeadBrakePanel.draw_speed(sidebar, self.speed, s.is_metric, speed_color)

  def _draw_lower_left(self, rect: rl.Rectangle) -> None:
    s = ui_state
    sm = s.sm
    fresh = (sm.valid['carrotMan'] and sm.alive['carrotMan'] and
             0 <= time.monotonic()-sm.recv_time['carrotMan'] < 1)
    notice = select_road_notice(s.xSpdType, s.xSpdLimitOrg, s.xSpdLimit, s.xSpdDist,
                                s.bumpTarget, s.bumpDistance) if fresh else None
    # A critical steering instruction retains priority over advisory road notices.
    if (notice is None or self._show_wheel_critical or not self._can_draw_top_icons or
        self.setspeed_changed):
      self._draw_steering_wheel(rect)
      return
    x, y, w, h = lower_notice_box(rect.x, rect.y, rect.height)
    if h < 48:
      self._draw_steering_wheel(rect)
      return
    scale = min(1., h/90)
    cx = rect.x+LEFT_CENTER
    rl.draw_rectangle_rounded(rl.Rectangle(x,y,w,h), .12, 8, rl.Color(0,0,0,190))
    def text(value, top, size, color, available=None, centered=False):
      size *= scale
      measured = measure_text_cached(self._font_bold, value, size)
      size *= min(1., ((w-8) if available is None else available*scale)/max(1., measured.x))
      measured = measure_text_cached(self._font_bold, value, size)
      rl.draw_text_ex(self._font_bold, value, rl.Vector2(cx-measured.x/2, y+top*scale-(measured.y/2 if centered else 0)), size, 0, color)
    target = round(notice.speed*(1 if s.is_metric else KM_TO_MILE))
    if notice.kind == 'CAM':
      radius = 29*scale
      rl.draw_circle(int(cx), int(y+30*scale), radius, rl.RED)
      rl.draw_circle(int(cx), int(y+30*scale), radius*.79, rl.WHITE)
      text(str(target), 30, 40, rl.BLACK, available=42, centered=True)
    else:
      image = self.img_speed_bump
      rl.draw_texture_pro(image, rl.Rectangle(0,0,image.width,image.height),
                          rl.Rectangle(cx-24*scale,y,48*scale,24*scale), rl.Vector2(0,0), 0, rl.WHITE)
      text(str(target), 22, 36, rl.Color(255,220,70,255))
    distance = f'{notice.distance:.0f}m' if notice.distance < 1000 else f'{notice.distance/1000:.1f}km'
    text(distance, 60, 24, rl.WHITE)

  def _draw_car_stat(self, rect: rl.Rectangle) -> None:
    """Draw KisaPilot-style CAR Status."""
    s = ui_state

    img = self.img_car
    icon_x, icon_y, _, _ = left_car_box(rect.x, rect.y)
    x_center = rect.x+LEFT_CENTER
    y_center = icon_y+self.img_car_height/2
    icon_rect = rl.Rectangle(icon_x, icon_y, self.img_car_width, self.img_car_height)
    source_rect = rl.Rectangle(0, 0, img.width, img.height)
    rl.draw_texture_pro(img, source_rect, icon_rect, rl.Vector2(0, 0), 0, rl.Color(255, 255, 255, 150))

    fl = s.tpmsPressureFl
    fr = s.tpmsPressureFr
    rl_p = s.tpmsPressureRl
    rr = s.tpmsPressureRr
    unit = s.tpmsUnit  # 0: psi, 1: kpa, 2: bar

    font_size = 22 if unit == 2 else (20 if unit != 0 else 24)

    def fmt_val(v):
      if v is None or v == 0 or v == 255:
        return ""
      return f"{int(round(v))}" if unit != 2 else f"{v:.1f}"

    def draw_value(offset_x, offset_y, value):
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
                      rl.Vector2(offset_x-tsz/2, offset_y),
                      font_size, 0, col)

    x_offset = 22 if unit == 0 else 23  # left_right wheel distance
    y_offset_front = -img.height // 3.4  # front
    y_offset_rear = img.height // 4.2    # rear

    # offset based on tire loc
    front_text_adjust, rear_text_adjust = -21, -14

    draw_value(x_center - x_offset, y_center + y_offset_front + front_text_adjust, fl)  # Front-left
    draw_value(x_center + x_offset, y_center + y_offset_front + front_text_adjust, fr)  # Front-right
    draw_value(x_center - x_offset, y_center + y_offset_rear + rear_text_adjust, rl_p)  # Rear-left
    draw_value(x_center + x_offset, y_center + y_offset_rear + rear_text_adjust, rr)  # Rear-right

    if s.brakeLights:
      brake_width = 8
      brake_height = 6
      brake_spacing = 14

      brake_left = rl.Rectangle(
        x_center - brake_spacing,
        y_center + img.height // 2 - 7,
        brake_width,
        brake_height
      )
      rl.draw_rectangle_rounded(brake_left, 0.9, 8, rl.Color(255, 0, 0, 180))

      brake_right = rl.Rectangle(
        x_center + brake_spacing - brake_width,
        y_center + img.height // 2 - 7,
        brake_width,
        brake_height
      )
      rl.draw_rectangle_rounded(brake_right, 0.9, 8, rl.Color(255, 0, 0, 180))

    if s.autoHold:
      center_text = "H"
      font = self._font_bold
      font_size = 9
      color = rl.Color(0, 255, 0, 230)
      lines = center_text.split("\n")
      line_height = font_size
      for i, line in enumerate(lines):
        tsz = measure_text_cached(font, line, font_size).x
        rl.draw_text_ex(font, line, rl.Vector2(x_center-tsz/2, icon_y+60+i*10), font_size, 0, color)

    if s.cruise_gap:
      gap_count = max(1, min(4, int(s.cruise_gap)))
      color = {1: rl.Color(220,60,60,255), 2: rl.Color(230,200,60,255),
               3: rl.Color(60,200,120,255), 4: rl.Color(230,230,230,255)}[gap_count]
      for i, box in enumerate(gap_bar_boxes(rect.x, rect.y)):
        ink = color if i < gap_count else rl.Color(70,80,90,140)
        rl.draw_rectangle_rounded(rl.Rectangle(*box), .25, 4, ink)
