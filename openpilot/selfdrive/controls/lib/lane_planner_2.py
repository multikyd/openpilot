import math
import numpy as np
from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
# from openpilot.common.swaglog import cloudlog
# from openpilot.common.logger import sLogger
from openpilot.common.params import Params

TRAJECTORY_SIZE = 33
# positive numbers go right
CAMERA_OFFSET = 0.04 #0.08
PATH_OFFSET = CAMERA_OFFSET - 0.04

MIN_LANE_DISTANCE = 2.6
MAX_LANE_DISTANCE = 3.7
MAX_LANE_CENTERING_AWAY = 1.85
KEEP_MIN_DISTANCE_FROM_LANE = 1.35
KEEP_MIN_DISTANCE_FROM_EDGELANE = 1.15

def clamp(num, min_value, max_value):
  # weird broken case, do something reasonable
  if min_value > num > max_value:
    return (min_value + max_value) * 0.5
  # ok, basic min/max below
  if num < min_value:
    return min_value
  if num > max_value:
    return max_value
  return num

def sigmoid(x, scale=1, offset=0):
  return (1 / (1 + math.exp(x*scale))) + offset

def lerp(start, end, t):
  t = clamp(t, 0.0, 1.0)
  return (start * (1.0 - t)) + (end * t)

def max_abs(a, b):
  return a if abs(a) > abs(b) else b

class LanePlanner:
  def __init__(self):
    self.params = Params()

    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    #self.lane_width_estimate = FirstOrderFilter(3.2, 9.95, DT_MDL)
    self.lane_width_estimate = FirstOrderFilter(3.2, 3.0, DT_MDL)
    self.lane_width = 3.2
    self.lane_width_last = self.lane_width
    self.lane_change_multiplier = 1
    #self.lane_width_updated_count = 0

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.debugText = ""
    self.lane_width_left = 0.0
    self.lane_width_right = 0.0
    self.lane_width_left_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_width_right_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_offset_filtered = FirstOrderFilter(0.0, 2.0, DT_MDL)
    self.lane_center_offset_filtered = FirstOrderFilter(0.0, 0.8, DT_MDL)
    self.lane_center_offset = self.params.get("LaneCenterOffset", return_default=True)
    self.lane_center_curvature = 0.0
    self.lane_position_geometry = None
    self.lane_position_class = 'unknown'
    self.lane_position_candidate = 'unknown'
    self.lane_position_confirm = 0.0
    self.lane_position_offset = 0.0
    # Session-only correction of a sustained lane tracking error, separate from preference.
    self.lane_tracking_trim = 0.0
    self.lane_tracking_applied = 0.0
    self.lane_tracking_stable = 0.0
    self.lane_tracking_ready = False

    self.lanefull_mode = False
    self.d_prob_count = 0
    self.second = 0.0

    self.camera_offset = CAMERA_OFFSET
    self.path_offset = self.camera_offset - 0.04
    self.path_offset2 = 0.0
    self.road_edge_offset = 0.0
    self.timer = 0
    self.timer2 = 0
    self.total_camera_offset = self.camera_offset
    self.right_lane_to_right_edge_width = 0.0
    self.left_lane_to_left_edge_width = 0.0
    self.left_edge_offset = self.params.get("LeftEdgeOffset", return_default=True)
    self.right_edge_offset = self.params.get("RightEdgeOffset", return_default=True)

    self.adjustLaneOffset = self.params.get("AdjustLaneOffset")
    self.adjustCurveOffset = self.adjustLaneOffset #self.params.get("AdjustCurveOffset")
    # Actuator preview belongs in the curvature command, not the lane geometry.

  def parse_model(self, md):

    self.timer += DT_MDL
    if self.timer > 2.0:
      self.timer = 0.0
      self.camera_offset = self.params.get("CameraOffsetAdj", return_default=True)
      self.path_offset = self.camera_offset - 0.04
      self.adjustLaneOffset = self.params.get("AdjustLaneOffset")
      self.adjustCurveOffset = self.adjustLaneOffset #self.params.get("AdjustCurveOffset")
      self.lane_center_offset = self.params.get("LaneCenterOffset", return_default=True)

    if self.left_edge_offset != 0.0 or self.right_edge_offset != 0.0: # kisapilot
      left_edge_prob = np.clip(1.0 - md.roadEdgeStds[0], 0.0, 1.0)
      left_nearside_prob = md.laneLineProbs[0]
      left_close_prob = md.laneLineProbs[1]
      right_close_prob = md.laneLineProbs[2]
      right_nearside_prob = md.laneLineProbs[3]
      right_edge_prob = np.clip(1.0 - md.roadEdgeStds[1], 0.0, 1.0)

      self.right_lane_to_right_edge_width = abs(self.rll_y[0] - md.roadEdges[1].y[0])
      self.left_lane_to_left_edge_width = abs(self.lll_y[0] - md.roadEdges[0].y[0])
      lane_to_edge_threshold = 2.6

      self.timer2 += DT_MDL
      if self.timer2 > 1.0:
        self.timer2 = 0.0
        if right_nearside_prob < 0.2 and left_nearside_prob < 0.2:
          if self.road_edge_offset != 0.0:
            if self.road_edge_offset > 0.0:
              self.road_edge_offset -= max(0.01, round((self.road_edge_offset/5), 2))
              if self.road_edge_offset < 0.0:
                self.road_edge_offset = 0.0
            elif self.road_edge_offset < 0.0:
              self.road_edge_offset += max(0.01, round((self.road_edge_offset/5), 2))
              if self.road_edge_offset > 0.0:
                self.road_edge_offset = 0.0
        elif right_edge_prob > 0.3 and right_nearside_prob < 0.3 and right_close_prob > 0.4 and left_nearside_prob >= right_nearside_prob and self.right_lane_to_right_edge_width < lane_to_edge_threshold:
          if self.right_edge_offset != 0.0 and self.road_edge_offset != self.right_edge_offset:
            if self.road_edge_offset < self.right_edge_offset:
              self.road_edge_offset += max(0.01, round((self.right_edge_offset/5), 2))
              if self.road_edge_offset > self.right_edge_offset:
                self.road_edge_offset = self.right_edge_offset
            elif self.road_edge_offset > self.right_edge_offset:
              self.road_edge_offset -= max(0.01, round((self.right_edge_offset/5), 2))
              if self.road_edge_offset < self.right_edge_offset:
                self.road_edge_offset = self.right_edge_offset
        elif left_edge_prob > 0.3 and left_nearside_prob < 0.3 and left_close_prob > 0.4 and right_nearside_prob >= left_nearside_prob and self.left_lane_to_left_edge_width < lane_to_edge_threshold:
          if self.left_edge_offset != 0.0 and self.road_edge_offset != self.left_edge_offset:
            if self.road_edge_offset < self.left_edge_offset:
              self.road_edge_offset += max(0.01, round((self.left_edge_offset/5), 2))
              if self.road_edge_offset > self.left_edge_offset:
                self.road_edge_offset = self.left_edge_offset
            elif self.road_edge_offset > self.left_edge_offset:
              self.road_edge_offset -= max(0.01, round((self.left_edge_offset/5), 2))
              if self.road_edge_offset < self.left_edge_offset:
                self.road_edge_offset = self.left_edge_offset
        else:
          if self.road_edge_offset != 0.0:
            if self.road_edge_offset > 0.0:
              self.road_edge_offset -= max(0.01, round((self.road_edge_offset/5), 2))
              if self.road_edge_offset < 0.0:
                self.road_edge_offset = 0.0
            elif self.road_edge_offset < 0.0:
              self.road_edge_offset += max(0.01, round((self.road_edge_offset/5), 2))
              if self.road_edge_offset > 0.0:
                self.road_edge_offset = 0.0
        self.path_offset2 = self.road_edge_offset
    else:
      self.road_edge_offset = 0.0
      self.path_offset2 = self.road_edge_offset

    self.total_camera_offset = self.camera_offset + self.road_edge_offset

    lane_lines = md.laneLines
    edges = md.roadEdges
    self.lane_position_geometry = None
    if (len(lane_lines) >= 4 and len(edges) >= 2 and len(md.laneLineProbs) >= 4 and
        len(md.roadEdgeStds) >= 2 and all(len(line.y) for line in (lane_lines[1], lane_lines[2], edges[0], edges[1]))):
      self.lane_position_geometry = (md.laneLineProbs[0], md.laneLineProbs[3],
                                     md.roadEdgeStds[0], md.roadEdgeStds[1],
                                     edges[0].y[0], edges[1].y[0], lane_lines[1].y[0], lane_lines[2].y[0])

    if len(lane_lines) >= 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y) + self.total_camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.total_camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(edges[0].t) == TRAJECTORY_SIZE:
      self.le_y = np.array(edges[0].y) + md.roadEdgeStds[0] * 0.4
      self.re_y = np.array(edges[1].y) - md.roadEdgeStds[1] * 0.4
    else:
      self.le_y = self.lll_y
      self.re_y = self.rll_y

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.Desire.laneChangeRight]

  def _get_lane_center_offset(self, CS, laneline_active):
    # Positive y is right. Keep this preference separate from camera calibration.
    value = self.lane_center_offset
    curvature = abs(self.lane_center_curvature)
    enabled = (laneline_active and not CS.leftBlinker and not CS.rightBlinker and
               value is not None and np.isfinite(value) and np.isfinite(curvature))
    target = 0.0
    if enabled:
      # Full preference on straight roads; fade out between radii 500m and 200m.
      curve_factor = np.interp(curvature, [0.002, 0.005], [1.0, 0.0])
      target = float(np.clip(value, -0.05, 0.05)) * curve_factor
    filtered = self.lane_center_offset_filtered.update(target)
    if not enabled:
      return 0.0
    confidence = np.interp(self.d_prob, [0.3, 0.8], [0.0, 1.0])
    width_factor = np.interp(self.lane_width, [2.8, 3.2], [0.0, 1.0])
    return float(filtered * confidence * width_factor)

  def _get_lane_position_offset(self, CS, laneline_active):
    # A preference is added only to a well-observed lane path, never to model mode.
    enabled = (laneline_active and self.lane_change_multiplier > .5 and
               not CS.leftBlinker and not CS.rightBlinker and not getattr(CS, 'steeringPressed', False))
    if not enabled:
      self.lane_position_class = self.lane_position_candidate = 'unknown'
      self.lane_position_confirm = self.lane_position_offset = 0.0
      return 0.0

    candidate = 'unknown'
    geometry = self.lane_position_geometry
    values = (self.lane_width_left, self.lane_width_right, self.l_prob, self.r_prob, self.lane_center_curvature)
    valid = geometry is not None and np.all(np.isfinite((*geometry, *values)))
    width = center = 0.0
    if valid:
      outer_left, outer_right, edge_std_left, edge_std_right, edge_left, edge_right, lane_left, lane_right = geometry
      width, center = lane_right-lane_left, (lane_left+lane_right)/2.
      left, right = self.lane_width_left, self.lane_width_right
      if 2.6 < width < 4.2 and min(self.l_prob, self.r_prob) >= .75:
        if (left > 2.2 and .1 < right < 1.8 and outer_left > .3 and outer_right < .15 and
            0. <= edge_std_right < .6 and -.1 <= edge_right-lane_right <= 1.8):
          candidate = 'right'
        elif (right > 2.2 and .1 < left < 1.8 and outer_right > .3 and outer_left < .15 and
              0. <= edge_std_left < .6 and -.1 <= lane_left-edge_left <= 1.8):
          candidate = 'left'
        elif (left > 2.2 and right > 2.2 and min(outer_left, outer_right) > .3):
          candidate = 'middle'
        elif (left > 2.5 and right > 2.5 and
              not (0. <= edge_std_left < .6 and lane_left-edge_left < 1.8) and
              not (0. <= edge_std_right < .6 and edge_right-lane_right < 1.8)):
          # Neighbor-width predictions can survive faint outer paint. A nearby
          # trustworthy road edge contradicts a neighboring lane and vetoes this.
          candidate = 'middle' 

    if candidate != self.lane_position_candidate:
      self.lane_position_candidate = candidate
      self.lane_position_confirm = 0.0
    self.lane_position_confirm = min(2., self.lane_position_confirm+DT_MDL)
    self.lane_position_class = candidate if self.lane_position_confirm >= 1.0 else 'unknown'

    target = 0.0
    if valid and self.lane_position_class != 'unknown' and abs(center) < .35:
      # Let the normal controller finish recentering after a lane change first.
      preference = {'right': .05, 'left': -.05, 'middle': -.03}[self.lane_position_class]
      curve_factor = float(np.interp(abs(self.lane_center_curvature), [.002, .005], [1., 0.]))
      width_factor = float(np.interp(width, [2.6, 2.8], [0., 1.]))
      target = preference*curve_factor*width_factor
      # Keep at least 1.25 m from the target center to a lane boundary or a trusted edge.
      budget = max(0., min(.05, width/2.-1.25))
      target = float(np.clip(target, -budget, budget))
      if target > 0 and 0 <= edge_std_right < .6:
        target = min(target, max(0., edge_right-center-1.25))
      elif target < 0 and 0 <= edge_std_left < .6:
        target = max(target, -max(0., center-edge_left-1.25))
    # Limit lateral target drift to 1.5 cm/s; never jump between edge preferences.
    self.lane_position_offset += float(np.clip(target-self.lane_position_offset, -.015*DT_MDL, .015*DT_MDL))
    if valid:
      # A shrinking safety margin overrides the gradual comfort transition.
      left_budget = right_budget = max(0., min(.05, width/2.-1.25))
      if 0 <= edge_std_left < .6:
        left_budget = min(left_budget, max(0., center-edge_left-1.25))
      if 0 <= edge_std_right < .6:
        right_budget = min(right_budget, max(0., edge_right-center-1.25))
      self.lane_position_offset = float(np.clip(self.lane_position_offset, -left_budget, right_budget))
    return self.lane_position_offset

  def _get_lane_tracking_offset(self, CS, laneline_active, desired_offset):
    # Learn only on settled, straight, confidently observed middle-lane driving.
    # Never learn a driver's correction, a lane change or a cornering transient.
    geometry = self.lane_position_geometry
    values = (self.l_prob, self.r_prob, self.lane_center_curvature, desired_offset,
              self.camera_offset, getattr(CS, 'vEgo', math.nan), getattr(CS, 'steeringRateDeg', math.nan))
    valid = geometry is not None and np.all(np.isfinite((*geometry, *values)))
    active = (laneline_active and self.lane_change_multiplier > .5 and
              not CS.leftBlinker and not CS.rightBlinker and not getattr(CS, 'steeringPressed', False))
    if not valid or not active:
      self.lane_tracking_stable = self.lane_tracking_applied = 0.0
      return 0.0
    _, _, edge_std_left, edge_std_right, edge_left, edge_right, lane_left, lane_right = geometry
    width = lane_right-lane_left
    center = (lane_left+lane_right)/2. + self.camera_offset
    good = (self.lane_tracking_ready and self.lane_position_class == 'middle' and
            self.lane_position_confirm >= 1. and min(self.l_prob, self.r_prob) >= .9 and
            3.0 <= width <= 4.0 and 10. <= CS.vEgo <= 40. and
            abs(self.lane_center_curvature) < .0005 and
            abs(CS.steeringRateDeg) < 3. and abs(center) < .30)
    self.lane_tracking_stable = min(3., self.lane_tracking_stable+DT_MDL) if good else 0.0
    if self.lane_tracking_stable >= 2.:
      # Positive error means right of the desired location. Do not include this
      # integral's own output in the reference or it would cancel its correction.
      error = -center-desired_offset
      if abs(error) > .01:
        step = float(np.clip(-.04*error*DT_MDL, -.003*DT_MDL, .003*DT_MDL))
        self.lane_tracking_trim = float(np.clip(self.lane_tracking_trim+step, -.12, .12))
    # Hold the session estimate through turns, but fade its application before
    # tight curvature. Returning to a lane ramps in at <=5 mm/s.
    curve_factor = float(np.interp(abs(self.lane_center_curvature), [.001, .003], [1., 0.]))
    confidence = float(np.interp(min(self.l_prob, self.r_prob), [.75, .9], [0., 1.]))
    width_factor = float(np.interp(width, [2.8, 3.0], [0., 1.]))
    target = self.lane_tracking_trim*curve_factor*confidence*width_factor
    self.lane_tracking_applied += float(np.clip(target-self.lane_tracking_applied, -.005*DT_MDL, .005*DT_MDL))
    # Geometry safety limits take priority over the comfort slew. Maintain
    # 1.25 m between the complete desired center and either boundary/valid edge.
    left_margin = right_margin = max(0., width/2.-1.25)
    raw_center = (lane_left+lane_right)/2.
    if 0. <= edge_std_left < .6:
      left_margin = min(left_margin, max(0., raw_center-edge_left-1.25))
    if 0. <= edge_std_right < .6:
      right_margin = min(right_margin, max(0., edge_right-raw_center-1.25))
    # The learned correction itself must not worsen a margin already consumed
    # by existing explicit calibration/preferences.
    low = min(0., -left_margin-desired_offset)
    high = max(0., right_margin-desired_offset)
    applied_low, applied_high = sorted((0., target))
    self.lane_tracking_applied = float(np.clip(self.lane_tracking_applied,
                                             max(low, applied_low), min(high, applied_high)))
    return self.lane_tracking_applied

  def get_d_path(self, CS, v_ego, path_t, path_xyz, curve_speed):
    #if v_ego > 0.1:
    #  self.lane_width_updated_count = max(0, self.lane_width_updated_count - 1)
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = np.interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      #prob_mods.append(np.interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
      prob_mods.append(np.interp(width_at_t, [4.5, 6.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = np.interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = np.interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    self.l_prob, self.r_prob = l_prob, r_prob

    # Find current lanewidth
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])

    both_lane_available = False
    #speed_lane_width = np.interp(v_ego*3.6, [0., 60.], [2.8, 3.5])
    if l_prob > 0.5 and r_prob > 0.5 and self.lane_change_multiplier > 0.5:
      both_lane_available = True
      #self.lane_width_updated_count = max_updated_count
      self.lane_width_estimate.update(current_lane_width)
      self.lane_width_last = self.lane_width_estimate.x
    #elif self.lane_width_updated_count <= 0 and v_ego > 0.1:   # 양쪽차선이 없을때.... 일정시간후(10초)부터 speed차선폭 적용함.
    #  self.lane_width_estimate.update(speed_lane_width)
    else:
      self.lane_width_estimate.update(self.lane_width_last)

    self.lane_width =  self.lane_width_estimate.x
    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    # 가장 차선이 진한쪽으로 골라서..
    self.d_prob = max(l_prob, r_prob) if not both_lane_available else 1.0

    # 좌/우의 차선폭을 필터링.
    if self.lane_width_left > 0:
      self.lane_width_left_filtered.update(self.lane_width_left)
      #self.lane_width_left_filtered.x = self.lane_width_left #바로적용
    if self.lane_width_right > 0:
      self.lane_width_right_filtered.update(self.lane_width_right)
      #self.lane_width_right_filtered.x = self.lane_width_right #바로적용

    ADJUST_OFFSET_LIMIT = 0.4 #max(self.adjustLaneOffset, self.adjustCurveOffset)
    offset_curve = 0.0
    ## curve offset
    offset_curve = np.interp(abs(curve_speed), [50, 200], [self.adjustCurveOffset, 0.0]) * np.sign(curve_speed)

    offset_lane = 0.0
    if self.lane_width_left_filtered.x > 2.2 and self.lane_width_right_filtered.x > 2.2: #양쪽에 차로가 여유 있는경우
      offset_lane = 0.0
    elif self.lane_width_left_filtered.x < 2.0 and self.lane_width_right_filtered.x < 2.0: #양쪽에 차로가 여유 없는경우
      offset_lane = 0.0
    elif self.lane_width_left_filtered.x > self.lane_width_right_filtered.x:
      offset_lane = np.interp(self.lane_width, [2.5, 2.9], [0.0, self.adjustLaneOffset]) # 차선이 좁으면 안함..
    else:
      offset_lane = np.interp(self.lane_width, [2.5, 2.9], [0.0, -self.adjustLaneOffset]) # 차선이 좁으면 안함..

    #select lane path
    lane_path_y = (path_from_left_lane + path_from_right_lane) / 2.

    use_laneless_center_adjust = False
    if use_laneless_center_adjust:
      ## 0.5초 앞의 중심을 보도록함.
      lane_path_y_center = np.interp(0.5, path_t, lane_path_y)
      path_xyz_y_center = np.interp(0.5, path_t, path_xyz[:,1])
      #lane_path_y_center = lane_path_y[0]
      #path_xyz_y_center = path_xyz[:,1][0]
      diff_center = (lane_path_y_center - path_xyz_y_center) if not self.lanefull_mode else 0.0
    else:
      diff_center = 0.0
    #print("center = {:.2f}={:.2f}-{:.2f}, lanefull={}".format(diff_center, lane_path_y_center, path_xyz_y_center, self.lanefull_mode))
    #diff_center = lane_path_y[5] - path_xyz[:,1][5] if not self.lanefull_mode else 0.0
    if offset_curve * offset_lane < 0:
      offset_total = np.clip(offset_curve + offset_lane + diff_center, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)
    else:
      offset_total = np.clip(max(offset_curve, offset_lane, key=abs) + diff_center, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)

    ## self.d_prob = 0 if lane_changing
    self.d_prob *= self.lane_change_multiplier  ## 차선변경중에는 꺼버림.
    if self.lane_change_multiplier < 0.5:
      #self.lane_offset_filtered.x = 0.0
      pass
    else:
      self.lane_offset_filtered.update(np.interp(self.d_prob, [0, 0.3], [0, offset_total]))

    ## laneless at lowspeed
    self.d_prob *= np.interp(v_ego*3.6, [5., 10.], [0.0, 1.0])

    #self.debugText = "OFFSET({:.2f}={:.2f}+{:.2f}+{:.2f}),Vc:{:.2f},dp:{:.1f},lf:{},lrw={:.1f}|{:.1f}|{:.1f}".format(
    #  self.lane_offset_filtered.x,
    #  diff_center, offset_lane, offset_curve,
    #  curve_speed,
    #  self.d_prob, self.lanefull_mode,
    #  self.lane_width_left_filtered.x, self.lane_width, self.lane_width_right_filtered.x)

    laneline_active = False
    self.d_prob_count = self.d_prob_count + 1 if self.d_prob > 0.3 else 0
    if self.lanefull_mode and self.d_prob_count > int(1 / DT_MDL):
      laneline_active = True
      safe_idxs = np.isfinite(self.ll_t)
      if safe_idxs[0]:
        lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
        path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]


    lane_center_offset = self._get_lane_center_offset(CS, laneline_active)
    position_offset = self._get_lane_position_offset(CS, laneline_active)
    # Bound the combined preference; the manual global trim does not stack unboundedly.
    lane_center_offset = float(np.clip(lane_center_offset+position_offset, -.05, .05))
    # Preserve the existing total calibration adjustment, previously added here
    # and again in LateralPlanner. Apply the new lane-only preference once.
    calibration_offset = 2.0 * (self.path_offset + self.path_offset2)
    tracking_offset = self._get_lane_tracking_offset(
      CS, laneline_active, lane_center_offset + self.lane_offset_filtered.x + calibration_offset + self.road_edge_offset)
    path_xyz[:, 1] += self.lane_offset_filtered.x + calibration_offset + lane_center_offset + tracking_offset

    self.offset_total = self.lane_offset_filtered.x + lane_center_offset + tracking_offset

    return path_xyz, laneline_active

  def calculate_plan_yaw_and_yaw_rate(self, path_xyz):
    if path_xyz.shape[0] < 3:
        # 너무 짧으면 직진 가정
        N = path_xyz.shape[0]
        return np.zeros(N), np.zeros(N)

    # x, y 추출
    x = path_xyz[:, 0]
    y = path_xyz[:, 1]

    # 모두 동일한 점인지 확인
    if np.allclose(x, x[0]) and np.allclose(y, y[0]):
        return np.zeros(len(x)), np.zeros(len(x))

    # 안전한 diff 계산
    dx = np.diff(x)
    dy = np.diff(y)
    mask = (dx == 0) & (dy == 0)
    dx[mask] = 1e-4
    dy[mask] = 0.0

    yaw = np.arctan2(dy, dx)
    yaw = np.append(yaw, yaw[-1])  # N-1 → N
    yaw = np.unwrap(yaw)

    dx_full = np.clip(np.diff(x), 1e-4, None)
    yaw_rate = np.diff(yaw) / dx_full
    yaw_rate = np.append(yaw_rate, yaw_rate[-1])
    yaw_rate = np.append(yaw_rate, 0.0)

    # NaN/Inf 방어
    if np.any(np.isnan(yaw_rate)) or np.any(np.isinf(yaw_rate)):
        yaw_rate = np.zeros_like(yaw_rate)
    if np.any(np.isnan(yaw)) or np.any(np.isinf(yaw)):
        yaw = np.zeros_like(yaw)

    return yaw, yaw_rate
