#!/usr/bin/env python3
import os
import time
import numpy as np
from openpilot.cereal import log
from opendbc.car.interfaces import ACCEL_MIN
from openpilot.selfdrive.controls.lib.congestion_follow import CongestionFollow
from openpilot.selfdrive.controls.lib.stop_approach import StopApproach, stop_approach_reference
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
# from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner
from openpilot.selfdrive.carrot.carrot_functions import XState


if __name__ == '__main__':  # generating code
  from acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
MPC_SOURCES = (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1, LongitudinalPlanSource.cruise, LongitudinalPlanSource.e2e)

X_DIM = 3
U_DIM = 1
PARAM_DIM = 8
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 5.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.
A_CHANGE_COST = 200.
A_CHANGE_COST_STARTING = 10. #30.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
# Code generation must not read device parameters; CarrotPlanner supplies the runtime value.
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0
CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 1.6
MIN_X_LEAD_FACTOR = 0.5

def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.moreRelaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.standard:
    return 1.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.moreRelaxed:
    return 2.0
  elif personality==log.LongitudinalPersonality.relaxed:
    return 1.75
  elif personality==log.LongitudinalPersonality.standard:
    return 1.45
  elif personality==log.LongitudinalPersonality.aggressive:
    return 1.25
  else:
    raise NotImplementedError("Longitudinal personality not supported")

def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)

def get_safe_obstacle_distance(v_ego, t_follow=None, comfort_brake=COMFORT_BRAKE, stop_distance=STOP_DISTANCE):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_distance

def desired_follow_distance(v_ego, v_lead, comfort_brake, stop_distance, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow, comfort_brake, stop_distance) - get_stopped_equivalence_factor(v_lead)


def recovering_lead_tau(lead, a_lead_tau):
  # Only shorten persistence when a moving radar lead's braking is easing,
  # relative closing speed is small, and its current deceleration is mild.
  # A stopped/slow lead, strong braking, or rapid closing keeps the original
  # conservative prediction.
  values = [lead.vLead, lead.vRel, lead.aLeadK, lead.jLead, a_lead_tau]
  if not lead.present or not lead.radar or not np.all(np.isfinite(values)):
    return a_lead_tau
  if lead.vLead <= 4.0 or not -0.75 < lead.aLeadK < 0.0 or lead.vRel <= -2.0 or lead.jLead <= 0.0:
    return a_lead_tau
  weight = (float(np.interp(lead.vLead, [4.0, 6.0], [0.0, 1.0])) *
            float(np.interp(lead.aLeadK, [-0.75, -0.25], [0.0, 1.0])) *
            float(np.interp(lead.vRel, [-2.0, -0.5], [0.0, 1.0])) *
            float(np.interp(lead.jLead, [0.0, 0.5], [0.0, 1.0])))
  return a_lead_tau + weight * max(0.0, _LEAD_ACCEL_TAU - a_lead_tau)


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # states
  x_ego, v_ego, a_ego = SX.sym('x_ego'), SX.sym('v_ego'), SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  a_prev = SX.sym('a_prev')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  comfort_brake = SX.sym('comfort_brake')
  stop_distance = SX.sym('stop_distance')
  model.p = vertcat(a_min, a_max, x_obstacle, a_prev, lead_t_follow, lead_danger_factor, comfort_brake, stop_distance)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model

def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  a_prev = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]
  comfort_brake = ocp.model.p[6]
  stop_distance = ocp.model.p[7]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow, comfort_brake, stop_distance)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - a_prev,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, lead_t_follow, LEAD_DANGER_FACTOR, comfort_brake, stop_distance])


  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc', dt=DT_MDL):
    self.mode = mode
    self.dt = dt
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)

    self.a_change_cost = A_CHANGE_COST
    self.j_lead = 0.0

    self.reset()
    self.source = LongitudinalPlanSource.cruise

    self.t_follow = 1.0
    self.desired_distance = 0.0
    self.lead_danger_factor = LEAD_DANGER_FACTOR

    self.e2e_x = np.zeros(13, dtype=np.float64)

    self.personality = 1
    self.enableSpeedTF = 0

  def reset(self):
    self.congestion_follow = CongestionFollow()
    self.stop_approach = StopApproach()
    self._stop_approach_active = False
    self.solver.reset()

    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N, 1))
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.j_solution = np.zeros(N)
    self.a_prev = np.array(self.a_solution)
    self.yref = np.zeros((N+1, COST_DIM))

    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.lead_loss_armed = False
    self.lead_loss_remaining = 0.0
    self.early_lead_id = None
    self.early_lead_distance = None
    self.early_lead_confirm = 0.0
    self.early_lead_decel = 0.0
    self.turn_lead_id = None
    self.turn_lead_age = 0.0
    self.turn_lead_confirm = 0.0
    self.turn_loss_remaining = 0.0
    self.turn_loss_clear_time = 0.0
    self.turn_loss_reacquire = 0.0
    self.turn_reacquire_id = None
    self.turn_loss_limit = None
    self.turn_loss_elapsed = self.turn_exit_confirm = 0.0
    self.turn_peak_angle = self.turn_peak_curvature = 0.0
    self.turn_exit_curvature = None
    self.turn_exit_side = 0
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard, jerk_factor=1.0, a_change_cost_starting=A_CHANGE_COST_STARTING):
    #jerk_factor = get_jerk_factor(personality)
    if self.mode == 'acc':
      a_change_cost = self.a_change_cost if prev_accel_constraint else a_change_cost_starting
      cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, jerk_factor * a_change_cost, jerk_factor * J_EGO_COST]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau, j_lead):
    j_lead_tau = np.interp(j_lead, [-2.0, 0.0, 2.0], [0.2, 2.0, 0.1]) # tau: 2: 2sec, 1: 4sec, 0.5: 10sec
    j_lead_traj = j_lead * np.exp(-j_lead_tau * (T_IDXS**2)/2.)
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.) + j_lead_traj
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead, j_lead):
    v_ego = self.x0[1]
    if lead is not None and lead.present:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
      if self.mode == 'acc':
        a_lead_tau = recovering_lead_tau(lead, a_lead_tau)
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = np.clip(x_lead, min_x_lead, 1e8)
    v_lead = np.clip(v_lead, 0.0, 1e8)
    a_lead = np.clip(a_lead, -10., 5.)

    if a_lead < -2.0 and j_lead > 0.5:
      a_lead = a_lead + j_lead
      a_lead = min(a_lead, -0.5)
      a_lead_tau = max(a_lead_tau, 1.5)

    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau, j_lead)
    return lead_xv, v_lead

  def set_accel_limits(self, min_a, max_a):
    # TODO this sets a max accel limit, but the minimum limit is only for cruise decel
    # needs refactor
    self.cruise_min_a = min_a
    self.max_a = max_a

  def cruise_accel_after_lead_loss(self, lead_one, lead_two, v_ego, reset_state, mode):
    # Briefly soften cruise acceleration after losing a nearby slowing lead.
    # Do not synthesize a lead or change braking/obstacle constraints.
    if reset_state or mode != 'acc' or not 1.5 <= v_ego <= 15.0:
      self.lead_loss_armed = False
      self.lead_loss_remaining = 0.0
      return self.max_a

    if lead_one.present or lead_two.present:
      self.lead_loss_remaining = 0.0
      self.lead_loss_armed = any(
        lead.present and 0.0 < lead.dRel < max(10.0, 1.8 * v_ego) and
        (lead.vRel < -0.3 or lead.aLeadK < -0.5 or lead.vLead < 2.0)
        for lead in (lead_one, lead_two))
      return self.max_a

    if self.lead_loss_armed:
      self.lead_loss_remaining = 0.8
      self.lead_loss_armed = False

    if self.lead_loss_remaining <= 0.0:
      return self.max_a

    # Hold for 0.5 s, then release over 0.3 s. Never raise the normal limit.
    cruise_accel = min(self.max_a, float(np.interp(
      self.lead_loss_remaining, [0.0, 0.3], [self.max_a, 0.2])))
    self.lead_loss_remaining = max(0.0, self.lead_loss_remaining - self.dt)
    return cruise_accel

  def cruise_accel_in_turn_after_lead_loss(self, lead_one, lead_two, v_ego, a_ego, steering_angle, normal_max, enabled,
                                          future_curvature=None, allow_resume=True):
    # Losing a selected vehicle in a tight turn does not establish a clear road.
    if not enabled or not np.all(np.isfinite((v_ego, a_ego, steering_angle, normal_max))) or not 1.0 <= v_ego <= 12.0:
      self.turn_lead_id = None
      self.turn_lead_confirm = self.turn_lead_age = 0.0
      self.turn_loss_remaining = self.turn_loss_clear_time = self.turn_loss_reacquire = 0.0
      self.turn_reacquire_id = None
      self.turn_loss_limit = None
      self.turn_loss_elapsed = self.turn_exit_confirm = 0.0
      self.turn_peak_angle = self.turn_peak_curvature = 0.0
      self.turn_exit_curvature = None
      self.turn_exit_side = 0
      return normal_max

    preview_valid = future_curvature is not None and np.isfinite(future_curvature)
    if preview_valid:
      if self.turn_exit_curvature is None:
        self.turn_exit_curvature = abs(future_curvature)
      else:
        self.turn_exit_curvature += self.dt/(.15+self.dt) * (abs(future_curvature)-self.turn_exit_curvature)
    else:
      self.turn_exit_curvature = None
    turning = abs(steering_angle) >= 45.0
    reliable = [lead for lead in (lead_one, lead_two) if
                lead.present and lead.radar and lead.radarTrackId >= 0 and lead.modelProb >= .6 and
                np.all(np.isfinite((lead.dRel, lead.vLead, lead.dPath))) and
                3.0 < lead.dRel < max(20.0, 4.0*v_ego) and lead.vLead > 1.0 and abs(lead.dPath) < 3.5]
    matching = next((lead for lead in reliable if lead.radarTrackId == self.turn_lead_id), None)
    if self.turn_loss_limit is None:
      if turning and reliable:
        selected = matching if matching is not None else reliable[0]
        if matching is None:
          self.turn_peak_angle = self.turn_peak_curvature = 0.0
          self.turn_exit_curvature = abs(future_curvature) if preview_valid else None
          self.turn_exit_side = int(np.sign(steering_angle))
        self.turn_lead_confirm = min(1., self.turn_lead_confirm+self.dt) if matching is not None else self.dt
        self.turn_lead_id = selected.radarTrackId
        self.turn_lead_age = 0.0
      elif turning and self.turn_lead_id is not None and self.turn_lead_confirm >= .3 and self.turn_lead_age < 1.0:
        self.turn_loss_remaining = 4.0
        self.turn_loss_elapsed = self.turn_exit_confirm = 0.0
        self.turn_loss_clear_time = self.turn_loss_reacquire = 0.0
        # Start from measured acceleration, rather than abruptly demanding braking.
        self.turn_loss_limit = min(max(0., normal_max), max(0., a_ego))
      else:
        self.turn_lead_age += self.dt
        if not turning or self.turn_lead_age >= 1.0:
          self.turn_lead_id = None
          self.turn_lead_confirm = 0.0

    if self.turn_lead_id is not None:
      self.turn_peak_angle = max(self.turn_peak_angle, abs(steering_angle))
      if preview_valid:
        self.turn_peak_curvature = max(self.turn_peak_curvature, self.turn_exit_curvature)
    if self.turn_loss_limit is None:
      return normal_max
    # A departing lead can already be beyond the close-lead arming range.
    # Reacquire it only in a narrow path corridor while this turn is opening.
    departing = [lead for lead in (lead_one, lead_two) if
                 allow_resume and lead.present and lead.radar and lead.radarTrackId >= 0 and lead.modelProb >= .85 and
                 np.all(np.isfinite((lead.dRel, lead.vLead, lead.dPath))) and
                 15. < lead.dRel < min(50., max(35., 8.*v_ego)) and
                 lead.vLead-v_ego > 1. and abs(lead.dPath) < 1. and preview_valid and
                 self.turn_exit_side == int(np.sign(steering_angle)) and steering_angle*future_curvature <= 0. and
                 abs(future_curvature)*v_ego**2 <= 1.6 and
                 self.turn_peak_angle >= 60. and abs(steering_angle) < .85*self.turn_peak_angle and
                 self.turn_exit_curvature < .85*self.turn_peak_curvature]
    recovered = next((lead for lead in reliable if lead.modelProb >= .75 and abs(lead.dPath) < 2.0), None)
    departing_recovery = recovered is None and bool(departing)
    if departing_recovery:
      recovered = departing[0]
    if recovered is not None:
      self.turn_loss_reacquire = self.turn_loss_reacquire+self.dt if recovered.radarTrackId == self.turn_reacquire_id else self.dt
      self.turn_reacquire_id = recovered.radarTrackId
    else:
      self.turn_loss_reacquire = 0.0
      self.turn_reacquire_id = None
    if self.turn_loss_reacquire >= (.35 if departing_recovery else .6):
      # A new front-radar ID can replace the corner track after stable confirmation.
      self.turn_lead_id = recovered.radarTrackId
      self.turn_lead_confirm = 1.0
      self.turn_lead_age = 0.0
      self.turn_loss_remaining = 0.0
    elif matching is not None:
      self.turn_lead_age = 0.0
    else:
      self.turn_lead_age += self.dt
      if turning and self.turn_loss_remaining == 0.0 and self.turn_lead_age < 1.0 and self.turn_lead_confirm >= .3:
        # Re-arm if a recently recovered lead disappears again during release.
        self.turn_loss_remaining = 4.0
        self.turn_loss_elapsed = self.turn_exit_confirm = 0.0
        self.turn_peak_angle = abs(steering_angle)
        self.turn_peak_curvature = self.turn_exit_curvature if preview_valid else 0.0
        self.turn_exit_side = int(np.sign(steering_angle))
        self.turn_loss_clear_time = 0.0
    if self.turn_loss_remaining > 0.0:
      self.turn_loss_remaining = max(0., self.turn_loss_remaining-self.dt)
      self.turn_loss_elapsed += self.dt
      self.turn_loss_clear_time = self.turn_loss_clear_time+self.dt if abs(steering_angle) < 30. else 0.
      if self.turn_loss_clear_time >= .6:
        self.turn_loss_remaining = 0.0
    target = 0.0 if self.turn_loss_remaining > 0.0 else max(0., normal_max)
    if self.turn_loss_remaining > 0.0:
      # A turn can start opening well before the steering wheel is nearly straight.
      # Keep the initial blind hold, then permit only a small positive ceiling.
      same_turn = (self.turn_exit_side == int(np.sign(steering_angle)) and
                   preview_valid and steering_angle*future_curvature <= 0.)
      # A distant, receding track may flicker between radar and vision. It
      # can allow the small opening cap, but cannot clear the blind hold.
      clear_for_creep = all(not lead.present or (
        np.all(np.isfinite((lead.dRel, lead.vLead, lead.dPath))) and
        lead.dRel > max(18., 3.*v_ego) and lead.vLead-v_ego > 1. and abs(lead.dPath) < 2.)
        for lead in (lead_one, lead_two))
      opening = (allow_resume and same_turn and self.turn_loss_elapsed >= .9 and
                 clear_for_creep and
                 self.turn_peak_angle >= 60. and self.turn_peak_curvature >= .005 and
                 abs(steering_angle) <= .85*self.turn_peak_angle and
                 self.turn_peak_angle-abs(steering_angle) >= 15. and
                 self.turn_exit_curvature <= .8*self.turn_peak_curvature and
                 abs(future_curvature)*v_ego**2 <= 1.6 and
                 abs(future_curvature) <= .85*self.turn_peak_curvature and
                 abs(future_curvature) <= self.turn_exit_curvature+.0015)
      self.turn_exit_confirm = self.turn_exit_confirm+self.dt if opening else 0.0
      if self.turn_exit_confirm >= .35:
        unwind = min(1.-abs(steering_angle)/self.turn_peak_angle,
                     1.-self.turn_exit_curvature/self.turn_peak_curvature)
        target = float(np.interp(unwind, [.2, .5], [.15, .3]))
    else:
      self.turn_exit_confirm = 0.0
    self.turn_loss_limit += float(np.clip(target-self.turn_loss_limit, -.8*self.dt, .5*self.dt))
    cap = min(normal_max, self.turn_loss_limit)
    if self.turn_loss_remaining == 0.0 and self.turn_loss_limit >= max(0., normal_max)-1e-6:
      self.turn_loss_limit = None
      self.turn_lead_id = None
      self.turn_lead_confirm = 0.0
    return cap

  def cruise_accel_for_approaching_lead(self, lead, v_ego, desired_distance, normal_max, enabled):
    # Shape only the soft cruise target. Lead obstacles and brake limits stay intact.
    values = (v_ego, desired_distance, lead.dRel, lead.vRel, lead.vLead, lead.aLeadK, lead.modelProb)
    if (not enabled or not lead.present or not lead.radar or lead.modelProb < .8 or
        not np.all(np.isfinite(values)) or not 8.0 <= v_ego <= 25.0 or lead.vLead < 3.0 or
        abs((lead.vLead-v_ego)-lead.vRel) > 1.5):
      self.early_lead_id = self.early_lead_distance = None
      self.early_lead_confirm = self.early_lead_decel = 0.0
      return normal_max

    stable = (self.early_lead_id == lead.radarTrackId and self.early_lead_distance is not None and
              abs(lead.dRel-self.early_lead_distance) <= max(3.0, v_ego*.3))
    self.early_lead_id, self.early_lead_distance = lead.radarTrackId, lead.dRel
    if not stable:
      self.early_lead_confirm = self.early_lead_decel = 0.0
      return normal_max

    closing = -lead.vRel
    room = lead.dRel-desired_distance
    approaching = (1.0 < closing < 6.0 and lead.aLeadK < .3 and
                   max(1.8*v_ego, desired_distance+5.0) < lead.dRel < 4.0*v_ego)
    self.early_lead_confirm = min(1., self.early_lead_confirm+self.dt) if approaching else 0.
    # Remove a persistent speed difference while there is room, before hard braking.
    target = float(np.clip(.6*closing**2/(2.*max(room, 5.)), .12, .35)) if self.early_lead_confirm >= .5 else 0.
    self.early_lead_decel += float(np.clip(target-self.early_lead_decel, -.2*self.dt, .15*self.dt))
    if self.early_lead_decel < 1e-6:
      self.early_lead_decel = 0.0
    return min(normal_max, -self.early_lead_decel) if self.early_lead_decel else normal_max

  def update(self, carrot, reset_state, radarstate, v_cruise, x, v, a, j, personality=log.LongitudinalPersonality.standard,
             prev_accel_constraint=True, future_curvature=None, allow_turn_resume=False, allow_congestion=False, cutin_accel_limit=None,
             turn_exit_strength=0., allow_stop_approach=False, radar_stamp=0., observation_time=0.):
    v_ego = self.x0[1]
    a_ego = self.x0[2]
    t_follow = carrot.get_T_FOLLOW(personality, v_ego, a_ego)
    self.status = radarstate.leadOne.present or radarstate.leadTwo.present

    if radarstate.leadOne.present:
      j_lead = radarstate.leadOne.jLead
      self.j_lead = j_lead * 0.1 + self.j_lead * 0.9
    else:
      self.j_lead = 0.0

    lead_xv_0, lead_v_0 = self.process_lead(radarstate.leadOne, np.clip(self.j_lead * carrot.j_lead_factor, -1.0, 1.0))
    lead_xv_1, _ = self.process_lead(radarstate.leadTwo, 0.0)

    mode = self.mode
    comfort_brake = carrot.comfort_brake
    stop_distance = carrot.stop_distance
    a_change_cost2 = carrot.a_change_cost2
    lead_danger_factor2 = carrot.lead_danger_factor2
    
    if mode == 'blended':
      stop_x = 1000.0
    else:
      v_cruise, stop_x, mode = carrot.v_cruise, carrot.stop_dist, carrot.mode
      desired_distance = desired_follow_distance(v_ego, lead_v_0, comfort_brake, stop_distance, t_follow)
      t_follow = carrot.dynamic_t_follow(t_follow, radarstate.leadOne, desired_distance, self.a_prev)

    jam_follow, jam_accel = self.congestion_follow.update(
      radarstate.leadOne, v_ego, self.max_a, t_follow,
      allow_congestion and mode == 'acc' and carrot.desireState <= .1 and
      carrot.xState not in [XState.e2eStop, XState.e2eStopped], self.dt)
    if not reset_state:
      t_follow = jam_follow

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])
    
    self.desired_distance = desired_follow_distance(v_ego, lead_v_0, comfort_brake, stop_distance, t_follow)

    self.params[:,0] = ACCEL_MIN if not reset_state else a_ego
    # negative accel constraint causes problems because negative speed is not allowed
    self.params[:,1] = max(0.0, self.max_a if not reset_state else a_ego)

    cruise_max_a = self.cruise_accel_after_lead_loss(
      radarstate.leadOne, radarstate.leadTwo, v_ego, reset_state, mode)
    if mode == 'acc' and not reset_state:
      cruise_max_a = min(cruise_max_a, carrot.signal_follow.cruise_accel(self.max_a))
      if carrot.signal_follow.strength > 0:
        # Also bound the solver's acceleration: a nearby lead must not undo the signal cap.
        self.params[:,1] = np.minimum(self.params[:,1], max(0.0, cruise_max_a))

    cruise_max_a = self.cruise_accel_for_approaching_lead(
      radarstate.leadOne, v_ego, self.desired_distance, cruise_max_a,
      mode == 'acc' and not reset_state and carrot.desireState <= .1 and
      carrot.xState not in [XState.e2eStop, XState.e2eStopped])

    cruise_max_a = self.cruise_accel_in_turn_after_lead_loss(
      radarstate.leadOne, radarstate.leadTwo, v_ego, a_ego, carrot.steering_angle, cruise_max_a,
      mode == 'acc' and not reset_state and carrot.desireState <= .1,
      future_curvature=future_curvature,
      allow_resume=(allow_turn_resume and carrot.xState not in [XState.e2eStop, XState.e2eStopped] and
                    carrot.signal_follow.strength <= 0.))
    if self.turn_loss_limit is not None:
      # Also constrain acceleration if another lead/cruise objective becomes dominant.
      self.params[:,1] = np.minimum(self.params[:,1], max(0., cruise_max_a))

    if not reset_state and jam_accel < self.max_a:
      cruise_max_a = min(cruise_max_a, jam_accel)
      self.params[:,1] = np.minimum(self.params[:,1], max(0., jam_accel))

    if mode == 'acc' and not reset_state and cutin_accel_limit is not None and np.isfinite(cutin_accel_limit):
      cruise_max_a = min(cruise_max_a, cutin_accel_limit)
      self.params[:,1] = np.minimum(self.params[:,1], max(0., cutin_accel_limit))

    # Update in ACC mode or ACC/e2e blend
    if mode == 'acc':
      #self.params[:,5] = LEAD_DANGER_FACTOR
      # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
      # when the leads are no factor.
      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      # TODO does this make sense when max_a is negative?
      v_upper = v_ego + (T_IDXS * max(self.cruise_min_a, cruise_max_a) * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow, comfort_brake, stop_distance)

      adjust_dist = carrot.trafficStopDistanceAdjust if v_ego > 0.1 else -2.0
      if 50 < stop_x + adjust_dist < cruise_obstacle[0]:
        stop_x = cruise_obstacle[0] - adjust_dist
      x2 = stop_x * np.ones(N+1) + adjust_dist

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle, x2])
      min_idx = int(np.argmin(x_obstacles[0]))
      self.source = MPC_SOURCES[min_idx]

      if v_cruise == 0 and self.source == LongitudinalPlanSource.cruise:
        self.params[:,0] = - carrot.autoNaviSpeedDecelRate
      #elif self.source in [LongitudinalPlanSource.cruise, LongitudinalPlanSource.e2e]:
      #  self.params[:,0] = - COMFORT_BRAKE

      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))
      e2ex = (x[N] + 5.0) * np.ones(N+1)
      x = (x[N] + 5.0) * np.ones(N+1)      

      # These are not used in ACC mode
      v[:], a[:], j[:] = 0.0, 0.0, 0.0

      if radarstate.leadOne.present:
        self.a_change_cost = float(np.interp(abs(self.j_lead), [0.3, 2.0], [A_CHANGE_COST, a_change_cost2]))
      else:
        self.a_change_cost = A_CHANGE_COST

      #safe_distance = lead_0_obstacle[0] - get_safe_obstacle_distance(v_ego, comfort_brake, stop_distance)
      self.lead_danger_factor = lead_danger_factor2 #np.interp(safe_distance, [-30.0, 0.0], [0.9, LEAD_DANGER_FACTOR]) # 이걸적용하니, 사고방지턱 감속시 너무 급정거하는것 같음.
      self.params[:,5] = self.lead_danger_factor
      
    elif mode == 'blended':
      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))
      e2ex = (x[N] + 5.0) * np.ones(N+1)

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = LongitudinalPlanSource.e2e if x_and_cruise[1,0] < x_and_cruise[1,1] else LongitudinalPlanSource.cruise

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    # Shift only the soft distance objective during a confirmed easing approach.
    # Keep t_follow, stop_distance, obstacle predictions and danger constraints intact.
    approach_bias = self.stop_approach.update(
      radarstate.leadOne, radarstate.leadTwo, v_ego, a_ego, self.desired_distance, stop_distance,
      radar_stamp, observation_time,
      enabled=(allow_stop_approach and mode == 'acc' and not reset_state and
               carrot.xState not in [XState.e2eStop, XState.e2eStopped, XState.e2ePrepare] and
               carrot.signal_follow.strength <= 0. and
               self.turn_loss_limit is None and cutin_accel_limit is None and jam_accel >= self.max_a-1e-6))
    self.yref[:,0] = 0.
    if approach_bias > 0.:
      self.yref[:,0] = stop_approach_reference(
        approach_bias, self.v_solution, lead_xv_0[:,1], lead_0_obstacle,
        np.full(N+1, 1e6) if self.stop_approach.same_lead(radarstate.leadOne, radarstate.leadTwo) else lead_1_obstacle,
        cruise_obstacle, x2)
    approach_active = bool(np.any(self.yref[:,0] < 0.))
    if approach_active != self._stop_approach_active:
      cloudlog.event('low_speed_stop_approach', active=approach_active,
                     bias_m=float(approach_bias), v_ego=float(v_ego),
                     distance=float(radarstate.leadOne.dRel), lead_id=int(radarstate.leadOne.radarTrackId))
      self._stop_approach_active = approach_active

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.a_prev)
    self.params[:,4] = t_follow
    self.params[:,6] = comfort_brake
    self.params[:,7] = stop_distance

    self.e2e_x = e2ex[:]

    self.t_follow = t_follow

    # Apply this cycle's follow/jerk factor and lead acceleration-change cost.
    # Only soften cruise recovery when none of the existing safety caps is binding.
    # Lead obstacles, stop objectives, acceleration bounds and braking costs remain.
    exit_strength = float(np.clip(turn_exit_strength, 0., 1.)) if np.isfinite(turn_exit_strength) else 0.
    exit_strength = exit_strength if (mode == 'acc' and not reset_state and
                                     self.turn_loss_limit is None and cruise_max_a >= self.max_a-1e-6) else 0.
    self.set_weights(prev_accel_constraint, personality=personality,
                     jerk_factor=carrot.jerk_factor_apply*(1.-.35*exit_strength),
                     a_change_cost_starting=carrot.aChangeCostStarting)
    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # Check if it got within lead comfort range
    # TODO This should be done cleaner
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow, comfort_brake, stop_distance))- self.x_sol[:,0] < 0.0):
        self.source = LongitudinalPlanSource.lead0
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow, comfort_brake, stop_distance))- self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = LongitudinalPlanSource.lead1

  def run(self):
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.a_prev = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
