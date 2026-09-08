#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource, STOP_DISTANCE, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.vn_follow import effective_t_follow, vn_min_follow_m
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# VF8/VF9: when model looks like a red-light stop, request ~3% more deceleration
# during the approach, then fade the boost near stop so the final lead/stop gap
# stays at the normal VinFast standstill distance (not inflated by hard braking).
# Thresholds still need committed stop intent so green/coast approaches do not
# brake early, but they must trigger during the approach: while this reads False
# the softening below is what is applied instead.
VF_REDLIGHT_FINGERPRINTS = {"VINFAST_VF8", "VINFAST_VF9"}
VF_REDLIGHT_ACCEL_SCALE = 1.03
VF_REDLIGHT_BRAKE_PROB = 0.60
# The model path is "short" relative to where a free-driving prediction would end,
# so the stop is recognized during the approach instead of in the last few meters.
VF_REDLIGHT_PATH_SHORT_FRAC = 0.55
VF_REDLIGHT_PATH_X_MAX = 60.0      # [m] cap on the short-path threshold
VF_REDLIGHT_PATH_X_MIN = 20.0      # [m] floor on the short-path threshold
VF_REDLIGHT_LEAD_DREL_MIN = 18.0   # [m] a lead past this is not ordinary following
VF_REDLIGHT_LEAD_MOVING_V = 2.0    # [m/s] a lead slower than this is part of the stop
VF_REDLIGHT_MIN_VEGO = 0.8         # [m/s]
# Full +3% above this speed; taper to no boost by the low end (gap control).
VF_REDLIGHT_BOOST_FULL_KPH = 25.0
VF_REDLIGHT_BOOST_FADE_KPH = 8.0
# Debounce the heuristic so the applied scale cannot chatter on noisy model probs.
VF_REDLIGHT_ENGAGE_FRAMES = 3   # ~0.15 s of agreement before committing
VF_REDLIGHT_HOLD_FRAMES = 20    # ~1.0 s of hold once committed

# VF8/VF9: soften the onset of braking so approaches don't start too early, then
# fade the softening back to full authority as the request grows. The fade is what
# keeps a real stop from arriving 22% deeper than the planner intended.
VF_MILD_DECEL_SCALE = 0.93
VF_MILD_SOFTEN_START = 0.4  # [m/s²] full softening below this decel
VF_MILD_DECEL_FLOOR = -1.6  # [m/s²] no softening at or beyond this decel

# Shared VF6–VF9: 4 m sits on a moto at a red light. Sit at stock 6 m even on
# aggressive, and give standard/relaxed extra room (positive MPC adjust).
VF_STOP_LEAD_GAP_M = {
  int(log.LongitudinalPersonality.aggressive): 6.0,
  int(log.LongitudinalPersonality.standard): 7.0,
  int(log.LongitudinalPersonality.relaxed): 8.0,
}
VF67_STOP_LEAD_GAP_M = VF_STOP_LEAD_GAP_M
VF67_STOP_FINGERPRINTS = {"VINFAST_VF6", "VINFAST_VF7", "VINFAST_VF8", "VINFAST_VF8_ECO", "VINFAST_VF9"}

# Set False to restore personality-only T_FOLLOW (no Thông tư 38/2024 floor).
VN_LEGAL_FOLLOW = True


def vf_stop_lead_gap_m(personality, fingerprint=None) -> float:
  try:
    key = int(personality)
  except (TypeError, ValueError):
    key = int(log.LongitudinalPersonality.standard)
  table = VF67_STOP_LEAD_GAP_M if fingerprint in VF67_STOP_FINGERPRINTS else VF_STOP_LEAD_GAP_M
  return table.get(key, table[int(log.LongitudinalPersonality.standard)])


def vf_stop_lead_adjust_m(personality, fingerprint=None) -> float:
  """How far to shift a stopped lead so the standstill gap matches the platform.

  The MPC subtracts this from the obstacle position and settles where
  `obstacle - x_ego == STOP_DISTANCE`, so the gap it holds is
  `STOP_DISTANCE + adjust`. The sign therefore follows the extra room wanted
  beyond STOP_DISTANCE: 0 m on aggressive, +2 m on relaxed (VF6–VF9 6/7/8 m).
  """
  return vf_stop_lead_gap_m(personality, fingerprint) - STOP_DISTANCE


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def is_vf_red_light_slowdown(sm, CP) -> bool:
  """Heuristic red-light stop intent for VF8/VF9 (no dedicated cereal flag)."""
  if CP.carFingerprint not in VF_REDLIGHT_FINGERPRINTS:
    return False

  CS = sm['carState']
  if CS.standstill or CS.vEgo < VF_REDLIGHT_MIN_VEGO:
    return False

  model = sm['modelV2']
  action = model.action
  lead = sm['radarState'].leadOne
  # A stopped or crawling lead is part of the stop (a queue at a light), not an
  # ordinary follow target, so it must not disqualify the committed-stop path.
  ordinary_following = (lead.status and float(lead.dRel) <= VF_REDLIGHT_LEAD_DREL_MIN
                        and float(lead.vLead) > VF_REDLIGHT_LEAD_MOVING_V)

  brake_probs = model.meta.disengagePredictions.brakePressProbs
  brake_prob = float(brake_probs[1]) if len(brake_probs) > 1 else 0.0
  desired_a = float(action.desiredAcceleration)
  should_stop = bool(action.shouldStop)

  model_x = model.position.x
  free_path = CS.vEgo * ModelConstants.T_IDXS[-1] * VF_REDLIGHT_PATH_SHORT_FRAC
  path_limit = np.clip(free_path, VF_REDLIGHT_PATH_X_MIN, VF_REDLIGHT_PATH_X_MAX)
  path_short = len(model_x) > 0 and float(model_x[-1]) < path_limit

  # Red-light-like: committed stop intent with a path much shorter than free driving.
  # Keep thresholds high so green/coast approaches do not brake early.
  if ordinary_following:
    return False
  if should_stop and desired_a < -0.5 and path_short:
    return True
  if brake_prob > VF_REDLIGHT_BRAKE_PROB and desired_a < -0.5 and path_short:
    return True
  if path_short and desired_a < -0.7 and brake_prob > 0.45:
    return True
  return False


def vf_red_light_accel_scale(v_ego: float) -> float:
  """Speed-dependent red-light decel scale.

  Full +3% above ``VF_REDLIGHT_BOOST_FULL_KPH``, linearly fades to 1.0 by
  ``VF_REDLIGHT_BOOST_FADE_KPH`` so the final standstill gap matches normal ACC.
  """
  v_kph = max(float(v_ego), 0.0) * 3.6
  if v_kph >= VF_REDLIGHT_BOOST_FULL_KPH:
    return VF_REDLIGHT_ACCEL_SCALE
  if v_kph <= VF_REDLIGHT_BOOST_FADE_KPH:
    return 1.0
  # Linear fade from full scale → 1.0 between full and fade speeds.
  x = (v_kph - VF_REDLIGHT_BOOST_FADE_KPH) / (VF_REDLIGHT_BOOST_FULL_KPH - VF_REDLIGHT_BOOST_FADE_KPH)
  return 1.0 + (VF_REDLIGHT_ACCEL_SCALE - 1.0) * x


def vf_mild_decel_scale(a_target: float) -> float:
  """Soften the first bit of braking, fading to full authority by the floor.

  There is no step at the floor: a request just under it is scaled almost as
  little as one just past it.
  """
  return float(np.interp(-a_target, [VF_MILD_SOFTEN_START, -VF_MILD_DECEL_FLOOR],
                         [VF_MILD_DECEL_SCALE, 1.0]))


def apply_vf_late_brake(sm, CP, a_target: float, red_light: bool) -> float:
  """VF8/VF9: delay the onset of braking; keep committed stops authoritative."""
  if CP.carFingerprint not in VF_REDLIGHT_FINGERPRINTS or a_target >= 0.0:
    return a_target

  if red_light:
    return float(a_target * vf_red_light_accel_scale(sm['carState'].vEgo))

  return float(a_target * vf_mild_decel_scale(a_target))


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    # VinFast standstill: VF6–VF9 6/7/8 m from personality.
    # Tests can pin vf_stop_lead_adjust_override.
    self.vf_stop_lead_adjust_override = None
    self.mpc.stop_lead_obstacle_adjust_m = (
      vf_stop_lead_adjust_m(log.LongitudinalPersonality.standard, CP.carFingerprint) if CP.brand == "vinfast" else 0.0
    )
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False
    self.vf_redlight_count = 0
    self.vf_redlight_hold = 0
    self.vn_follow_m = None

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def vf_red_light_committed(self, sm) -> bool:
    """Debounced red-light stop intent, held briefly so the scale cannot chatter."""
    if sm['carState'].standstill:
      self.vf_redlight_count = 0
      self.vf_redlight_hold = 0
      return False

    if is_vf_red_light_slowdown(sm, self.CP):
      self.vf_redlight_count = min(self.vf_redlight_count + 1, VF_REDLIGHT_ENGAGE_FRAMES)
    else:
      self.vf_redlight_count = 0

    if self.vf_redlight_count >= VF_REDLIGHT_ENGAGE_FRAMES:
      self.vf_redlight_hold = VF_REDLIGHT_HOLD_FRAMES
    else:
      self.vf_redlight_hold = max(0, self.vf_redlight_hold - 1)

    return self.vf_redlight_hold > 0

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # Get new v_cruise and a_desired from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.a_desired = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.a_desired, v_cruise)

    if force_slow_decel:
      v_cruise = 0.0

    t_follow = None
    if self.CP.brand == "vinfast":
      if self.vf_stop_lead_adjust_override is not None:
        self.mpc.stop_lead_obstacle_adjust_m = float(self.vf_stop_lead_adjust_override)
      else:
        self.mpc.stop_lead_obstacle_adjust_m = vf_stop_lead_adjust_m(
          sm['selfdriveState'].personality, self.CP.carFingerprint)
      if VN_LEGAL_FOLLOW:
        # Highway floor: max(personality T_FOLLOW, legal 35/55/70/100 m). No-op below 60 km/h.
        self.vn_follow_m = vn_min_follow_m(v_ego, self.vn_follow_m)
        t_follow = effective_t_follow(v_ego, get_T_FOLLOW(sm['selfdriveState'].personality), self.vn_follow_m)
      else:
        self.vn_follow_m = None
    else:
      self.vn_follow_m = None

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality, t_follow=t_follow)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if self.is_e2e(sm):
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      # VN floor: if closer than the legal gap, do not let e2e coast/accel inside it.
      lead = sm['radarState'].leadOne
      vn_gap = self.vn_follow_m
      if (vn_gap is not None and lead.status and float(lead.dRel) < vn_gap
          and output_a_target_e2e > output_a_target_mpc):
        output_a_target = output_a_target_mpc
      elif output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    # VF8/VF9: delay the onset of braking; small boost only on committed red-light stops.
    output_a_target = apply_vf_late_brake(sm, self.CP, output_a_target, self.vf_red_light_committed(sm))

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
