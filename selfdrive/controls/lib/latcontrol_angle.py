import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from opendbc.car.vinfast.values import is_vf6_safety_platform

# TODO This is speed dependent
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees
VINFAST_PI_ENABLED = True

VINFAST_ANGLE_KP_BP = [0.0, 3.5, 5.56, 8.33, 11.11, 55.56]  # m/s breakpoints
VINFAST_ANGLE_KP_VF8 = [0.2, 0.3, 0.5, 0.6, 0.7, 0.8]
# VINFAST_ANGLE_KP_VF8 = [0.5, 0.6, 0.7, 0.8, 0.9, 1]
VINFAST_ANGLE_KP_VF9 = [0.8, 0.9, 0.95, 1.0, 1.1, 1.3]
VINFAST_ANGLE_KP_VF6 = [1.15, 1.25, 1.35, 1.45, 1.55, 1.7]
VINFAST_ANGLE_KI_VF8 = 0.005
VINFAST_ANGLE_KI_VF9 = 0.035
VINFAST_ANGLE_KI_VF6 = 0.042
VINFAST_ANGLE_KD = 0.0
VINFAST_ANGLE_KD_VF6 = 0.06  # Light D to damp small-angle oscillation without killing mid-angle response

HIGH_ANGLE_START_DEG = 50.0
HIGH_ANGLE_END_DEG = 470.0
HIGH_ANGLE_KP_SCALE_MIN = 0.45
HIGH_ANGLE_END_DEG_VF6 = 180.0  # VF6 OEM current max angle @ 0 km/h
HIGH_ANGLE_KP_SCALE_MIN_VF6 = 0.7
MAX_PI_CORR_RATE_DEG_PER_CYCLE = 0.35
MAX_PI_CORR_RATE_VF6_DEG_PER_CYCLE = 0.75
VINFAST_MAX_ANGLE_CORR_VF6 = 22.0
VINFAST_MAX_ANGLE_CORR_DEFAULT = 15.0

# VF6: damp hunting near straight driving only; keep full authority from ~10-25 deg
VF6_SMALL_ANGLE_DES_DEG = 8.0
VF6_SMALL_ANGLE_ERR_DEADBAND_DEG = 0.18
VF6_SMALL_ANGLE_MAX_CORR_RATE_DEG_PER_CYCLE = 0.22
VF6_SMALL_ANGLE_I_BLEED = 0.9
VF6_SMALL_CORR_ZERO_DEG = 0.04
VF6_SMALL_CENTER_DECAY = 0.88
VF6_MID_ANGLE_START_DEG = 10.0
VF6_MID_ANGLE_END_DEG = 26.0
VF6_MID_ANGLE_PI_BOOST = 1.28

# VF9 center stabilization (unchanged)
VF9_SMALL_ANGLE_DES_DEG = 12.0
VF9_SMALL_ANGLE_ERR_DEADBAND_DEG = 0.30
VF9_SMALL_ANGLE_MAX_CORR_RATE_DEG_PER_CYCLE = 0.12
VF9_SMALL_ANGLE_I_BLEED = 0.94
VF9_SMALL_CORR_ZERO_DEG = 0.06
VF9_SMALL_CENTER_DECAY = 0.90


class LatControlAngle(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand == "tesla"
    self.steer_angle_saturation_threshold = 20.0 if CP.brand == "vinfast" else STEER_ANGLE_SATURATION_THRESHOLD

    if CP.brand == "vinfast" and VINFAST_PI_ENABLED:
      self.vinfast_car_fingerprint = getattr(CP, "carFingerprint", "")

      if is_vf6_safety_platform(self.vinfast_car_fingerprint):
        vinfast_kp = [VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF6]
        vinfast_ki = VINFAST_ANGLE_KI_VF6
        vinfast_kd = VINFAST_ANGLE_KD_VF6
        max_angle_correction = VINFAST_MAX_ANGLE_CORR_VF6
      elif self.vinfast_car_fingerprint == "VINFAST_VF9":
        vinfast_kp = [VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF9]
        vinfast_ki = VINFAST_ANGLE_KI_VF9
        vinfast_kd = VINFAST_ANGLE_KD
        max_angle_correction = VINFAST_MAX_ANGLE_CORR_DEFAULT
      elif self.vinfast_car_fingerprint == "VINFAST_VF8":
        vinfast_kp = [VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF8]
        vinfast_ki = VINFAST_ANGLE_KI_VF8
        vinfast_kd = VINFAST_ANGLE_KD
        max_angle_correction = VINFAST_MAX_ANGLE_CORR_DEFAULT
      else:
        vinfast_kp = [VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF8]
        vinfast_ki = VINFAST_ANGLE_KI_VF8
        vinfast_kd = VINFAST_ANGLE_KD
        max_angle_correction = VINFAST_MAX_ANGLE_CORR_DEFAULT

      self.pid = PIDController(
        vinfast_kp,
        vinfast_ki,
        vinfast_kd,
        pos_limit=max_angle_correction,
        neg_limit=-max_angle_correction,
        rate=100,
      )
      self.prev_angle_error = 0.0
      self.prev_angle_correction = 0.0
    else:
      self.pid = None
      self.prev_angle_error = 0.0
      self.prev_angle_correction = 0.0
      self.vinfast_car_fingerprint = None

  def reset(self):
    if self.pid is not None:
      self.pid.reset()
    self.prev_angle_error = 0.0
    self.prev_angle_correction = 0.0
    super().reset()

  def _vf6_mid_angle_boost(self, abs_des_ff: float) -> float:
    if abs_des_ff < VF6_MID_ANGLE_START_DEG or abs_des_ff > VF6_MID_ANGLE_END_DEG:
      return 1.0
    mid = 0.5 * (VF6_MID_ANGLE_START_DEG + VF6_MID_ANGLE_END_DEG)
    half_width = 0.5 * (VF6_MID_ANGLE_END_DEG - VF6_MID_ANGLE_START_DEG)
    t = 1.0 - abs(abs_des_ff - mid) / half_width
    return 1.0 + (VF6_MID_ANGLE_PI_BOOST - 1.0) * max(0.0, t)

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    _ = lat_delay
    angle_log = log.ControlsState.LateralAngleState.new_message()

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
      if self.pid is not None:
        self.pid.reset()
    else:
      angle_log.active = True
      angle_steers_des_ff = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des_ff += params.angleOffsetDeg

      if self.pid is not None:
        angle_error = angle_steers_des_ff - CS.steeringAngleDeg
        use_vf6_tune = is_vf6_safety_platform(self.vinfast_car_fingerprint)
        use_vf9_tune = self.vinfast_car_fingerprint == "VINFAST_VF9"
        abs_des_ff = abs(angle_steers_des_ff)

        if use_vf6_tune and abs_des_ff < VF6_SMALL_ANGLE_DES_DEG:
          if abs(angle_error) < VF6_SMALL_ANGLE_ERR_DEADBAND_DEG:
            angle_error = 0.0
          if hasattr(self.pid, 'i') and abs(angle_error) < (VF6_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.5):
            self.pid.i *= VF6_SMALL_ANGLE_I_BLEED
        elif use_vf9_tune and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG:
          if abs(angle_error) < VF9_SMALL_ANGLE_ERR_DEADBAND_DEG:
            angle_error = 0.0
          if hasattr(self.pid, 'i') and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.5):
            self.pid.i *= VF9_SMALL_ANGLE_I_BLEED

        error_rate = (angle_error - self.prev_angle_error) / self.dt if use_vf6_tune else 0.0

        if self.prev_angle_error * angle_error < 0:
          if hasattr(self.pid, 'i'):
            self.pid.i *= 0.5 if use_vf6_tune else 0.4

        self.prev_angle_error = angle_error

        freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

        raw_angle_correction = self.pid.update(
          error=angle_error,
          error_rate=error_rate,
          speed=CS.vEgo,
          feedforward=0.0,
          freeze_integrator=freeze_integrator,
        )

        high_angle_end = HIGH_ANGLE_END_DEG_VF6 if use_vf6_tune else HIGH_ANGLE_END_DEG
        high_angle_scale_min = HIGH_ANGLE_KP_SCALE_MIN_VF6 if use_vf6_tune else HIGH_ANGLE_KP_SCALE_MIN
        if abs_des_ff <= HIGH_ANGLE_START_DEG:
          kp_scale = 1.0
        elif abs_des_ff >= high_angle_end:
          kp_scale = high_angle_scale_min
        else:
          t = (abs_des_ff - HIGH_ANGLE_START_DEG) / (high_angle_end - HIGH_ANGLE_START_DEG)
          kp_scale = 1.0 - t * (1.0 - high_angle_scale_min)

        scaled_correction = raw_angle_correction * kp_scale
        if use_vf6_tune:
          scaled_correction *= self._vf6_mid_angle_boost(abs_des_ff)
        elif use_vf9_tune and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG:
          if abs(scaled_correction) < VF9_SMALL_CORR_ZERO_DEG and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.2):
            scaled_correction = 0.0

        if use_vf6_tune:
          if abs_des_ff < VF6_SMALL_ANGLE_DES_DEG:
            corr_rate_limit = VF6_SMALL_ANGLE_MAX_CORR_RATE_DEG_PER_CYCLE
          elif VF6_MID_ANGLE_START_DEG <= abs_des_ff <= VF6_MID_ANGLE_END_DEG:
            corr_rate_limit = MAX_PI_CORR_RATE_VF6_DEG_PER_CYCLE
          else:
            corr_rate_limit = MAX_PI_CORR_RATE_VF6_DEG_PER_CYCLE * 0.85
        else:
          corr_rate_limit = MAX_PI_CORR_RATE_DEG_PER_CYCLE
          if use_vf9_tune and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG:
            corr_rate_limit = min(corr_rate_limit, VF9_SMALL_ANGLE_MAX_CORR_RATE_DEG_PER_CYCLE)

        delta_corr = scaled_correction - self.prev_angle_correction
        delta_corr = max(-corr_rate_limit, min(corr_rate_limit, delta_corr))
        angle_correction = self.prev_angle_correction + delta_corr

        if use_vf6_tune and abs_des_ff < VF6_SMALL_ANGLE_DES_DEG:
          if abs(angle_correction) < VF6_SMALL_CORR_ZERO_DEG and abs(angle_error) < (VF6_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.2):
            angle_correction = 0.0
          elif abs(angle_error) < (VF6_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.2):
            angle_correction *= VF6_SMALL_CENTER_DECAY
        elif use_vf9_tune and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.2):
          angle_correction *= VF9_SMALL_CENTER_DECAY

        self.prev_angle_correction = angle_correction
        angle_steers_des = float(angle_steers_des_ff + angle_correction)
      else:
        angle_steers_des = float(angle_steers_des_ff)

    if self.use_steer_limited_by_safety:
      angle_control_saturated = steer_limited_by_safety
    else:
      angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > self.steer_angle_saturation_threshold
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = float(angle_steers_des)
    return 0, float(angle_steers_des), angle_log
