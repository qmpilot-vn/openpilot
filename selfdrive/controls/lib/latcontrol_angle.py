import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

# TODO This is speed dependent
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

# VinFast only: extra PID wraps feedforward desired angle vs measured angle. Latest EPS often
# tracks feedforward well — disable PI to try pure feedforward angle (still angle actuation in CAN).
# Keep exactly one of the next two lines active (prefix the other with #).
VINFAST_USE_STEERING_ANGLE_PID = True
# VINFAST_USE_STEERING_ANGLE_PID = False

VINFAST_ANGLE_KP_BP = [0.0, 3.5, 5.56, 8.33, 11.11, 55.56]  # m/s breakpoints
VINFAST_ANGLE_KP_VF8 = [0.7, 0.75, 0.85, 0.9, 1.1, 1.3]
VINFAST_ANGLE_KP_VF9 = [0.8, 0.9, 0.95, 1.0, 1.1, 1.3]
VINFAST_ANGLE_KI_VF8 = 0.035
VINFAST_ANGLE_KI_VF9 = 0.005
VINFAST_ANGLE_KD = 0.0
HIGH_ANGLE_START_DEG = 50.0
HIGH_ANGLE_END_DEG = 470.0
HIGH_ANGLE_KP_SCALE_MIN = 0.1
MAX_PI_CORR_RATE_DEG_PER_CYCLE = 0.35
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
    self.use_steer_limited_by_safety = CP.brand in ("tesla", "hyundai")
    self.steer_angle_saturation_threshold = 20.0 if CP.brand == "vinfast" else STEER_ANGLE_SATURATION_THRESHOLD

    if CP.brand == "vinfast" and VINFAST_USE_STEERING_ANGLE_PID:
      self.vinfast_car_fingerprint = getattr(CP, "carFingerprint", "")
      if self.vinfast_car_fingerprint == "VINFAST_VF8":
        vinfast_kp = [VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF8]
        vinfast_ki = VINFAST_ANGLE_KI_VF8
      elif self.vinfast_car_fingerprint == "VINFAST_VF9":
        vinfast_kp = [VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF9]
        vinfast_ki = VINFAST_ANGLE_KI_VF9
      else:
        vinfast_kp = [VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF8]
        vinfast_ki = VINFAST_ANGLE_KI_VF8

      max_angle_correction = 15.0
      self.pid = PIDController(
        vinfast_kp,
        vinfast_ki,
        VINFAST_ANGLE_KD,
        pos_limit=max_angle_correction,
        neg_limit=-max_angle_correction,
        rate=100
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
        is_vf9 = self.vinfast_car_fingerprint == "VINFAST_VF9"

        if is_vf9 and abs(angle_steers_des_ff) < VF9_SMALL_ANGLE_DES_DEG:
          if abs(angle_error) < VF9_SMALL_ANGLE_ERR_DEADBAND_DEG:
            angle_error = 0.0
          if hasattr(self.pid, 'i') and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.5):
            self.pid.i *= VF9_SMALL_ANGLE_I_BLEED

        if self.prev_angle_error * angle_error < 0:
          if hasattr(self.pid, 'i'):
            self.pid.i *= 0.4

        self.prev_angle_error = angle_error

        freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

        raw_angle_correction = self.pid.update(
          error=angle_error,
          error_rate=0.0,
          speed=CS.vEgo,
          feedforward=0.0,
          freeze_integrator=freeze_integrator
        )

        abs_des_ff = abs(angle_steers_des_ff)
        if abs_des_ff <= HIGH_ANGLE_START_DEG:
          kp_scale = 1.0
        elif abs_des_ff >= HIGH_ANGLE_END_DEG:
          kp_scale = HIGH_ANGLE_KP_SCALE_MIN
        else:
          t = (abs_des_ff - HIGH_ANGLE_START_DEG) / (HIGH_ANGLE_END_DEG - HIGH_ANGLE_START_DEG)
          kp_scale = 1.0 - t * (1.0 - HIGH_ANGLE_KP_SCALE_MIN)

        scaled_correction = raw_angle_correction * kp_scale
        if is_vf9 and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG:
          if abs(scaled_correction) < VF9_SMALL_CORR_ZERO_DEG and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.2):
            scaled_correction = 0.0

        corr_rate_limit = MAX_PI_CORR_RATE_DEG_PER_CYCLE
        if is_vf9 and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG:
          corr_rate_limit = min(corr_rate_limit, VF9_SMALL_ANGLE_MAX_CORR_RATE_DEG_PER_CYCLE)
        delta_corr = scaled_correction - self.prev_angle_correction
        delta_corr = max(-corr_rate_limit, min(corr_rate_limit, delta_corr))
        angle_correction = self.prev_angle_correction + delta_corr
        if is_vf9 and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.2):
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
