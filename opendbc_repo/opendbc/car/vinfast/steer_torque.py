"""EPS LATE_CON torque factor. Interpolation from vf-dev-c3xl / origin/vf6.

Max angle for full factor:
  VF6 / VF7 Plus / VF8 Plus          180
  VF8 2023-24 production / VF9       100
  VF8 non-production only            470
"""
_VF6_MAX_ANGLE = 180.0
_VF6_MIN_FACTOR = 0.6
_VF6_MAX_FACTOR = 1.0

# origin/vf6 VF8/VF9 factors; only the full-torque angle changes by platform
_VF8_MIN_FACTOR = 0.5
_VF8_MAX_FACTOR = 0.6
_VF8_VF9_PROD_MAX_ANGLE = 100.0
_VF8_NON_PROD_MAX_ANGLE = 470.0

_VF6_PLATFORMS = ("VINFAST_VF6", "VINFAST_VF7_PLUS", "VINFAST_VF8_PLUS")
# 2023-24 VF8 is the production car. 470° is not its torque-factor scale.
_VF8_NON_PROD = ()
_VF8_VF9_PROD = ("VINFAST_VF8", "VINFAST_VF9", "VINFAST_VF8_ECO")


def _fp(car_fingerprint) -> str:
  return getattr(car_fingerprint, "name", None) or str(car_fingerprint or "")


def is_vf6_platform(car_fingerprint) -> bool:
  fp = _fp(car_fingerprint)
  return fp in _VF6_PLATFORMS or fp.startswith("VINFAST_VF6") or fp.startswith("VINFAST_VF7")


def is_vf8_non_production(car_fingerprint) -> bool:
  return _fp(car_fingerprint) in _VF8_NON_PROD


def steer_angle_max(car_fingerprint) -> float:
  fp = _fp(car_fingerprint)
  if is_vf6_platform(fp):
    return _VF6_MAX_ANGLE
  if fp in _VF8_NON_PROD:
    return _VF8_NON_PROD_MAX_ANGLE
  return _VF8_VF9_PROD_MAX_ANGLE


def torque_factor(apply_angle, lat_active, car_fingerprint) -> float:
  abs_angle = abs(apply_angle) if lat_active else 0.0
  max_angle = steer_angle_max(car_fingerprint)
  if is_vf6_platform(car_fingerprint):
    min_tf, max_tf = _VF6_MIN_FACTOR, _VF6_MAX_FACTOR
  else:
    min_tf, max_tf = _VF8_MIN_FACTOR, _VF8_MAX_FACTOR
  if max_angle <= 0.0 or abs_angle >= max_angle:
    return max_tf
  return min_tf + (max_tf - min_tf) * (abs_angle / max_angle)
