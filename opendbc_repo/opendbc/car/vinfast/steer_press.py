"""VinFast steeringPressed latch.

The packaged .so latched on raw |driver torque| (~2 Nm, ~40 ms). A speed bump
flips torque ±2–4 Nm and spikes SAS rate; that looked like a grab and dropped
latActive via steerOverride.

This latch:

- averages torque so 20 ms sign-flips cancel
- zeros samples with implausible SAS rate (bump / sensor glitch)
- keeps vf6 hysteresis (press / release + on/off dwell)
- when EPS is angle-active, still raises press_nm with |steerRate| so a real
  OP turn's torsion bar is not treated as a driver grab
"""
from collections import deque
from types import SimpleNamespace

# 100 Hz carstate. CAN torque/SAS are ~50 Hz (samples usually duplicated).
STEER_TQ_AVG_FRAMES = 8
STEER_RATE_GLITCH_DEG_S = 80.0  # 000002c0/5 bump: 88–240 deg/s
STEER_RATE_INHIBIT_FRAMES = 20  # ignore new press ~200 ms after a SAS glitch

# Tai overlay (values.py): 1.5 / 0.75. Fallback when values_impl.so is missing
# or does not export the names (host x86, or older package).
_DEFAULTS = SimpleNamespace(
  STEER_DRIVER_PRESS_NM=1.5,
  STEER_DRIVER_RELEASE_NM=0.75,
  STEER_PRESSED_MIN_COUNT=5,
  STEER_RELEASE_MIN_COUNT=50,
  STEER_PRESS_RATE_GAIN=0.02,
  STEER_PRESS_NM_MAX=4.0,
)


_CCP = None
_CCP_READY = False


def _ccp():
  global _CCP, _CCP_READY
  if _CCP_READY:
    return _CCP
  _CCP_READY = True
  try:
    from opendbc.car.vinfast.values import CarControllerParams as CCP
  except Exception:
    _CCP = _DEFAULTS
    return _CCP
  for name, val in vars(_DEFAULTS).items():
    if not hasattr(CCP, name):
      setattr(CCP, name, val)
  _CCP = CCP
  return _CCP


class VinFastSteerPress:
  def __init__(self):
    self._buf = deque([0.0] * STEER_TQ_AVG_FRAMES, maxlen=STEER_TQ_AVG_FRAMES)
    self.latched = False
    self.pressed = False
    self.press_cnt = 0
    self.release_cnt = 0
    self.inhibit = 0

  def update(self, torque_nm: float, rate_deg_s: float,
             eps_angle_active: bool = False, eps_drive_intervention: bool = False) -> bool:
    ccp = _ccp()
    glitch = abs(rate_deg_s) >= STEER_RATE_GLITCH_DEG_S
    self._buf.append(0.0 if glitch else float(torque_nm))
    avg = abs(sum(self._buf) / len(self._buf))
    if glitch:
      self.inhibit = STEER_RATE_INHIBIT_FRAMES
    else:
      self.inhibit = max(0, self.inhibit - 1)

    press_nm = float(ccp.STEER_DRIVER_PRESS_NM)
    if eps_angle_active and not glitch:
      press_nm = min(
        float(ccp.STEER_PRESS_NM_MAX),
        press_nm + float(ccp.STEER_PRESS_RATE_GAIN) * abs(rate_deg_s),
      )

    allow_new = self.inhibit == 0
    if avg > press_nm and allow_new:
      self.latched = True
    elif avg < float(ccp.STEER_DRIVER_RELEASE_NM):
      self.latched = False

    raw = self.latched or (bool(eps_drive_intervention) and avg > press_nm and allow_new)
    if raw:
      self.release_cnt = 0
      self.press_cnt = min(self.press_cnt + 1, int(ccp.STEER_PRESSED_MIN_COUNT))
      if self.press_cnt >= int(ccp.STEER_PRESSED_MIN_COUNT):
        self.pressed = True
    else:
      self.press_cnt = 0
      self.release_cnt = min(self.release_cnt + 1, int(ccp.STEER_RELEASE_MIN_COUNT))
      if self.release_cnt >= int(ccp.STEER_RELEASE_MIN_COUNT):
        self.pressed = False
    return self.pressed
