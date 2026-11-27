"""VinFast driver-hand estimate, steering yield and the steeringPressed latch.

What the torque sensor reads with hands off (rlogs 00000436 VF6 and
00000014 VF9, 100 Hz, 100k frames with lateral on):

- While openpilot turns the wheel the reading is the wheel's own inertia and
  drag: about -J*accel - b*rate. It stays under 1.1 Nm on the VF9 and is not
  aligned with the rotation on the VF6. The older "up to 1.6 Nm per 80 deg/s"
  figure came from frames where a hand was already pulling.
- A speed bump flips the reading +-4..5 Nm at about 16 Hz. It is the only
  hands-off event above 3 Nm in those logs.

So the hand estimate is the 4-frame mean of the signed torque (a bump cancels)
with the inertia/drag term removed only where it has the same sign as the
reading. The model can shrink the reading. It cannot add to it or flip it.

- yield_blend goes 0..1 between STEER_YIELD_START_NM and STEER_YIELD_FULL_NM,
  from the lowest estimate of the last STEER_YIELD_PERSIST_FRAMES, so one bump
  half-wave does not move the angle request.
- steeringPressed latches when the torque impulse above STEER_TAKEOVER_NM
  reaches STEER_TAKEOVER_IMPULSE_NMS in one direction: 2.0 Nm for 240 ms,
  2.5 Nm for 120 ms, about 60 ms at 3.5 Nm and above. A sign change starts over.
  Release stays sticky: below STEER_DRIVER_RELEASE_NM for STEER_RELEASE_MIN_COUNT.
"""
from collections import deque
from types import SimpleNamespace

# 100 Hz carstate. CAN torque/SAS are ~50 Hz and usually duplicated, so 4 frames
# is one sign-flip each way. Long enough to cancel a bump, short enough that a
# grab shows up inside ~40 ms.
STEER_TQ_AVG_FRAMES = 4
STEER_PRESS_DT = 0.01

# Same numbers as values.CarControllerParams, for a values module that does not export them.
_DEFAULTS = SimpleNamespace(
  STEER_DRIVER_RELEASE_NM=0.8,
  STEER_RELEASE_MIN_COUNT=50,
  STEER_SELF_TQ_VF6=(0.0005, 0.003),
  STEER_SELF_TQ_VF8=(0.0003, 0.012),
  STEER_YIELD_START_NM=1.2,
  STEER_YIELD_FULL_NM=2.2,
  STEER_YIELD_PERSIST_FRAMES=4,
  STEER_TAKEOVER_NM=1.5,
  STEER_TAKEOVER_NM_CAP=3.5,
  STEER_TAKEOVER_IMPULSE_NMS=0.12,
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


def _is_vf6(fingerprint) -> bool:
  try:
    from opendbc.car.vinfast.values import is_vf6_safety_platform
    return bool(is_vf6_safety_platform(fingerprint))
  except Exception:
    return "VF6" in str(fingerprint) or "VF7" in str(fingerprint)


def _clip(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else hi if x > hi else x


class VinFastSteerPress:
  def __init__(self, fingerprint=None):
    ccp = _ccp()
    self._inertia, self._drag = ccp.STEER_SELF_TQ_VF6 if _is_vf6(fingerprint) else ccp.STEER_SELF_TQ_VF8
    self._buf = deque([0.0] * STEER_TQ_AVG_FRAMES, maxlen=STEER_TQ_AVG_FRAMES)
    self._rate = deque([0.0] * (STEER_TQ_AVG_FRAMES + 1), maxlen=STEER_TQ_AVG_FRAMES + 1)
    persist = max(1, int(ccp.STEER_YIELD_PERSIST_FRAMES))
    self._hand = deque([0.0] * persist, maxlen=persist)
    self.pressed = False
    self.release_cnt = 0
    self.evidence = 0.0
    self._evidence_sign = 0.0
    self.yield_blend = 0.0
    self.excess_nm = 0.0

  def update(self, torque_nm: float, rate_deg_s: float,
             eps_angle_active: bool = False, eps_drive_intervention: bool = False) -> bool:
    # Both flags stay in the signature because carstate always passes them.
    # EPS_Drive_Intervention never left 0 in any log, with or without a hand on the wheel.
    del eps_angle_active, eps_drive_intervention
    ccp = _ccp()
    self._buf.append(float(torque_nm))
    self._rate.append(float(rate_deg_s))
    avg = sum(self._buf) / len(self._buf)
    accel = (self._rate[-1] - self._rate[0]) / (STEER_PRESS_DT * (len(self._rate) - 1))
    own = -self._inertia * accel - self._drag * self._rate[-1]
    hand = max(0.0, abs(avg) - abs(own)) if own * avg > 0.0 else abs(avg)
    self.excess_nm = hand

    self._hand.append(hand)
    start = float(ccp.STEER_YIELD_START_NM)
    span = float(ccp.STEER_YIELD_FULL_NM) - start
    self.yield_blend = _clip((min(self._hand) - start) / span, 0.0, 1.0) if span > 0.0 else 0.0

    floor = float(ccp.STEER_TAKEOVER_NM)
    impulse = float(ccp.STEER_TAKEOVER_IMPULSE_NMS)
    if hand > floor:
      sign = 1.0 if avg > 0.0 else -1.0
      if sign != self._evidence_sign:
        self._evidence_sign = sign
        self.evidence = 0.0
      gain = min(hand, float(ccp.STEER_TAKEOVER_NM_CAP)) - floor
      self.evidence = min(self.evidence + gain * STEER_PRESS_DT, impulse)
    else:
      self.evidence = max(0.0, self.evidence - (floor - hand) * STEER_PRESS_DT)

    if self.evidence >= impulse:
      self.pressed = True
      self.release_cnt = 0
    elif self.pressed:
      if abs(avg) < float(ccp.STEER_DRIVER_RELEASE_NM):
        self.release_cnt = min(self.release_cnt + 1, int(ccp.STEER_RELEASE_MIN_COUNT))
        if self.release_cnt >= int(ccp.STEER_RELEASE_MIN_COUNT):
          self.pressed = False
          self.evidence = 0.0
      else:
        self.release_cnt = 0
    else:
      self.release_cnt = 0
    return self.pressed
