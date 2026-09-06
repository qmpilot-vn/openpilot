"""Host-side VinFast stop-and-go override.

VF6/VF7 InfoCAN cannot publish receding speed — see REENGAGE.md for the
latch and Wait track tree.
"""
import math
from dataclasses import dataclass

# Internal lead classes for TTC behavior tuning
LEAD_CLASS_NONE = 0
LEAD_CLASS_RECEDING = 1
LEAD_CLASS_STEADY = 2
LEAD_CLASS_CLOSING = 3
LEAD_CLASS_CRITICAL = 4

POPUP_REENGAGE = 3


def _is_radar_lead(ld) -> bool:
  """InfoCAN / radar only. VF6 vision leadOne flickers at red lights and must
  not set saw_lead or look like a receding car."""
  if ld is None or not getattr(ld, "status", False):
    return False
  return bool(getattr(ld, "radar", True))


def _is_vision_lead(ld) -> bool:
  return ld is not None and bool(getattr(ld, "status", False)) and not bool(getattr(ld, "radar", True))


def _close_vision_blocks_launch(lead_one, lead_two) -> bool:
  """Seg 0 first stop: vision-only ~3.7 m, no InfoCAN. Hold, do not count as a car."""
  for ld in (lead_one, lead_two):
    if not _is_vision_lead(ld):
      continue
    if float(getattr(ld, "dRel", 99.0)) > ReengageParams.VISION_BLOCK_DREL:
      continue
    if float(getattr(ld, "modelProb", 0.0)) < ReengageParams.VISION_BLOCK_PROB:
      continue
    if LeadSituation._lateral(ld) > ReengageParams.IN_LANE_YREL:
      continue
    return True
  return False


class ReengageParams:
  LAUNCH_ACCEL = 1.0
  BOOST_FRAMES = 60  # 0.6 s at 100 Hz (card apply)
  CREEP_DISTANCE = 10.0
  LEAD_RECEDING_VREL = 0.10
  CLOSING_SPEED_MIN = 0.1

  # Keep pulling past 10 km/h — old 10 km/h cap handed to the planner too
  # early, so the car sat there until longitudinal PID woke up.
  HANDOFF_CREEP = 15.0 / 3.6
  HANDOFF_FAST = 25.0 / 3.6
  HANDOFF_FAR = 20.0 / 3.6
  TAPER_START_FRAC = 0.85

  BOOST_RETRIGGER_SPEED = 1.5
  LAUNCH_BOOST_SPEED = 0.3
  CREEP_RESUME = 0.45
  NORMAL_RESUME = 0.75

  STOP_ACCEL_CREEP = -1.0
  STOP_ACCEL_NORMAL = -1.5
  # Same full-brake edge as VF6/VF7 so VF8/VF9 hold stop from ~6 m.
  DIST_FULL_BRAKE = 6.0
  # VF8/VF9 used to resume at 6/8 m (NORMAL sat on DIST_FULL_BRAKE) so
  # launch snapped to +1.0 with no blend. Give a real blend, still tighter
  # than VF6 InfoCAN 9/11.5 because VF8 has Rel_Vx.
  DIST_RESUME_CREEP = 10.5
  DIST_RESUME_NORMAL = 8.5
  DIST_HYST = 0.8
  DREL_SPIKE_M = 2.5
  DREL_SPIKE_HOLD = 3
  ACCEL_SLEW = 12.0  # m/s^2 per second
  DT = 0.02

  # VF8/VF9 launch punch: shared 1.0 / 0.6 s felt too hard off the line.
  VF8_LAUNCH_ACCEL = 0.80
  VF8_CREEP_RESUME = 0.28
  VF8_NORMAL_RESUME = 0.48
  VF8_BOOST_FRAMES = 40  # 0.4 s at 100 Hz
  VF8_ACCEL_SLEW = 6.0
  VF8_RECEDING_FLOOR_STANDSTILL = 0.50

  TTC_HARD_BRAKE = 0.9
  TTC_DECEL_END = 2.0
  TTC_GENTLE_END = 4.0
  TTC_STRONG_BRAKE_CREEP = -1.2
  TTC_STRONG_BRAKE_NORMAL = -2.0
  TTC_DECEL_AT_2S_CREEP = -0.20
  TTC_DECEL_AT_2S_NORMAL = -0.35
  TTC_GENTLE_AT_4S_CREEP = 0.15
  TTC_GENTLE_AT_4S_NORMAL = 0.20

  RECEDING_FLOOR_DIST = 6.0
  RECEDING_FLOOR_STANDSTILL = 0.9
  RECEDING_FLOOR_MOVING = 0.3

  # Motorbikes splitting the lane / shoulder typically sit outside this.
  IN_LANE_YREL = 1.8
  # First-in-line empty road (never saw a lead): require resume this many
  # frames (~30 ms at 100 Hz). Empty-after-saw_lead launches immediately.
  LAUNCH_CONFIRM_FRAMES = 3
  # Vision can steal both radarState slots for ~0.75 s at a red light
  # (567ee85c1955eff0|0000023b--3f948270f3/1), and liveTracks also drops
  # the 8.4 m car for ~100 ms. Keep the last InfoCAN lead through empty
  # or vision-only gaps so neither looks like vanish → launch.
  RADAR_HOLDOVER_FRAMES = 100  # 1.0 s at 100 Hz
  # radard zeros vLeadK below 1.2 m/s, so recede on InfoCAN range opening.
  DREL_OPEN_M = 1.0
  # First-in-line only: a close in-lane vision ghost must not look empty.
  # Never sets saw_lead / moving_away — vision is a brake, not a green.
  VISION_BLOCK_DREL = 8.0
  VISION_BLOCK_PROB = 0.50

  # VF6/VF7 stop closer; InfoCAN dRel often sits near 7 m at red lights.
  VF6_CREEP_DISTANCE = 12.0
  VF6_DIST_FULL_BRAKE = 6.0
  VF6_DIST_RESUME_CREEP = 11.5
  VF6_DIST_RESUME_NORMAL = 9.0
  VF6_RECEDING_FLOOR_DIST = 8.5


def is_vf6_reengage_platform(fingerprint: str) -> bool:
  fp = str(fingerprint or "")
  return fp in ("VINFAST_VF6", "VINFAST_VF7") or fp.startswith("VINFAST_VF6") or fp.startswith("VINFAST_VF7")


@dataclass(frozen=True)
class LeadSituation:
  distance: float
  v_rel: float
  closing_speed: float
  ttc: float
  abs_yrel: float
  in_lane: bool
  road_clear: bool
  moving_away: bool
  creeping: bool
  fast_resume: bool
  lead_class: int

  @property
  def handoff_speed(self) -> float:
    if self.creeping:
      return ReengageParams.HANDOFF_CREEP
    if self.fast_resume:
      return ReengageParams.HANDOFF_FAST
    return ReengageParams.HANDOFF_FAR

  @classmethod
  def from_leads(cls, lead_one, lead_two, v_ego: float = 0.0, vf6: bool = False,
                 d_stop: float | None = None, raw_d: float | None = None) -> "LeadSituation":
    chosen = cls._pick_lead(lead_one, lead_two)
    if chosen is None:
      return cls(
        distance=float('inf'), v_rel=0.0, closing_speed=0.0, ttc=float('inf'),
        abs_yrel=0.0, in_lane=False, road_clear=True, moving_away=False,
        creeping=False, fast_resume=True, lead_class=LEAD_CLASS_NONE,
      )

    distance = float(chosen.dRel)
    v_rel = float(getattr(chosen, 'vRel', 0.0))
    v_lead_k = float(getattr(chosen, 'vLeadK', 0.0))
    abs_yrel = cls._lateral(chosen)
    # radard used to publish vRel=-vEgo whenever vLeadK≈0. An offset
    # stationary track then looks like a closing stopped car.
    if vf6 and abs_yrel > 1.0 and abs(v_lead_k) < 0.15 and abs(v_rel + float(v_ego)) < 0.25:
      v_rel = 0.0
    # InfoCAN (VF6/VF7) has no Rel_Vx — fall back to Kalman absolute speed for TTC.
    elif abs(v_rel) < 0.25:
      if v_lead_k > 0.01:
        v_rel = v_lead_k - float(v_ego)
    closing_speed = max(0.0, -v_rel)
    ttc = distance / closing_speed if closing_speed > ReengageParams.CLOSING_SPEED_MIN else float('inf')
    road_clear = False
    in_lane = abs_yrel <= ReengageParams.IN_LANE_YREL
    open_d = distance if raw_d is None else float(raw_d)
    opening = (d_stop is not None) and (open_d - float(d_stop) >= ReengageParams.DREL_OPEN_M)
    moving_away = in_lane and ((v_rel > ReengageParams.LEAD_RECEDING_VREL) or opening)
    creep_dist = ReengageParams.VF6_CREEP_DISTANCE if vf6 else ReengageParams.CREEP_DISTANCE
    creeping = (distance < creep_dist) and (not moving_away)
    fast_resume = moving_away

    if moving_away:
      lead_class = LEAD_CLASS_RECEDING
    elif closing_speed <= ReengageParams.CLOSING_SPEED_MIN:
      lead_class = LEAD_CLASS_STEADY
    elif ttc < ReengageParams.TTC_HARD_BRAKE:
      lead_class = LEAD_CLASS_CRITICAL
    else:
      lead_class = LEAD_CLASS_CLOSING

    return cls(
      distance=distance,
      v_rel=v_rel,
      closing_speed=closing_speed,
      ttc=ttc,
      abs_yrel=abs_yrel,
      in_lane=in_lane,
      road_clear=road_clear,
      moving_away=moving_away,
      creeping=creeping,
      fast_resume=fast_resume,
      lead_class=lead_class,
    )

  @staticmethod
  def _lateral(lead) -> float:
    yrel = abs(float(getattr(lead, 'yRel', 0.0)))
    dpath = abs(float(getattr(lead, 'dPath', 0.0)))
    return dpath if dpath > 1e-3 else yrel

  @classmethod
  def _pick_lead(cls, lead_one, lead_two):
    """Prefer the closest in-lane object. A filtering motorbike beside us
    must not hide the car ahead that actually gates stop-and-go."""
    candidates = [ld for ld in (lead_one, lead_two) if _is_radar_lead(ld)]
    if not candidates:
      return None
    in_lane = [ld for ld in candidates if cls._lateral(ld) <= ReengageParams.IN_LANE_YREL]
    pool = in_lane if in_lane else candidates
    return min(pool, key=lambda ld: float(ld.dRel))


@dataclass
class _LeadView:
  status: bool
  dRel: float
  vRel: float
  yRel: float
  vLeadK: float
  dPath: float


class VinFastReengage:
  def __init__(self, vf6: bool = False):
    self.vf6 = bool(vf6)
    self.active = False
    self.overriding = False
    self.committed = False
    self.handoff_lock = False
    self.accel = 0.2
    self.lead_class = LEAD_CLASS_NONE
    self.boost_frames = 0
    self.standstill_armed = False
    self.popup_prev = False
    self.launch_confirm = 0
    # Any track this stop (in-lane or filtering). Vanish must not look like
    # first-in-line empty green.
    self.saw_lead = False
    self._d_filt = None
    self._d_hold = 0
    self._d_stop = None
    self._radar_hold = None
    self._radar_hold_age = 0
    self._in_brake = True
    self._slew_accel = 0.0
    self._was_overriding = False

  def observe_popup(self, popup: int) -> None:
    self.popup_prev = (popup == POPUP_REENGAGE)

  def _clear_latch(self) -> None:
    self.active = False
    self.overriding = False
    self.committed = False
    self.boost_frames = 0
    self.standstill_armed = False
    self.launch_confirm = 0
    self._in_brake = True
    self._was_overriding = False

  def _remember_lead(self, situation: LeadSituation, standstill: bool) -> None:
    if not situation.road_clear:
      self.saw_lead = True
    elif not standstill:
      self.saw_lead = False

  def _filter_lead(self, lead):
    if not _is_radar_lead(lead):
      self._d_hold = 0
      return None
    d = float(lead.dRel)
    if self._d_filt is None:
      self._d_filt = d
      return lead
    if abs(d - self._d_filt) > ReengageParams.DREL_SPIKE_M:
      self._d_hold += 1
      if self._d_hold < ReengageParams.DREL_SPIKE_HOLD:
        d = self._d_filt
      else:
        self._d_hold = 0
        self._d_filt = d
    else:
      self._d_hold = 0
      self._d_filt = 0.7 * self._d_filt + 0.3 * d
      d = self._d_filt
    return _LeadView(
      status=True,
      dRel=d,
      vRel=float(getattr(lead, 'vRel', 0.0)),
      yRel=float(getattr(lead, 'yRel', 0.0)),
      vLeadK=float(getattr(lead, 'vLeadK', 0.0)),
      dPath=float(getattr(lead, 'dPath', 0.0)),
    )

  def _resolve_radar_lead(self, lead_one, lead_two, standstill: bool):
    """Radar pick, or the last InfoCAN lead through a short empty / vision gap."""
    raw_d = None
    picked = LeadSituation._pick_lead(lead_one, lead_two)
    if picked is not None:
      raw_d = float(picked.dRel)
      picked = self._filter_lead(picked)
      self._radar_hold = picked
      self._radar_hold_age = 0
      if standstill:
        self._d_stop = raw_d if self._d_stop is None else min(self._d_stop, raw_d)
    elif (standstill and self._radar_hold is not None and
          self._radar_hold_age < ReengageParams.RADAR_HOLDOVER_FRAMES and
          (_is_vision_lead(lead_one) or _is_vision_lead(lead_two))):
      # Vision stole both slots — keep last InfoCAN. True empty (no vision)
      # is vanish → launch, same as VF8.
      self._radar_hold_age += 1
      picked = self._radar_hold
    else:
      picked = None
      if not standstill:
        self._radar_hold = None
        self._radar_hold_age = 0
    if not standstill:
      self._d_stop = None
    return picked, raw_d

  def _standstill_may_launch(self, sit: LeadSituation, allow_launch: bool,
                             popup_trigger: bool, vision_block: bool = False) -> bool:
    """Positive takeover at rest.

    Stock popup 3 fires when a lead leaves — a motorbike running a red looks
    the same to radar as a car leaving on green. Do not launch just because
    the road is now empty. A stopped in-lane car on the 7 m VF6 edge is also
    not a green — wait until they actually recede.
    """
    if not allow_launch:
      return False
    if vision_block and sit.road_clear and (not self.saw_lead):
      return False
    if (not sit.road_clear) and (not sit.in_lane):
      return False
    if sit.road_clear and self.saw_lead:
      # Planner resume is the green. Blocking here left us stopped after the
      # car ahead left, because popup-3-empty was treated like a motorbike.
      return True
    if not sit.road_clear:
      return bool(sit.moving_away)
    return self.committed or popup_trigger or (self.launch_confirm >= ReengageParams.LAUNCH_CONFIRM_FRAMES)

  def update(self, popup: int, long_active: bool, v_ego: float, standstill: bool,
             lead_one, lead_two, allow_launch: bool = False) -> tuple:
    """Latch / handoff / accel override. Returns (overriding, accel_cmd or None to use the model)."""
    picked, raw_d = self._resolve_radar_lead(lead_one, lead_two, standstill)
    situation = LeadSituation.from_leads(
      picked, None, v_ego=v_ego, vf6=self.vf6, d_stop=self._d_stop, raw_d=raw_d,
    )
    self._remember_lead(situation, standstill)

    if standstill:
      self.handoff_lock = False

    at_handoff = (not standstill) and (v_ego >= situation.handoff_speed)
    popup_req = (popup == POPUP_REENGAGE) and long_active
    popup_trigger = popup_req and (not self.popup_prev)

    # Sticky popup 3 at the cap would re-latch every frame and fight handoff.
    if at_handoff:
      self._clear_latch()
      self.handoff_lock = True
    elif (not self.handoff_lock):
      was_active = self.active
      if popup_req:
        self.active = True
      if popup_trigger or (self.active and (not was_active)):
        self.boost_frames = (ReengageParams.VF8_BOOST_FRAMES if not self.vf6
                             else ReengageParams.BOOST_FRAMES)

    if self.active and (not long_active):
      self._clear_latch()

    if standstill and allow_launch and situation.road_clear and (not self.saw_lead):
      self.launch_confirm += 1
    else:
      self.launch_confirm = 0

    if self.active and long_active:
      if standstill:
        vision_block = _close_vision_blocks_launch(lead_one, lead_two)
        if self._standstill_may_launch(situation, allow_launch, popup_trigger, vision_block):
          self.committed = True
        else:
          self.committed = False
          self.overriding = False
          self._was_overriding = False
          return False, None
      elif not self.committed:
        self.overriding = False
        self._was_overriding = False
        return False, None

      accel = self._compute_accel(v_ego, standstill, situation)
      self.accel = accel
      self.lead_class = situation.lead_class
      self.overriding = True
      if self.boost_frames > 0:
        self.boost_frames -= 1
      return True, accel

    self.overriding = False
    self._was_overriding = False
    self.lead_class = LEAD_CLASS_NONE
    self.boost_frames = 0
    self.standstill_armed = False
    return False, None

  def _compute_accel(self, v_ego: float, standstill: bool, sit: LeadSituation) -> float:
    P = ReengageParams
    vf6 = self.vf6
    dist_full = P.VF6_DIST_FULL_BRAKE if vf6 else P.DIST_FULL_BRAKE
    dist_resume = (P.VF6_DIST_RESUME_CREEP if sit.creeping else P.VF6_DIST_RESUME_NORMAL) if vf6 else (
      P.DIST_RESUME_CREEP if sit.creeping else P.DIST_RESUME_NORMAL)
    recede_floor = P.VF6_RECEDING_FLOOR_DIST if vf6 else P.RECEDING_FLOOR_DIST

    if standstill:
      self.standstill_armed = True
    elif self.standstill_armed and v_ego < P.BOOST_RETRIGGER_SPEED:
      self.boost_frames = P.VF8_BOOST_FRAMES if not vf6 else P.BOOST_FRAMES
      self.standstill_armed = False

    launch_boost_active = standstill or (v_ego < P.LAUNCH_BOOST_SPEED) or (self.boost_frames > 0)
    launch_accel = P.LAUNCH_ACCEL if vf6 else P.VF8_LAUNCH_ACCEL
    stop_accel = P.STOP_ACCEL_CREEP if sit.creeping else P.STOP_ACCEL_NORMAL
    creep_resume = P.CREEP_RESUME if vf6 else P.VF8_CREEP_RESUME
    normal_resume = P.NORMAL_RESUME if vf6 else P.VF8_NORMAL_RESUME

    if sit.creeping:
      resume_accel = launch_accel if launch_boost_active else creep_resume
    else:
      resume_accel = launch_accel if launch_boost_active else normal_resume

    if sit.road_clear or sit.moving_away:
      # Receding in-lane lead is a green, even if InfoCAN dRel still sits
      # on the VF6 full-brake edge that a stopped car occupies.
      self._in_brake = False
      accel_dist = resume_accel
    else:
      if self._in_brake:
        enter_resume = sit.distance >= (dist_full + P.DIST_HYST)
      else:
        enter_resume = sit.distance >= (dist_full - P.DIST_HYST)
      if not enter_resume:
        self._in_brake = True
        accel_dist = stop_accel
      elif sit.distance >= dist_resume:
        self._in_brake = False
        accel_dist = resume_accel
      else:
        self._in_brake = False
        u_dist = (sit.distance - dist_full) / max(0.1, dist_resume - dist_full)
        accel_dist = stop_accel + max(0.0, min(1.0, u_dist)) * (resume_accel - stop_accel)

    strong_brake = P.TTC_STRONG_BRAKE_CREEP if sit.creeping else P.TTC_STRONG_BRAKE_NORMAL
    decel_at_2s = P.TTC_DECEL_AT_2S_CREEP if sit.creeping else P.TTC_DECEL_AT_2S_NORMAL
    gentle_at_4s = P.TTC_GENTLE_AT_4S_CREEP if sit.creeping else P.TTC_GENTLE_AT_4S_NORMAL

    if sit.road_clear or not math.isfinite(sit.ttc):
      accel_ttc = resume_accel
    elif sit.ttc < P.TTC_HARD_BRAKE:
      accel_ttc = strong_brake
    elif sit.ttc <= P.TTC_DECEL_END:
      u_ttc = (sit.ttc - P.TTC_HARD_BRAKE) / max(0.1, (P.TTC_DECEL_END - P.TTC_HARD_BRAKE))
      accel_ttc = strong_brake + u_ttc * (decel_at_2s - strong_brake)
    elif sit.ttc <= P.TTC_GENTLE_END:
      u_ttc = (sit.ttc - P.TTC_DECEL_END) / max(0.1, (P.TTC_GENTLE_END - P.TTC_DECEL_END))
      accel_ttc = decel_at_2s + u_ttc * (gentle_at_4s - decel_at_2s)
    else:
      accel_ttc = resume_accel

    accel_cmd = min(accel_dist, accel_ttc)
    if sit.moving_away and sit.distance > recede_floor:
      floor = (P.RECEDING_FLOOR_STANDSTILL if vf6 else P.VF8_RECEDING_FLOOR_STANDSTILL) if standstill else P.RECEDING_FLOOR_MOVING
      accel_cmd = max(accel_cmd, floor)

    if accel_cmd > 0.0:
      max_speed = sit.handoff_speed
      taper_start = max_speed * P.TAPER_START_FRAC
      if v_ego >= taper_start:
        taper = (max_speed - v_ego) / max(1e-3, max_speed - taper_start)
        accel_cmd = accel_cmd * max(0.0, taper)

    if self._was_overriding and accel_cmd > self._slew_accel:
      # Brake / cut accel immediately (TTC, 7 m edge). Only slew on the
      # way up so a red-light dRel flicker cannot jump -1.0 -> +0.6.
      step = (P.ACCEL_SLEW if vf6 else P.VF8_ACCEL_SLEW) * P.DT
      accel_cmd = min(self._slew_accel + step, accel_cmd)
    self._slew_accel = accel_cmd
    self._was_overriding = True
    return accel_cmd
