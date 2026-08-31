import math
import numpy as np

from cereal import car
from openpilot.common.constants import CV
from opendbc.car.vinfast.values import is_vf6_safety_platform
from openpilot.sunnypilot.selfdrive.car.cruise_ext import VCruiseHelperSP


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
VINFAST_V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 60
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


def _button_type(bt):
  """Normalize capnp ButtonEvent type to ButtonType enum (supports .type.raw)."""
  return getattr(bt, "raw", bt)


class VCruiseHelper(VCruiseHelperSP):
  def __init__(self, CP, CP_SP):
    VCruiseHelperSP.__init__(self, CP, CP_SP)
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}
    # VinFast: first valid car set speed (TagSpeed / cluster V) becomes experimental init.
    default_experimental = VINFAST_V_CRUISE_INITIAL_EXPERIMENTAL_MODE if CP.brand == "vinfast" else V_CRUISE_INITIAL_EXPERIMENTAL_MODE
    self.v_initial_experimental_mode = default_experimental
    self.last_tag_speed_kph = 0.0
    self._car_set_speed_latched = False
    # Adjust v_initial_experimental_mode via accel/decel before cruise is initialized (session-only)
    self.adjusting_vmax_init = False
    self.vmax_init_adjust_timer = 0
    self.vf6_platform = is_vf6_safety_platform(CP.carFingerprint)

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    self.get_minimum_set_speed(is_metric)
    _enabled = self.update_enabled_state(CS, enabled)

    # VF6/VF7 (origin/vf6): ADAS_ACC_TagSpeed on InfoCAN → pcmCruiseSpeed sync
    if self.vf6_platform:
      if CS.cruiseState.available:
        if self.CP_SP.pcmCruiseSpeed or self.CP.pcmCruise:
          self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
          self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
          if CS.cruiseState.speed == 0:
            self.v_cruise_kph = V_CRUISE_UNSET
            self.v_cruise_cluster_kph = V_CRUISE_UNSET
          elif CS.cruiseState.speed == -1:
            self.v_cruise_kph = -1
            self.v_cruise_cluster_kph = -1
        elif not self.CP.pcmCruise or (not self.CP_SP.pcmCruiseSpeed and _enabled):
          self._update_v_cruise_non_pcm(CS, _enabled, is_metric)
          self.update_speed_limit_assist_v_cruise_non_pcm()
          self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        self.v_cruise_kph = V_CRUISE_UNSET
        self.v_cruise_cluster_kph = V_CRUISE_UNSET

      if not self.CP.pcmCruise or not self.CP_SP.pcmCruiseSpeed:
        self.update_button_timers(CS, enabled)
      return

    # VinFast: latch the first dash set speed as experimental init (do not follow later tag moves).
    if self.CP.brand == "vinfast":
      self._maybe_latch_car_set_speed(CS)

    if self.CP.brand == "vinfast":
      self.update_vmax_init_experimental(CS, enabled, is_metric)

    if CS.cruiseState.available:
      if not self.CP.pcmCruise or (not self.CP_SP.pcmCruiseSpeed and _enabled):
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, _enabled, is_metric)
        self.update_speed_limit_assist_v_cruise_non_pcm()
        self.v_cruise_cluster_kph = self.v_cruise_kph
        # timers are advanced once at the end of this method; a second call here would
        # step them twice per cycle, so `timer % CRUISE_LONG_PRESS` never hits and
        # holding +/- would do nothing
      else:
        # pcmCruise + pcmCruiseSpeed: follow car set speed, allow button deltas (sunnypilot)
        car_speed_kph = CS.cruiseState.speed * CV.MS_TO_KPH if CS.cruiseState.speed > 0 else V_CRUISE_UNSET

        self.update_button_timers(CS, enabled)
        button_adjustment = 0.0

        if enabled and CS.buttonEvents:
          long_press = False
          button_type = None
          v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

          for b in CS.buttonEvents:
            br = _button_type(b.type)
            if br in self.button_timers and not b.pressed:
              if self.button_timers[br] > CRUISE_LONG_PRESS:
                continue  # end long press
              button_type = br
              break
          else:
            for k, timer in self.button_timers.items():
              if timer and timer % CRUISE_LONG_PRESS == 0:
                button_type = k
                long_press = True
                break

          if button_type is not None:
            cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
            if not (button_type == ButtonType.accelCruise and cruise_standstill):
              if self.button_change_states[button_type]["enabled"]:
                if not self.update_speed_limit_assist_pre_active_confirmed(button_type):
                  long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
                  button_adjustment = v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

        if button_adjustment != 0.0 and car_speed_kph != V_CRUISE_UNSET:
          self.v_cruise_kph = car_speed_kph + button_adjustment
          self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)
        elif car_speed_kph != V_CRUISE_UNSET:
          self.v_cruise_kph = car_speed_kph
        else:
          if CS.cruiseState.speed == 0:
            self.v_cruise_kph = V_CRUISE_UNSET
          elif CS.cruiseState.speed == -1:
            self.v_cruise_kph = -1
          else:
            self.v_cruise_kph = V_CRUISE_UNSET

        if self.v_cruise_kph != V_CRUISE_UNSET and self.v_cruise_kph != -1:
          self.v_cruise_cluster_kph = self.v_cruise_kph
        elif CS.cruiseState.speedCluster > 0:
          self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        elif CS.cruiseState.speed == 0:
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_cluster_kph = -1
        else:
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

    if not self.CP.pcmCruise or not self.CP_SP.pcmCruiseSpeed:
      self.update_button_timers(CS, enabled)

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      br = _button_type(b.type)
      if br in self.button_timers and not b.pressed:
        if self.button_timers[br] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = br
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Sunnypilot: only adjust in D; allow unknown (tests / ports without gear)
    if (CS.gearShifter != car.CarState.GearShifter.unknown and
        CS.gearShifter != car.CarState.GearShifter.drive):
      return

    if not enabled:
      return

    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    if not self.button_change_states[button_type]["enabled"]:
      return

    if self.update_speed_limit_assist_pre_active_confirmed(button_type):
      return

    long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      br = _button_type(b.type)
      if br in self.button_timers:
        self.button_timers[br] = 1 if b.pressed else 0
        self.button_change_states[br] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def _car_set_speed_kph(self, CS):
    """VinFast cluster set speed: ADAS_ACC_TagSpeed is speed, else speedCluster."""
    if CS.cruiseState.speed > 0:
      kph = CS.cruiseState.speed * CV.MS_TO_KPH
    elif CS.cruiseState.speedCluster > 0:
      kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    else:
      return None
    if V_CRUISE_MIN <= kph <= V_CRUISE_MAX:
      return kph
    return None

  def _maybe_latch_car_set_speed(self, CS) -> None:
    if self._car_set_speed_latched:
      return
    kph = self._car_set_speed_kph(CS)
    if kph is None:
      return
    self.v_initial_experimental_mode = int(round(kph))
    self.last_tag_speed_kph = kph
    self._car_set_speed_latched = True

  def get_v_initial_experimental_mode(self) -> int:
    """Initial set speed cap for experimental mode (first VF set speed + vmax-init buttons)."""
    return self.v_initial_experimental_mode

  def update_vmax_init_experimental(self, CS, enabled, is_metric) -> bool:
    """Adjust v_initial_experimental_mode with accel/decel before cruise is initialized."""
    if self.v_cruise_initialized or not enabled:
      self.adjusting_vmax_init = False
      self.vmax_init_adjust_timer = 0
      return False

    button_type = None

    for b in CS.buttonEvents:
      br = _button_type(b.type)
      if br not in (ButtonType.accelCruise, ButtonType.decelCruise):
        continue
      if b.pressed:
        self.adjusting_vmax_init = True
        self.vmax_init_adjust_timer = 30
      elif self.adjusting_vmax_init:
        button_type = br
        break

    if self.vmax_init_adjust_timer > 0:
      self.vmax_init_adjust_timer -= 1
    else:
      self.adjusting_vmax_init = False

    if button_type is not None and self.adjusting_vmax_init:
      current_vmax = self.get_v_initial_experimental_mode()
      v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT
      long_press = False
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          long_press = True
          v_cruise_delta *= 5
          break

      if button_type == ButtonType.accelCruise:
        new_vmax = current_vmax + v_cruise_delta
      else:
        new_vmax = current_vmax - v_cruise_delta

      self.v_initial_experimental_mode = int(round(np.clip(new_vmax, V_CRUISE_MIN, V_CRUISE_MAX)))
      return True

    return False

  def initialize_v_cruise(self, CS, experimental_mode: bool, dynamic_experimental_control: bool) -> None:
    # VF6/VF7: set speed comes from ADAS_ACC_TagSpeed on InfoCAN (origin/vf6)
    if self.vf6_platform and (self.CP.pcmCruise or self.CP_SP.pcmCruiseSpeed):
      return

    if self.CP.pcmCruise:
      return

    if self.CP.brand == "vinfast":
      self._maybe_latch_car_set_speed(CS)

    initial_experimental_mode = experimental_mode and not dynamic_experimental_control
    if initial_experimental_mode:
      initial = self.get_v_initial_experimental_mode() if self.CP.brand == "vinfast" else V_CRUISE_INITIAL_EXPERIMENTAL_MODE
    else:
      initial = V_CRUISE_INITIAL

    if any(_button_type(b.type) in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      if self.v_cruise_kph_last > 0 and V_CRUISE_MIN <= self.v_cruise_kph_last <= V_CRUISE_MAX:
        self.v_cruise_kph = int(round(self.v_cruise_kph_last))
      else:
        self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph
