from unittest.mock import MagicMock

import pytest

from cereal import log, custom
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName


class _Params:
  def __init__(self, mads=True):
    self._mads = mads

  def get_bool(self, key, **_kwargs):
    if key == "Mads":
      return self._mads
    if key == "MadsMainCruiseAllowed":
      return True
    if key == "MadsUnifiedEngagementMode":
      return False
    return False

  def get(self, key, **_kwargs):
    if key == "MadsSteeringMode":
      return 2
    return 0


class _DiskParams:
  def get_bool(self, _key):
    return False


class _Selfdrive:
  def __init__(self, fingerprint, mads_on):
    self.CP = structs.CarParams(carFingerprint=fingerprint, brand="vinfast")
    self.CP.passive = False
    self.CP_SP = structs.CarParamsSP()
    self.params = _Params(mads_on)
    self.events = Events()
    self.events_sp = EventsSP()
    self.state_machine = MagicMock()
    self.state_machine.current_alert_types = []
    self.state_machine.soft_disable_timer = 0
    self.initialized = True
    self.enabled = False
    self.sm = {"pandaStates": []}
    self.CS_prev = structs.CarState()
    self.CS_prev.cruiseState.available = True
    self.CS_prev.cruiseState.enabled = False


def _car_state(available, enabled, button=None, pressed=False):
  cs = structs.CarState()
  cs.cruiseState.available = available
  cs.cruiseState.enabled = enabled
  if button is not None:
    be = cs.init("buttonEvents", 1)
    be[0].type = button
    be[0].pressed = pressed
  return cs


def _step(mads, cs, button_enable=False, update=True):
  mads.selfdrive.events.clear()
  mads.selfdrive.events_sp.clear()
  if button_enable:
    mads.selfdrive.events.add(EventName.buttonEnable)
  # Longitudinal reads buttonEnable after this and before mads.update.
  mads.suppress_vinfast_long_engage(mads.selfdrive.events, cs)
  kept_long = mads.selfdrive.events.has(EventName.buttonEnable)
  if update:
    mads.update(cs)
    mads.selfdrive.CS_prev = cs
  return kept_long


@pytest.fixture
def mads_for(monkeypatch):
  monkeypatch.setattr("openpilot.sunnypilot.mads.mads.Params", _DiskParams)

  def _build(fingerprint, mads_on=True):
    return ModularAssistiveDrivingSystem(_Selfdrive(fingerprint, mads_on))

  return _build


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF6", "VINFAST_VF8"])
def test_set_plus_arms_lateral_and_set_minus_cancels_without_long(mads_for, fingerprint):
  mads = mads_for(fingerprint)

  press_plus = _car_state(True, False, ButtonType.accelCruise, True)
  _step(mads, press_plus)
  assert mads.selfdrive.events_sp.has(EventNameSP.lkasEnable)
  assert mads.enabled and mads.active
  assert not mads.selfdrive.events.has(EventName.buttonEnable)

  release_plus = _car_state(True, False, ButtonType.accelCruise, False)
  assert _step(mads, release_plus, button_enable=True) is False
  assert mads.enabled

  press_minus = _car_state(True, False, ButtonType.decelCruise, True)
  _step(mads, press_minus)
  assert mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)
  assert not mads.enabled and not mads.active

  release_minus = _car_state(True, False, ButtonType.decelCruise, False)
  assert _step(mads, release_minus, button_enable=True) is False
  assert not mads.enabled


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF6", "VINFAST_VF8"])
def test_second_set_plus_keeps_long_engage(mads_for, fingerprint):
  mads = mads_for(fingerprint)
  _step(mads, _car_state(True, False, ButtonType.accelCruise, True))
  _step(mads, _car_state(True, False, ButtonType.accelCruise, False), button_enable=True)
  assert mads.enabled

  _step(mads, _car_state(True, False, ButtonType.accelCruise, True))
  assert not mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)
  assert mads.enabled

  assert _step(mads, _car_state(True, False, ButtonType.accelCruise, False), button_enable=True) is True


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF6", "VINFAST_VF8"])
def test_set_buttons_do_not_toggle_mads_while_acc_controls(mads_for, fingerprint):
  mads = mads_for(fingerprint)
  _step(mads, _car_state(True, False, ButtonType.accelCruise, True))
  assert mads.enabled

  _step(mads, _car_state(True, True, ButtonType.accelCruise, True))
  assert not mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)
  assert mads.enabled

  _step(mads, _car_state(True, True, ButtonType.decelCruise, True))
  assert not mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)
  assert mads.enabled

  assert _step(mads, _car_state(True, True, ButtonType.decelCruise, False), button_enable=True) is True


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF6", "VINFAST_VF8"])
def test_acc_main_rising_does_not_arm_mads(mads_for, fingerprint):
  mads = mads_for(fingerprint)
  mads.selfdrive.CS_prev.cruiseState.available = False
  _step(mads, _car_state(True, False))
  assert not mads.selfdrive.events_sp.has(EventNameSP.lkasEnable)
  assert not mads.enabled


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF6", "VINFAST_VF8"])
def test_spd_plus_arms_without_acc_main_and_main_drop_keeps_mads(mads_for, fingerprint):
  mads = mads_for(fingerprint)
  mads.selfdrive.CS_prev.cruiseState.available = False
  _step(mads, _car_state(False, False, ButtonType.accelCruise, True))
  assert mads.selfdrive.events_sp.has(EventNameSP.lkasEnable)
  assert mads.enabled

  _step(mads, _car_state(False, False))
  assert not mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)
  assert mads.enabled


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF6", "VINFAST_VF8"])
def test_mfs_set_arms_mads_cold_without_starting_long(mads_for, fingerprint):
  # SpdSet feeds are often dead before the first OEM SET; MFS SET must arm MADS.
  mads = mads_for(fingerprint)
  press = _car_state(False, False, ButtonType.setCruise, True)
  _step(mads, press)
  assert mads.selfdrive.events_sp.has(EventNameSP.lkasEnable)
  assert mads.enabled

  release = _car_state(False, False, ButtonType.setCruise, False)
  assert _step(mads, release, button_enable=True) is False
  assert mads.enabled


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF6", "VINFAST_VF8"])
def test_mfs_set_keeps_long_engage_after_mads_is_on(mads_for, fingerprint):
  mads = mads_for(fingerprint)
  _step(mads, _car_state(False, False, ButtonType.setCruise, True))
  assert mads.enabled
  _step(mads, _car_state(False, False, ButtonType.setCruise, False), button_enable=True)

  _step(mads, _car_state(False, False, ButtonType.setCruise, True))
  assert not mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)
  assert mads.enabled
  assert _step(mads, _car_state(False, False, ButtonType.setCruise, False), button_enable=True) is True


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF6", "VINFAST_VF8"])
def test_mads_off_keeps_set_buttons_on_the_long_path(mads_for, fingerprint):
  mads = mads_for(fingerprint, mads_on=False)
  _step(mads, _car_state(True, False, ButtonType.accelCruise, True))
  assert not mads.selfdrive.events_sp.has(EventNameSP.lkasEnable)
  assert not mads.enabled

  assert _step(mads, _car_state(True, False, ButtonType.accelCruise, False), button_enable=True) is True
