from cereal import car, custom

from openpilot.common.constants import CV
from openpilot.selfdrive.car import cruise as cruise_mod
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.car import vf_car_set_speed as choice


def test_only_vf8_and_vf9(tmp_path):
  assert choice.applies_to("VINFAST_VF8")
  assert choice.applies_to("VINFAST_VF9")
  assert choice.applies_to("VINFAST_VF8_ECO")
  assert not choice.applies_to("VINFAST_VF6")
  assert not choice.applies_to("VINFAST_VF8_PLUS")
  assert not choice.applies_to("VINFAST_VF7")


def test_missing_choice_is_off(tmp_path):
  assert choice.car_set_speed_enabled(tmp_path / "missing") is False


def test_vf8_follows_the_car_when_the_switch_is_on(tmp_path, monkeypatch):
  store = tmp_path / "VinFastCarSetSpeed"
  choice.set_car_set_speed(True, store)
  monkeypatch.setattr(cruise_mod, "car_set_speed_enabled", lambda path=None: True)

  CP = car.CarParams(brand="vinfast", carFingerprint="VINFAST_VF8", pcmCruise=False)
  CP_SP = custom.CarParamsSP(pcmCruiseSpeed=True)
  helper = VCruiseHelper(CP, CP_SP)
  assert helper._vf8_vf9
  assert not helper.vf6_platform

  CS = car.CarState(cruiseState={"available": True, "speed": 20.0, "speedCluster": 20.0})
  helper.update_v_cruise(CS, enabled=True, is_metric=True)
  assert helper.v_cruise_kph == 20.0 * CV.MS_TO_KPH


def test_vf8_keeps_its_own_set_speed_when_the_switch_is_off(tmp_path, monkeypatch):
  monkeypatch.setattr(cruise_mod, "car_set_speed_enabled", lambda path=None: False)
  CP = car.CarParams(brand="vinfast", carFingerprint="VINFAST_VF8", pcmCruise=False)
  CP_SP = custom.CarParamsSP(pcmCruiseSpeed=True)
  helper = VCruiseHelper(CP, CP_SP)
  helper.initialize_v_cruise(car.CarState(vEgo=10.0), experimental_mode=False, dynamic_experimental_control=False)
  assert helper.v_cruise_initialized
