#!/usr/bin/env python3
"""Experimental long keeps the model below 60 km/h and follows the cruise plan on an open highway."""
import pytest

from cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  LongitudinalPlanner, blend_experimental_accel, limit_vf_highway_cruise_brake)
from openpilot.selfdrive.test.longitudinal_maneuvers.test_vf_late_brake import build_sm

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
CITY = dict(v_ego=10.0, should_stop=False, lead_limiting=False)
HIGHWAY = dict(v_ego=25.0, should_stop=False, lead_limiting=False)


def test_city_keeps_a_timid_model_accel():
  # popv2-style, under 60 km/h: the cruise plan stays a ceiling.
  assert blend_experimental_accel(0.0, 0.6, **CITY) == 0.0
  assert blend_experimental_accel(0.05, 0.6, **CITY) == 0.05
  assert blend_experimental_accel(0.15, 0.6, **CITY) == 0.15


def test_highway_uses_the_cruise_plan_when_the_model_is_not_braking():
  assert blend_experimental_accel(0.0, 0.6, **HIGHWAY) == 0.6
  assert blend_experimental_accel(0.15, 0.6, **HIGHWAY) == 0.6


def test_above_set_speed_still_follows_cruise_brake():
  assert blend_experimental_accel(0.2, -0.4, **HIGHWAY) == -0.4
  assert blend_experimental_accel(0.2, -0.4, **CITY) == -0.4


def test_model_brake_wins_on_an_open_road():
  assert blend_experimental_accel(-0.05, 0.5, **CITY) == -0.05
  assert blend_experimental_accel(-0.3, 0.5, **HIGHWAY) == -0.3


def test_lead_keeps_the_lower_accel():
  held = dict(v_ego=25.0, should_stop=False, lead_limiting=True)
  assert blend_experimental_accel(0.4, 0.1, **held) == 0.1
  assert blend_experimental_accel(0.8, -0.4, **held) == -0.4
  assert blend_experimental_accel(-0.8, -0.2, **held) == -0.8


def test_stop_keeps_the_lower_accel():
  stopping = dict(v_ego=25.0, should_stop=True, lead_limiting=False)
  assert blend_experimental_accel(0.4, -0.5, **stopping) == -0.5
  assert blend_experimental_accel(0.4, 0.8, **stopping) == 0.4


def _planner(fingerprint, init_v=15.0):
  if fingerprint.startswith("VINFAST"):
    from opendbc.car.vinfast.interface import CarInterface
    from opendbc.car.vinfast.values import CAR
    car = getattr(CAR, fingerprint)
  else:
    from opendbc.car.honda.interface import CarInterface
    from opendbc.car.honda.values import CAR
    car = CAR.HONDA_CIVIC
  CP = CarInterface.get_non_essential_params(car)
  return LongitudinalPlanner(CP, CarInterface.get_non_essential_params_sp(CP, car), init_v=init_v)


def _settle(planner, sm, frames=16):
  for _ in range(frames):
    planner.update(sm)
  return planner


def _scene(v_ego, v_cruise, desired_a, *, lead_status=False, d_rel=200.0, v_lead=0.0,
           should_stop=False, e2e=True):
  # build_sm takes cruise in m/s and stores km/h. 15 m/s is under the 60 km/h legal-gap floor.
  return build_sm(v_ego=v_ego, d_rel=d_rel, v_lead=v_lead, path_x=80.0, desired_a=desired_a,
                  should_stop=should_stop, lead_status=lead_status, e2e=e2e, v_cruise=v_cruise)


def test_planner_open_road_keeps_a_zero_model_accel():
  planner = _settle(_planner("HONDA"), _scene(15.0, 25.0, 0.0))
  assert planner.output_a_target == pytest.approx(0.0, abs=0.05)
  assert planner.mpc.source == LongitudinalPlanSource.e2e
  assert not planner.output_should_stop


def test_planner_open_road_does_not_replace_a_timid_model():
  planner = _settle(_planner("HONDA"), _scene(15.0, 25.0, 0.2))
  assert planner.output_a_target == pytest.approx(0.2, abs=0.08)
  assert planner.output_a_target < 0.4


def test_planner_holds_model_brake_with_no_lead():
  planner = _settle(_planner("HONDA"), _scene(15.0, 25.0, -0.4))
  assert planner.output_a_target < -0.2
  assert planner.mpc.source == LongitudinalPlanSource.e2e


def test_planner_brakes_back_down_when_above_set_speed():
  planner = _settle(_planner("HONDA"), _scene(20.0, 15.0, 0.4), frames=20)
  assert planner.output_a_target < -0.05
  assert planner.mpc.source != LongitudinalPlanSource.e2e


def test_planner_does_not_accelerate_into_a_slower_lead():
  planner = _settle(_planner("HONDA"), _scene(15.0, 25.0, 0.8, lead_status=True, d_rel=18.0, v_lead=8.0))
  assert planner.output_a_target < 0.0
  assert planner.mpc.source != LongitudinalPlanSource.e2e


def test_planner_model_can_brake_harder_than_cruise_for_a_lead():
  planner = _settle(_planner("HONDA"), _scene(15.0, 25.0, -1.2, lead_status=True, d_rel=40.0, v_lead=14.5))
  assert planner.output_a_target < -0.5
  assert planner.mpc.source == LongitudinalPlanSource.e2e


def test_planner_stop_request_is_kept():
  planner = _settle(_planner("HONDA"), _scene(15.0, 25.0, 0.3, should_stop=True))
  assert planner.output_should_stop
  assert planner.output_a_target < 0.5


def test_planner_chill_mode_still_tracks_set_speed():
  planner = _settle(_planner("HONDA"), _scene(15.0, 25.0, 0.0, e2e=False))
  assert planner.output_a_target > 0.3
  assert planner.mpc.source != LongitudinalPlanSource.e2e


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF8", "VINFAST_VF6"])
def test_vinfast_open_road_keeps_the_model_and_still_brakes(fingerprint):
  holding = _settle(_planner(fingerprint), _scene(15.0, 25.0, 0.0))
  assert holding.output_a_target == pytest.approx(0.0, abs=0.05)
  assert holding.mpc.source == LongitudinalPlanSource.e2e

  braking = _settle(_planner(fingerprint), _scene(15.0, 25.0, -0.5))
  assert braking.output_a_target < -0.2
  assert braking.mpc.source == LongitudinalPlanSource.e2e


def test_highway_open_road_follows_the_cruise_plan():
  # 25 m/s is 90 km/h. A leveled-off model must not hold the car short of set speed.
  planner = _settle(_planner("HONDA", init_v=25.0), _scene(25.0, 35.0, 0.0))
  assert planner.output_a_target > 0.3
  assert planner.mpc.source != LongitudinalPlanSource.e2e


def test_highway_model_brake_still_wins():
  planner = _settle(_planner("HONDA", init_v=25.0), _scene(25.0, 35.0, -0.4))
  assert planner.output_a_target < -0.2
  assert planner.mpc.source == LongitudinalPlanSource.e2e


@pytest.mark.parametrize("fingerprint", ["VINFAST_VF8", "VINFAST_VF6"])
def test_vinfast_highway_reaches_toward_set_speed_and_still_brakes(fingerprint):
  holding = _settle(_planner(fingerprint, init_v=25.0), _scene(25.0, 35.0, 0.05))
  assert holding.output_a_target > 0.3
  assert holding.mpc.source != LongitudinalPlanSource.e2e

  braking = _settle(_planner(fingerprint, init_v=25.0), _scene(25.0, 35.0, -0.5))
  assert braking.output_a_target < -0.2
  assert braking.mpc.source == LongitudinalPlanSource.e2e


def test_vinfast_highway_cruise_brake_is_capped():
  # 100 km/h, set speed well below. The cruise plan wants about -1.2; publish a light brake.
  assert limit_vf_highway_cruise_brake(-2.0, 28.0, vinfast=True, cruise=True,
                                        should_stop=False, fcw=False, red_light=False) == pytest.approx(-0.8)
  # Under 65 km/h, and any lead, stop, or non-VinFast car, the hard request stands.
  assert limit_vf_highway_cruise_brake(-2.0, 15.0, vinfast=True, cruise=True,
                                        should_stop=False, fcw=False, red_light=False) == -2.0
  assert limit_vf_highway_cruise_brake(-2.0, 28.0, vinfast=True, cruise=False,
                                        should_stop=False, fcw=False, red_light=False) == -2.0
  assert limit_vf_highway_cruise_brake(-2.0, 28.0, vinfast=False, cruise=True,
                                        should_stop=False, fcw=False, red_light=False) == -2.0
  assert limit_vf_highway_cruise_brake(-2.0, 28.0, vinfast=True, cruise=True,
                                        should_stop=True, fcw=False, red_light=False) == -2.0


def test_vinfast_highway_overspeed_does_not_brake_hard():
  planner = _settle(_planner("VINFAST_VF8", init_v=28.0), _scene(28.0, 22.0, 0.2), frames=20)
  assert planner.output_a_target < -0.2
  assert planner.output_a_target >= -0.85
  assert planner.mpc.source == LongitudinalPlanSource.cruise


def test_highway_does_not_accelerate_into_a_slower_lead():
  planner = _settle(_planner("VINFAST_VF8", init_v=25.0),
                    _scene(25.0, 35.0, 0.8, lead_status=True, d_rel=40.0, v_lead=20.0))
  assert planner.output_a_target < 0.15
  assert planner.mpc.source != LongitudinalPlanSource.e2e
