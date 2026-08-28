#!/usr/bin/env python3
"""VF8/VF9 late-brake behavior: the softening must not push stops deeper than planned."""
from types import SimpleNamespace

import numpy as np
import pytest

import cereal.messaging as messaging
from cereal import log
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  VF_MILD_DECEL_FLOOR, VF_MILD_DECEL_SCALE, VF_REDLIGHT_ENGAGE_FRAMES, VF_REDLIGHT_HOLD_FRAMES,
  VF_STOP_LEAD_GAP_M, VF67_STOP_LEAD_GAP_M, LongitudinalPlanner, is_vf_red_light_slowdown,
  vf_mild_decel_scale, vf_stop_lead_adjust_m, vf_stop_lead_gap_m)
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_LEAD_MIN_GAP
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

# The flat scale this fork used before, kept here as the regression baseline.
OLD_MILD_DECEL_SCALE = 0.82


def vf_car_params():
  from opendbc.car.vinfast.values import CAR
  from opendbc.car.vinfast.interface import CarInterface
  CP = CarInterface.get_non_essential_params(CAR.VINFAST_VF8)
  return CP, CarInterface.get_non_essential_params_sp(CP, CAR.VINFAST_VF8)


def build_sm(v_ego, d_rel, v_lead, path_x, desired_a, brake_prob=0.0, should_stop=False,
             lead_status=True, e2e=False, personality=0, v_cruise=60.0):
  """Assemble the messages the planner reads. Nothing is published to a socket."""
  radar = messaging.new_message('radarState')
  lead = log.RadarState.LeadData.new_message()
  lead.dRel = float(d_rel)
  lead.vRel = float(v_lead - v_ego)
  lead.vLead = float(v_lead)
  lead.vLeadK = float(v_lead)
  lead.aLeadK = 0.0
  lead.aLeadTau = float(_LEAD_ACCEL_TAU)
  lead.status = bool(lead_status)
  lead.modelProb = 1.0 if lead_status else 0.0
  radar.radarState.leadOne = lead
  radar.radarState.leadTwo = lead

  model = messaging.new_message('modelV2')
  # A model that predicts holding the current speed, so the stop comes from the lead.
  position = log.XYZTData.new_message()
  position.x = [float(x) for x in np.linspace(0.0, path_x, ModelConstants.IDX_N)]
  model.modelV2.position = position
  velocity = log.XYZTData.new_message()
  velocity.x = [float(v_ego) for _ in ModelConstants.T_IDXS]
  model.modelV2.velocity = velocity
  accel = log.XYZTData.new_message()
  accel.x = [0.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.acceleration = accel
  # Straight path: smart cruise control reads these to bound lateral accel.
  for field in ('orientation', 'orientationRate'):
    data = log.XYZTData.new_message()
    data.x = data.y = data.z = [0.0 for _ in ModelConstants.T_IDXS]
    setattr(model.modelV2, field, data)
  model.modelV2.action.desiredAcceleration = float(desired_a)
  model.modelV2.action.shouldStop = bool(should_stop)
  model.modelV2.meta.disengagePredictions.gasPressProbs = [1.0] * 6
  model.modelV2.meta.disengagePredictions.brakePressProbs = [0.0, float(brake_prob)] + [0.0] * 4

  car_state = messaging.new_message('carState')
  car_state.carState.vEgo = float(v_ego)
  car_state.carState.standstill = bool(v_ego < 0.01)
  car_state.carState.vCruise = float(v_cruise * 3.6)

  control = messaging.new_message('controlsState')
  control.controlsState.longControlState = LongCtrlState.pid
  ss = messaging.new_message('selfdriveState')
  ss.selfdriveState.experimentalMode = bool(e2e)
  ss.selfdriveState.personality = int(personality)
  ss.selfdriveState.enabled = True

  car_control = messaging.new_message('carControl')
  car_control.carControl.orientationNED = [0.0, 0.0, 0.0]
  # Left disabled so smart cruise control and speed limit assist stay out of the loop;
  # the planner's own engagement comes from selfdriveState and controlsState.

  return {
    'radarState': radar.radarState,
    'carState': car_state.carState,
    'carControl': car_control.carControl,
    'controlsState': control.controlsState,
    'selfdriveState': ss.selfdriveState,
    'liveParameters': messaging.new_message('liveParameters').liveParameters,
    'modelV2': model.modelV2,
    'carStateSP': messaging.new_message('carStateSP').carStateSP,
    'liveMapDataSP': messaging.new_message('liveMapDataSP').liveMapDataSP,
    # Both names: which one the speed limit resolver reads depends on the device.
    'gpsLocation': messaging.new_message('gpsLocation').gpsLocation,
    'gpsLocationExternal': messaging.new_message('gpsLocationExternal').gpsLocationExternal,
  }


class TestVFMildDecelScale:
  def test_full_softening_at_onset(self):
    assert vf_mild_decel_scale(-0.05) == pytest.approx(VF_MILD_DECEL_SCALE)

  def test_no_softening_past_floor(self):
    for a in (VF_MILD_DECEL_FLOOR, -2.0, -3.5):
      assert vf_mild_decel_scale(a) == pytest.approx(1.0)

  def test_continuous_across_floor(self):
    """The old flat cut stepped 0.31 m/s² at the floor; that step is what grabbed late."""
    just_under = -1.599 * vf_mild_decel_scale(-1.599)
    just_past = -1.601 * vf_mild_decel_scale(-1.601)
    assert abs(just_past - just_under) < 0.01

    old_under = -1.599 * OLD_MILD_DECEL_SCALE
    assert abs(-1.601 - old_under) > 0.25

  def test_monotonic_command(self):
    """A larger request must always command more braking."""
    a = np.arange(-3.5, 0.0, 0.01)
    commanded = np.array([x * vf_mild_decel_scale(x) for x in a])
    assert np.all(np.diff(commanded) > 0)

  def test_planned_stop_distance_preserved(self):
    """Distance inflation is what put the car close to the object."""
    planned = 1.5
    new_inflation = 1.0 / vf_mild_decel_scale(-planned)
    old_inflation = 1.0 / OLD_MILD_DECEL_SCALE
    assert new_inflation < 1.03
    assert old_inflation > 1.20


class TestVFRedLightHeuristic:
  def setup_method(self):
    self.CP, _ = vf_car_params()

  def test_detects_empty_red_light_during_approach(self):
    # 40 km/h with a 40 m predicted path: the old flat 28 m gate missed this.
    sm = build_sm(v_ego=11.0, d_rel=200.0, v_lead=0.0, path_x=40.0, desired_a=-0.9,
                  brake_prob=0.7, lead_status=False)
    assert is_vf_red_light_slowdown(sm, self.CP)

  def test_detects_stop_behind_stopped_car(self):
    """A queue at a light is a stop, not ordinary following."""
    sm = build_sm(v_ego=8.0, d_rel=12.0, v_lead=0.0, path_x=25.0, desired_a=-0.9, brake_prob=0.7)
    assert is_vf_red_light_slowdown(sm, self.CP)

  def test_ignores_ordinary_following(self):
    sm = build_sm(v_ego=11.0, d_rel=12.0, v_lead=8.0, path_x=25.0, desired_a=-0.9, brake_prob=0.7)
    assert not is_vf_red_light_slowdown(sm, self.CP)

  def test_ignores_green_coast(self):
    sm = build_sm(v_ego=11.0, d_rel=200.0, v_lead=0.0, path_x=110.0, desired_a=-0.2,
                  brake_prob=0.2, lead_status=False)
    assert not is_vf_red_light_slowdown(sm, self.CP)

  def test_ignores_other_brands(self):
    from opendbc.car.honda.values import CAR
    from opendbc.car.honda.interface import CarInterface
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    sm = build_sm(v_ego=11.0, d_rel=200.0, v_lead=0.0, path_x=40.0, desired_a=-0.9,
                  brake_prob=0.7, lead_status=False)
    assert not is_vf_red_light_slowdown(sm, CP)


class TestVFRedLightDebounce:
  def setup_method(self):
    self.CP, self.CP_SP = vf_car_params()
    self.planner = LongitudinalPlanner(self.CP, self.CP_SP)

  def _sm(self, red_light):
    if red_light:
      return build_sm(v_ego=8.0, d_rel=200.0, v_lead=0.0, path_x=25.0, desired_a=-0.9,
                      brake_prob=0.7, lead_status=False)
    return build_sm(v_ego=8.0, d_rel=200.0, v_lead=0.0, path_x=80.0, desired_a=-0.1,
                    brake_prob=0.0, lead_status=False)

  def test_requires_agreement_to_engage(self):
    held = [self.planner.vf_red_light_committed(self._sm(True)) for _ in range(VF_REDLIGHT_ENGAGE_FRAMES)]
    assert held == [False] * (VF_REDLIGHT_ENGAGE_FRAMES - 1) + [True]

  def test_holds_through_single_frame_dropout(self):
    for _ in range(VF_REDLIGHT_ENGAGE_FRAMES):
      self.planner.vf_red_light_committed(self._sm(True))
    assert self.planner.vf_red_light_committed(self._sm(False))

  def test_releases_after_hold(self):
    for _ in range(VF_REDLIGHT_ENGAGE_FRAMES):
      self.planner.vf_red_light_committed(self._sm(True))
    held = [self.planner.vf_red_light_committed(self._sm(False)) for _ in range(VF_REDLIGHT_HOLD_FRAMES + 2)]
    assert held[0] and not held[-1]

  def test_standstill_clears(self):
    for _ in range(VF_REDLIGHT_ENGAGE_FRAMES):
      self.planner.vf_red_light_committed(self._sm(True))
    assert not self.planner.vf_red_light_committed(build_sm(v_ego=0.0, d_rel=5.0, v_lead=0.0,
                                                            path_x=5.0, desired_a=-0.5))


class ApproachSim:
  """Closed-loop approach to a stopped car through the real planner, MPC and LongControl.

  The actuator is ideal, so the resulting gap is the gap the control chain is aiming
  for rather than a prediction of what the car achieves on the road.
  """

  def __init__(self, mild_scale=None, stop_adjust=None, v_ego=14.0, d_rel=90.0):
    import openpilot.selfdrive.controls.lib.longitudinal_planner as lp
    self.lp = lp
    self.mild_scale = mild_scale
    CP, CP_SP = vf_car_params()
    self.CP = CP
    self.planner = LongitudinalPlanner(CP, CP_SP, init_v=v_ego)
    self.LoC = LongControl(CP, CP_SP)
    if stop_adjust is not None:
      self.planner.vf_stop_lead_adjust_override = float(stop_adjust)
    self.v_ego = v_ego
    self.d_rel = d_rel
    self.a_ego = 0.0
    self.brake_onset_d = None
    self.peak_decel = 0.0
    self.stopped_frames = 0
    self.min_gap = d_rel
    self.trace = []

  def _car_state(self):
    return SimpleNamespace(vEgo=self.v_ego, aEgo=self.a_ego, brakePressed=False,
                           standstill=self.v_ego < 0.01,
                           cruiseState=SimpleNamespace(standstill=False))

  def run(self, duration=40.0):
    orig = self.lp.vf_mild_decel_scale
    if self.mild_scale is not None:
      self.lp.vf_mild_decel_scale = lambda a: self.mild_scale if a > VF_MILD_DECEL_FLOOR else 1.0
    try:
      for _ in range(int(duration / DT_MDL)):
        sm = build_sm(v_ego=self.v_ego, d_rel=self.d_rel, v_lead=0.0,
                      path_x=max(self.v_ego, 1.0) * ModelConstants.T_IDXS[-1],
                      desired_a=self.a_ego + 0.1)
        self.planner.update(sm)

        # LongControl runs at DT_CTRL; its stopping state is what finishes the stop.
        for _ in range(int(DT_MDL / DT_CTRL)):
          self.a_ego = float(self.LoC.update(True, self._car_state(), self.planner.output_a_target,
                                             self.planner.output_should_stop, (ACCEL_MIN, ACCEL_MAX)))
          if self.brake_onset_d is None and self.a_ego < -0.5:
            self.brake_onset_d = self.d_rel
          # Only while moving: the stopping ramp keeps pushing toward stopAccel at standstill.
          if self.v_ego > 0.5:
            self.peak_decel = min(self.peak_decel, self.a_ego)
          self.v_ego = max(0.0, self.v_ego + self.a_ego * DT_CTRL)
          self.d_rel = max(0.0, self.d_rel - self.v_ego * DT_CTRL)

        self.min_gap = min(self.min_gap, self.d_rel)
        self.trace.append((self.d_rel, self.v_ego))
        self.stopped_frames = self.stopped_frames + 1 if self.v_ego <= 0.0 else 0
        if self.stopped_frames > int(1.0 / DT_MDL):
          break
    finally:
      self.lp.vf_mild_decel_scale = orig
    return self

  def speed_at_gap(self, gap):
    """Speed when the lead was this far away: under-braking shows up as arriving faster."""
    trace = np.array(self.trace)
    return float(np.interp(gap, trace[:, 0][::-1], trace[:, 1][::-1]))


class TestVFApproachStoppedCar:
  """The reported symptom: stopping too close to the object at a light."""

  @pytest.fixture(scope="class")
  def runs(self):
    results = {
      'new': ApproachSim().run(),
      'old': ApproachSim(mild_scale=OLD_MILD_DECEL_SCALE, stop_adjust=4.0).run(),
      'stock': ApproachSim(mild_scale=1.0, stop_adjust=0.0).run(),
    }
    print("\napproach a stopped car from 14 m/s, 90 m away:")
    for name, sim in results.items():
      fields = [
        f"final gap {sim.d_rel:5.2f} m",
        f"brake onset {sim.brake_onset_d:6.2f} m",
        f"peak decel {sim.peak_decel:5.2f} m/s²",
        f"speed at 40/20 m {sim.speed_at_gap(40):5.2f}/{sim.speed_at_gap(20):4.2f} m/s",
      ]
      print(f"  {name:5s} " + "  ".join(fields))
    return results

  def test_comes_to_a_stop(self, runs):
    assert runs['new'].v_ego == pytest.approx(0.0, abs=0.01)

  def test_keeps_a_gap(self, runs):
    assert STOP_LEAD_MIN_GAP <= runs['new'].d_rel <= 8.0
    assert runs['new'].min_gap == runs['new'].d_rel  # never closed in further than it stopped

  def test_brakes_no_harder_than_comfortable(self, runs):
    assert runs['new'].peak_decel > -2.5

  def test_approaches_slower_than_the_flat_cut(self, runs):
    """The flat 0.82 cut let the car arrive faster at every remaining distance."""
    for gap in (40.0, 20.0):
      assert runs['new'].speed_at_gap(gap) < runs['old'].speed_at_gap(gap)

  def test_needs_less_peak_braking_than_the_flat_cut(self, runs):
    assert runs['new'].peak_decel > runs['old'].peak_decel

  def test_tracks_stock_control(self, runs):
    """With the fade, the approach is back within a few percent of stock openpilot."""
    for gap in (40.0, 20.0):
      assert runs['new'].speed_at_gap(gap) == pytest.approx(runs['stock'].speed_at_gap(gap), rel=0.03)


class RedLightSim:
  """Blended-mode stop at a line, driven by a model that plans the stop itself.

  This is the path a red light actually takes on PMV2: no lead, and the planner
  output is the lower of the MPC and the model's own requested acceleration.
  """

  def __init__(self, v_ego=13.9, stopline=70.0):
    CP, CP_SP = vf_car_params()
    self.planner = LongitudinalPlanner(CP, CP_SP, init_v=v_ego)
    self.LoC = LongControl(CP, CP_SP)
    self.v_ego = v_ego
    self.a_ego = 0.0
    self.travelled = 0.0
    self.stopline = stopline
    self.peak_decel = 0.0
    self.committed_frames = 0

  def _model_plan(self):
    """What a model planning a stop at the line would publish."""
    d = max(self.stopline - self.travelled, 0.5)
    a_req = float(np.clip(-self.v_ego ** 2 / (2 * d), -3.0, 0.0)) if self.v_ego > 0.1 else 0.0
    t = np.array(ModelConstants.T_IDXS)
    v_plan = np.clip(self.v_ego + a_req * t, 0.0, None)
    x_plan = np.minimum(np.cumsum(np.diff(t, prepend=0.0) * v_plan), d)
    return d, a_req, v_plan, x_plan

  def run(self, duration=40.0):
    for i in range(int(duration / DT_MDL)):
      d, a_req, v_plan, x_plan = self._model_plan()
      sm = build_sm(v_ego=self.v_ego, d_rel=200.0, v_lead=0.0, path_x=d, desired_a=a_req,
                    brake_prob=0.7 if a_req < -0.3 else 0.05,
                    should_stop=bool(v_plan[3] < 0.5), lead_status=False, e2e=True)
      sm['modelV2'].velocity.x = [float(v) for v in v_plan]
      sm['modelV2'].position.x = [float(x) for x in x_plan]
      self.planner.update(sm)
      if self.planner.vf_redlight_hold > 0:
        self.committed_frames += 1

      for _ in range(int(DT_MDL / DT_CTRL)):
        self.a_ego = float(self.LoC.update(True, SimpleNamespace(
          vEgo=self.v_ego, aEgo=self.a_ego, brakePressed=False, standstill=self.v_ego < 0.01,
          cruiseState=SimpleNamespace(standstill=False)),
          self.planner.output_a_target, self.planner.output_should_stop, (ACCEL_MIN, ACCEL_MAX)))
        if self.v_ego > 0.5:
          self.peak_decel = min(self.peak_decel, self.a_ego)
        self.v_ego = max(0.0, self.v_ego + self.a_ego * DT_CTRL)
        self.travelled += self.v_ego * DT_CTRL

      if self.v_ego <= 0.0 and i * DT_MDL > 3.0:
        break
    return self

  @property
  def overshoot(self):
    return self.travelled - self.stopline


class TestVFRedLightStop:
  @pytest.fixture(scope="class")
  def sim(self):
    sim = RedLightSim().run()
    fields = [
      f"stopped {sim.overshoot:+.2f} m from the line",
      f"peak decel {sim.peak_decel:.2f} m/s²",
      f"committed for {sim.committed_frames} frames",
    ]
    print("\nred light stop from 13.9 m/s, line 70 m ahead: " + ", ".join(fields))
    return sim

  def test_stops_at_the_line(self, sim):
    assert abs(sim.overshoot) < 2.0

  def test_brakes_no_harder_than_comfortable(self, sim):
    assert sim.peak_decel > -2.5

  def test_commits_during_the_approach(self, sim):
    """The old gates left this at zero, so the approach only ever got the softening."""
    assert sim.committed_frames > int(2.0 / DT_MDL)


class TestVFStopLeadGap:
  def test_vf8_personality_gaps(self):
    assert VF_STOP_LEAD_GAP_M[int(log.LongitudinalPersonality.aggressive)] == 4.0
    assert VF_STOP_LEAD_GAP_M[int(log.LongitudinalPersonality.standard)] == 5.0
    assert VF_STOP_LEAD_GAP_M[int(log.LongitudinalPersonality.relaxed)] == 6.0
    assert vf_stop_lead_gap_m(log.LongitudinalPersonality.aggressive, "VINFAST_VF8") == 4.0
    assert vf_stop_lead_adjust_m(log.LongitudinalPersonality.aggressive, "VINFAST_VF9") == pytest.approx(2.0)
    assert vf_stop_lead_adjust_m(log.LongitudinalPersonality.standard, "VINFAST_VF8") == pytest.approx(1.0)
    assert vf_stop_lead_adjust_m(log.LongitudinalPersonality.relaxed, "VINFAST_VF9") == pytest.approx(0.0)

  def test_vf67_widens_standstill(self):
    for fp in ("VINFAST_VF6", "VINFAST_VF7"):
      assert vf_stop_lead_gap_m(log.LongitudinalPersonality.aggressive, fp) == 6.0
      assert vf_stop_lead_adjust_m(log.LongitudinalPersonality.aggressive, fp) == pytest.approx(0.0)
      assert vf_stop_lead_gap_m(log.LongitudinalPersonality.standard, fp) == 7.0
      assert vf_stop_lead_adjust_m(log.LongitudinalPersonality.standard, fp) == pytest.approx(-1.0)
      assert vf_stop_lead_gap_m(log.LongitudinalPersonality.relaxed, fp) == 8.0
    assert VF67_STOP_LEAD_GAP_M[int(log.LongitudinalPersonality.relaxed)] == 8.0
