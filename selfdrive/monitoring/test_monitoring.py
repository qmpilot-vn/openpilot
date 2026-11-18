import sys
from unittest.mock import MagicMock

import numpy as np
import pytest

from cereal import log, car
from openpilot.common.realtime import DT_DMON


def _install_host_mocks():
  # Device .so files are aarch64; x86 hosts need stubs to import policy.py.
  if 'openpilot.common.params_pyx' not in sys.modules:
    fake = MagicMock()
    class Params:
      def get_bool(self, key):
        return False
    fake.Params = Params
    fake.UnknownKeyName = type('UnknownKeyName', (Exception,), {})
    sys.modules['openpilot.common.params_pyx'] = fake
  if 'cereal.messaging' not in sys.modules:
    msg = MagicMock()
    def new_message(*args, **kwargs):
      ev = MagicMock()
      ev.driverMonitoringState = MagicMock()
      return ev
    msg.new_message = new_message
    sys.modules['cereal.messaging'] = msg


try:
  from openpilot.selfdrive.monitoring.policy import DriverMonitoring, DRIVER_MONITOR_SETTINGS
except ImportError:
  _install_host_mocks()
  from openpilot.selfdrive.monitoring.policy import DriverMonitoring, DRIVER_MONITOR_SETTINGS

EventName = log.OnroadEvent.EventName
dm_settings = DRIVER_MONITOR_SETTINGS()

TEST_TIMESPAN = 240  # seconds
DISTRACTED_SECONDS_TO_ORANGE = dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 3
DISTRACTED_SECONDS_TO_RED = dm_settings._VISION_POLICY_ALERT_3_TIMEOUT + 3
INVISIBLE_SECONDS_TO_ORANGE = dm_settings._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT + 3
INVISIBLE_SECONDS_TO_RED = dm_settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT + 3

def make_msg(face_detected, distracted=False, model_uncertain=False, face_prob=None,
             phone=0., yaw=0., blink=None):
  ds = log.DriverStateV2.new_message()
  ds.leftDriverData.faceOrientation = [0., yaw, 0.]
  ds.leftDriverData.facePosition = [0., 0.]
  ds.leftDriverData.faceProb = float(face_detected if face_prob is None else face_prob)
  ds.leftDriverData.leftEyeProb = 1.
  ds.leftDriverData.rightEyeProb = 1.
  blink_v = (1. * distracted) if blink is None else blink
  ds.leftDriverData.leftBlinkProb = blink_v
  ds.leftDriverData.rightBlinkProb = blink_v
  ds.leftDriverData.faceOrientationStd = [1.*model_uncertain, 1.*model_uncertain, 1.*model_uncertain]
  ds.leftDriverData.facePositionStd = [1.*model_uncertain, 1.*model_uncertain]
  ds.leftDriverData.phoneProb = phone
  return ds


# driver state from neural net, 10Hz
msg_NO_FACE_DETECTED = make_msg(False)
msg_ATTENTIVE = make_msg(True)
msg_DISTRACTED = make_msg(True, distracted=True)
msg_ATTENTIVE_UNCERTAIN = make_msg(True, model_uncertain=True)
msg_DISTRACTED_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=True)
msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=dm_settings._HI_STD_THRESHOLD*1.5)

# driver interaction with car
car_interaction_DETECTED = True
car_interaction_NOT_DETECTED = False

# some common state vectors
always_no_face = [msg_NO_FACE_DETECTED] * int(TEST_TIMESPAN / DT_DMON)
always_attentive = [msg_ATTENTIVE] * int(TEST_TIMESPAN / DT_DMON)
always_distracted = [msg_DISTRACTED] * int(TEST_TIMESPAN / DT_DMON)
always_true = [True] * int(TEST_TIMESPAN / DT_DMON)
always_false = [False] * int(TEST_TIMESPAN / DT_DMON)

class TestMonitoring:
  def _run_seq(self, msgs, interaction, engaged, standstill):
    DM = DriverMonitoring()
    alert_lvls = []
    for idx in range(len(msgs)):
      DM._update_states(msgs[idx], [0, 0, 0], 0, engaged[idx], standstill[idx])
      # cal_rpy and car_speed don't matter here

      # evaluate events at 10Hz for tests
      DM._update_events(interaction[idx], engaged[idx], standstill[idx], 0)
      alert_lvls.append(DM.alert_level)
    assert len(alert_lvls) == len(msgs), f"got {len(alert_lvls)} for {len(msgs)} driverState input msgs"
    return alert_lvls, DM


  # engaged, driver is attentive all the time
  def test_fully_aware_driver(self):
    alert_lvls, d_status = self._run_seq(always_attentive, always_false, always_true, always_false)
    assert all(a == 0 for a in alert_lvls)
    assert d_status.active_policy == log.DriverMonitoringState.MonitoringPolicy.vision

  # engaged, driver is distracted and does nothing
  def test_fully_distracted_driver(self):
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true, always_false)
    s = d_status.settings
    assert alert_lvls[int(s._VISION_POLICY_ALERT_1_TIMEOUT / 2 / DT_DMON)] == 0
    assert alert_lvls[int((s._VISION_POLICY_ALERT_1_TIMEOUT + \
                    (s._VISION_POLICY_ALERT_2_TIMEOUT - s._VISION_POLICY_ALERT_1_TIMEOUT) / 2) / DT_DMON)] == 0
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + \
                    (s._VISION_POLICY_ALERT_3_TIMEOUT - s._VISION_POLICY_ALERT_2_TIMEOUT) / 2) / DT_DMON)] == 2
    assert alert_lvls[int((s._VISION_POLICY_ALERT_3_TIMEOUT + \
                    (TEST_TIMESPAN - 10 - s._VISION_POLICY_ALERT_3_TIMEOUT) / 2) / DT_DMON)] == 3
    assert isinstance(d_status.awareness, float)

  # engaged, no face detected the whole time, no action
  def test_fully_invisible_driver(self):
    alert_lvls, d_status = self._run_seq(always_no_face, always_false, always_true, always_false)
    s = d_status.settings
    assert alert_lvls[int(s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT / 2 / DT_DMON)] == 0
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT + \
                    (s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT - s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT) / 2) / DT_DMON)] == 0
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT + \
                    (s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT - s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT) / 2) / DT_DMON)] == 2
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT + \
                    (TEST_TIMESPAN - 10 - s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT) / 2) / DT_DMON)] == 3
    assert d_status.active_policy == log.DriverMonitoringState.MonitoringPolicy.wheeltouch

  # engaged, down to orange, driver pays attention, back to normal; then down to orange, driver touches wheel
  #  - should have short orange recovery time and no green afterwards; wheel touch only recovers when paying attention
  def test_normal_driver(self):
    ds_vector = [msg_DISTRACTED] * int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON) + \
                [msg_ATTENTIVE] * int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON) + \
                [msg_DISTRACTED] * int((DISTRACTED_SECONDS_TO_ORANGE+2)/DT_DMON) + \
                [msg_ATTENTIVE] * (int(TEST_TIMESPAN/DT_DMON)-int((DISTRACTED_SECONDS_TO_ORANGE*3+2)/DT_DMON))
    interaction_vector = [car_interaction_NOT_DETECTED] * int(DISTRACTED_SECONDS_TO_ORANGE*3/DT_DMON) + \
                         [car_interaction_DETECTED] * (int(TEST_TIMESPAN/DT_DMON)-int(DISTRACTED_SECONDS_TO_ORANGE*3/DT_DMON))
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, always_true, always_false)
    assert alert_lvls[int(DISTRACTED_SECONDS_TO_ORANGE*0.5/DT_DMON)] == 0
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
    assert alert_lvls[int(DISTRACTED_SECONDS_TO_ORANGE*1.5/DT_DMON)] == 0
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3-0.1)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3+0.1)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3+4.0)/DT_DMON)] == 0

  # engaged, down to orange, driver dodges camera, then comes back still distracted, down to red, \
  #                          driver dodges, and then touches wheel to no avail, disengages and reengages
  #  - orange/red alert should remain after disappearance, and only disengaging clears red
  def test_biggest_comma_fan(self):
    _invisible_time = 2  # seconds
    ds_vector = always_distracted[:]
    interaction_vector = always_false[:]
    op_vector = always_true[:]
    ds_vector[int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON):int((DISTRACTED_SECONDS_TO_ORANGE+_invisible_time)/DT_DMON)] \
                                                        = [msg_NO_FACE_DETECTED] * int(_invisible_time/DT_DMON)
    ds_vector[int((DISTRACTED_SECONDS_TO_RED+_invisible_time)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time)/DT_DMON)] \
                                                        = [msg_NO_FACE_DETECTED] * int(_invisible_time/DT_DMON)
    interaction_vector[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+0.5)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+1.5)/DT_DMON)] \
                                                        = [True] * int(1/DT_DMON)
    op_vector[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+2.5)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+3)/DT_DMON)] \
                                                        = [False] * int(0.5/DT_DMON)
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, op_vector, always_false)
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE+0.5*_invisible_time)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+1.5*_invisible_time)/DT_DMON)] == 3
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+1.5)/DT_DMON)] == 3
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+3.5)/DT_DMON)] == 0

  # engaged, invisible driver, down to orange, driver touches wheel; then down to orange again, driver appears
  #  - both actions should clear the alert, but momentary appearance should not
  def test_sometimes_transparent_commuter(self):
    _visible_time = np.random.choice([0.5, 10])
    ds_vector = always_no_face[:]*2
    interaction_vector = always_false[:]*2
    ds_vector[int((2*INVISIBLE_SECONDS_TO_ORANGE+1)/DT_DMON):int((2*INVISIBLE_SECONDS_TO_ORANGE+1+_visible_time)/DT_DMON)] = \
                                                                                             [msg_ATTENTIVE] * int(_visible_time/DT_DMON)
    interaction_vector[int((INVISIBLE_SECONDS_TO_ORANGE)/DT_DMON):int((INVISIBLE_SECONDS_TO_ORANGE+1)/DT_DMON)] = [True] * int(1/DT_DMON)
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, 2*always_true, 2*always_false)
    assert alert_lvls[int(INVISIBLE_SECONDS_TO_ORANGE*0.5/DT_DMON)] == 0
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE+0.1)/DT_DMON)] == 0
    if _visible_time == 0.5:
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1-0.1)/DT_DMON)] == 2
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1+0.1+_visible_time)/DT_DMON)] == 2
    elif _visible_time == 10:
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1-0.1)/DT_DMON)] == 2
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1+0.1+_visible_time)/DT_DMON)] == 0

  # engaged, invisible driver, down to red, driver appears and then touches wheel, then disengages/reengages
  #  - only disengage will clear the alert
  def test_last_second_responder(self):
    _visible_time = 2  # seconds
    ds_vector = always_no_face[:]
    interaction_vector = always_false[:]
    op_vector = always_true[:]
    ds_vector[int(INVISIBLE_SECONDS_TO_RED/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time)/DT_DMON)] = [msg_ATTENTIVE] * int(_visible_time/DT_DMON)
    interaction_vector[int((INVISIBLE_SECONDS_TO_RED+_visible_time)/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time+1)/DT_DMON)] = [True] * int(1/DT_DMON)
    op_vector[int((INVISIBLE_SECONDS_TO_RED+_visible_time+1)/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time+1.5)/DT_DMON)] = [False] * int(0.5/DT_DMON)
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, op_vector, always_false)
    assert alert_lvls[int(INVISIBLE_SECONDS_TO_ORANGE*0.5/DT_DMON)] == 0
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED-0.1)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+0.5*_visible_time)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+_visible_time+0.5)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+_visible_time+1+0.1)/DT_DMON)] == 0

  # disengaged, always distracted driver
  #  - dm should stay quiet when not engaged
  def test_pure_dashcam_user(self):
    alert_lvls, _ = self._run_seq(always_distracted, always_false, always_false, always_false)
    assert all(a == 0 for a in alert_lvls)

  # engaged, car stops at traffic light, down to orange, no action, then car starts moving
  #  - should only reach green when stopped, but continues counting down on launch
  def test_long_traffic_light_victim(self):
    _redlight_time = 60  # seconds
    standstill_vector = always_true[:]
    standstill_vector[int(_redlight_time/DT_DMON):] = [False] * int((TEST_TIMESPAN-_redlight_time)/DT_DMON)
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true, standstill_vector)
    s = d_status.settings
    assert alert_lvls[int((_redlight_time-0.1)/DT_DMON)] == 0
    _alert_1_to_2 = s._VISION_POLICY_ALERT_2_TIMEOUT - s._VISION_POLICY_ALERT_1_TIMEOUT
    assert alert_lvls[int((_redlight_time+0.5)/DT_DMON)] == 0
    assert alert_lvls[int((_redlight_time+_alert_1_to_2+0.5)/DT_DMON)] == 2

  # engaged, distracted while moving, then car stops after reaching orange
  #  - should reset timer to pre green at standstill
  def test_distracted_then_stops(self):
    _stop_time = DISTRACTED_SECONDS_TO_ORANGE + 1  # stop 1 second after reaching orange
    standstill_vector = always_false[:]
    standstill_vector[int(_stop_time/DT_DMON):] = [True] * int((TEST_TIMESPAN-_stop_time)/DT_DMON)
    alert_lvls, _ = self._run_seq(always_distracted, always_false, always_true, standstill_vector)
    # just before and briefly after stopping: orange alert; goes away quickly after stopped
    assert alert_lvls[int((_stop_time+0.1)/DT_DMON)] == 2
    assert alert_lvls[int((_stop_time+0.5)/DT_DMON)] == 0

  # engaged, model is somehow uncertain and driver is distracted
  #  - should fall back to wheel touch after uncertain alert
  def test_somehow_indecisive_model(self):
    ds_vector = [msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN] * int(TEST_TIMESPAN/DT_DMON)
    interaction_vector = always_false[:]
    alert_lvls, d_status = self._run_seq(ds_vector, interaction_vector, always_true, always_false)
    s = d_status.settings
    t_fb = DT_DMON * s._HI_STD_FALLBACK_TIME + s._FACE_OFF_FRAMES * DT_DMON
    assert alert_lvls[int((t_fb + s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT - 2) / DT_DMON)] == 0
    assert alert_lvls[int((t_fb + s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT + 2) / DT_DMON)] == 2
    assert alert_lvls[int((t_fb + s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT + 2) / DT_DMON)] == 3


class TestDmNerf:
  def _run(self, msgs, interaction=None, engaged=None, standstill=None):
    n = len(msgs)
    interaction = always_false[:n] if interaction is None else interaction
    engaged = always_true[:n] if engaged is None else engaged
    standstill = always_false[:n] if standstill is None else standstill
    return TestMonitoring()._run_seq(msgs, interaction, engaged, standstill)

  def test_never_emits_silent_green(self):
    alert_lvls, _ = self._run(always_distracted)
    assert 1 not in [int(a) for a in alert_lvls]

  def test_cluster_pose_stays_quiet(self):
    # stock yaw trip was ~0.40 rad; cluster glance must not nag
    msgs = [make_msg(True, yaw=0.40)] * int(60 / DT_DMON)
    alert_lvls, dm = self._run(msgs)
    assert all(int(a) == 0 for a in alert_lvls)
    assert not dm.distracted_types['pose']

  def test_soft_blink_stays_quiet(self):
    msgs = [make_msg(True, blink=0.90)] * int(60 / DT_DMON)
    alert_lvls, dm = self._run(msgs)
    assert all(int(a) == 0 for a in alert_lvls)
    assert not dm.distracted_types['eye']

  def test_phone_still_reaches_orange(self):
    msgs = [make_msg(True, phone=0.85)] * int(50 / DT_DMON)
    alert_lvls, dm = self._run(msgs)
    assert dm.distracted_types['phone']
    assert int(alert_lvls[int(15 / DT_DMON)]) == 0
    assert int(alert_lvls[int(43 / DT_DMON)]) == 2

  def test_extreme_pose_still_reaches_orange(self):
    msgs = [make_msg(True, yaw=2.0)] * int(50 / DT_DMON)
    alert_lvls, dm = self._run(msgs)
    assert dm.distracted_types['pose']
    assert int(alert_lvls[int(43 / DT_DMON)]) == 2

  def test_face_prob_flicker_stays_latched(self):
    s = DRIVER_MONITOR_SETTINGS()
    msgs = []
    msgs += [make_msg(True, face_prob=0.85)] * int(2 / DT_DMON)
    # 1.0s in the old 0.7 deadband — must stay seen
    msgs += [make_msg(True, face_prob=0.55)] * int(1.0 / DT_DMON)
    # 1.0s below off threshold — still inside 1.5s hold
    msgs += [make_msg(True, face_prob=0.20)] * int(1.0 / DT_DMON)
    dm = DriverMonitoring()
    for i, msg in enumerate(msgs):
      dm._update_states(msg, [0, 0, 0], 0, True, False)
      dm._update_events(False, True, False, 0)
      assert dm.face_detected, f"lost face at t={i * DT_DMON:.2f}s"
    # 1.6s more below off threshold — now drop
    for _ in range(int(1.6 / DT_DMON)):
      dm._update_states(make_msg(False, face_prob=0.20), [0, 0, 0], 0, True, False)
      dm._update_events(False, True, False, 0)
    assert not dm.face_detected
    assert s._FACE_OFF_FRAMES == int(1.5 / DT_DMON)

  def test_alert_hold_blocks_brief_lookback(self):
    n_dist = int(DISTRACTED_SECONDS_TO_ORANGE / DT_DMON)
    n_look = int(1.0 / DT_DMON)
    msgs = [msg_DISTRACTED] * n_dist + [msg_ATTENTIVE] * n_look
    alert_lvls, _ = self._run(msgs)
    assert int(alert_lvls[n_dist - 1]) == 2
    assert int(alert_lvls[n_dist + n_look - 1]) == 2

  def test_alert_hold_clears_after_two_seconds(self):
    n_dist = int(DISTRACTED_SECONDS_TO_ORANGE / DT_DMON)
    n_look = int(3.5 / DT_DMON)
    msgs = [msg_DISTRACTED] * n_dist + [msg_ATTENTIVE] * n_look
    alert_lvls, _ = self._run(msgs)
    assert int(alert_lvls[n_dist - 1]) == 2
    assert int(alert_lvls[-1]) == 0

  def test_brief_face_after_wheel_orange_does_not_clear(self):
    n_inv = int(INVISIBLE_SECONDS_TO_ORANGE / DT_DMON)
    n_face = int(0.5 / DT_DMON)
    msgs = [msg_NO_FACE_DETECTED] * n_inv + [msg_ATTENTIVE] * n_face
    alert_lvls, dm = self._run(msgs)
    assert int(alert_lvls[n_inv - 1]) == 2
    assert int(alert_lvls[-1]) == 2
    assert dm.active_policy == log.DriverMonitoringState.MonitoringPolicy.wheeltouch

  def test_look_away_look_back_does_not_ping_pong(self):
    # 8s blink / 4s eyes-on, 90s total — never 40s continuous, must stay quiet
    cycle = [msg_DISTRACTED] * int(8 / DT_DMON) + [msg_ATTENTIVE] * int(4 / DT_DMON)
    msgs = (cycle * 8)[:int(90 / DT_DMON)]
    alert_lvls, _ = self._run(msgs)
    levels = {int(a) for a in alert_lvls}
    assert levels == {0}, f"ping-pong levels={levels}"

  def test_face_threshold_flicker_does_not_flip_policy(self):
    msgs = []
    for i in range(int(10 / DT_DMON)):
      msgs.append(make_msg(True, face_prob=0.71 if i % 2 == 0 else 0.69))
    alert_lvls, dm = self._run(msgs)
    assert dm.face_detected
    assert dm.active_policy == log.DriverMonitoringState.MonitoringPolicy.vision
    assert all(int(a) == 0 for a in alert_lvls)


def _build_sm(selfdrive_enabled, lat_active, steering_pressed, gas_pressed):
  cs = car.CarState.new_message()
  cs.vEgo = 30.0
  cs.gearShifter = car.CarState.GearShifter.drive
  cs.steeringPressed = steering_pressed
  cs.gasPressed = gas_pressed
  ss = log.SelfdriveState.new_message()
  ss.enabled = selfdrive_enabled
  cc = car.CarControl.new_message()
  cc.latActive = lat_active
  mv2 = log.ModelDataV2.new_message()
  mv2.meta.disengagePredictions.brakeDisengageProbs = [0.0]
  lc = log.LiveCalibrationData.new_message()
  lc.rpyCalib = [0.0, 0.0, 0.0]
  return {
    'carState': cs, 'selfdriveState': ss, 'carControl': cc,
    'modelV2': mv2, 'liveCalibration': lc, 'driverStateV2': make_msg(False),
  }


@pytest.mark.parametrize("selfdrive_enabled, lat_active, steering, gas, expected_op_engaged, expected_driver_engaged", [
  (False, False, False, False, False, False),  # disabled
  (True,  False, False, False, True,  False),  # OP enabled
  (False, True,  False, False, True,  False),  # MADS lat-only
  (True,  True,  False, False, True,  False),  # both active
  (False, True,  False, True,  True,  False),  # MADS lat-only + gas
  (True,  True,  False, True,  True,  True),   # full op + gas: override
  (False, True,  True,  False, True,  True),   # MADS lat-only + wheel touch: override
])
def test_run_step_engagement(selfdrive_enabled, lat_active, steering, gas,
                             expected_op_engaged, expected_driver_engaged):
  sm = _build_sm(selfdrive_enabled, lat_active, steering, gas)
  dm = DriverMonitoring()
  captured = {}
  orig = dm._update_events

  def spy(driver_engaged, op_engaged, standstill, wrong_gear):
    captured['driver_engaged'] = driver_engaged
    captured['op_engaged'] = op_engaged
    return orig(driver_engaged, op_engaged, standstill, wrong_gear)

  dm._update_events = spy
  dm.run_step(sm, demo=False)
  assert captured['op_engaged'] == expected_op_engaged
  assert captured['driver_engaged'] == expected_driver_engaged
