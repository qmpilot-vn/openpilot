import logging
import sys
import types

import numpy as np


def ensure_realtime_importable():
  rt = types.ModuleType("openpilot.common.realtime")
  rt.DT_MDL = 0.05
  rt.DT_CTRL = 0.01

  class Priority:
    CTRL_LOW = 51
    CTRL_HIGH = 53

  rt.Priority = Priority
  rt.config_realtime_process = lambda *args, **kwargs: None
  sys.modules["openpilot.common.realtime"] = rt


def ensure_swaglog_importable():
  sl = types.ModuleType("openpilot.common.swaglog")
  sl.cloudlog = logging.getLogger("cloudlog")
  sys.modules["openpilot.common.swaglog"] = sl


def ensure_params_importable():
  try:
    import openpilot.common.params  # noqa: F401
  except ImportError as e:
    if "params_pyx" not in str(e):
      raise
    params_stub = types.ModuleType("openpilot.common.params")

    class Params:
      def __init__(self, *args, **kwargs):
        pass

      def get(self, *args, **kwargs):
        return None

    params_stub.Params = Params
    params_stub.ParamKeyFlag = int
    params_stub.ParamKeyType = int
    params_stub.UnknownKeyName = KeyError
    sys.modules["openpilot.common.params"] = params_stub


def ensure_messaging_importable():
  try:
    import cereal.messaging  # noqa: F401
  except ImportError as e:
    if "ipc_pyx" not in str(e):
      raise
    import cereal

    messaging_stub = types.ModuleType("cereal.messaging")

    class SubMaster:
      pass

    class PubMaster:
      pass

    def new_message(*args, **kwargs):
      return types.SimpleNamespace()

    messaging_stub.SubMaster = SubMaster
    messaging_stub.PubMaster = PubMaster
    messaging_stub.new_message = new_message
    sys.modules["cereal.messaging"] = messaging_stub
    cereal.messaging = messaging_stub


ensure_params_importable()
ensure_messaging_importable()
ensure_realtime_importable()
ensure_swaglog_importable()

from opendbc.car.vinfast import radar_interface as vinfast_radar
from openpilot.selfdrive.controls import radard


def make_track(identifier=0, *, v_ego=6.0, d_rel=15.0, y_rel=0.0,
               v_rel=-1.0, yv_rel=0.0, cnt=8,
               motion_status=radard.MSTATUS_MOVING,
               motion_orientation=radard.ORIENT_PRECEEDING,
               lane_assignment=radard.LANE_HOST):
  track = radard.Track(identifier, v_ego + v_rel, radard.KalmanParams(radard.DT_MDL))
  for _ in range(cnt):
    track.update(d_rel, y_rel, v_rel, v_ego + v_rel, True, yv_rel,
                 motion_status=motion_status,
                 motion_orientation=motion_orientation,
                 lane_assignment=lane_assignment)
  track.yvRel = yv_rel
  track.vLeadK = v_ego + v_rel
  return track


def test_urban_crossing_projected_miss_is_rejected():
  track = make_track(v_ego=8.0, d_rel=14.0, y_rel=0.4, v_rel=-5.0, yv_rel=1.0,
                     motion_orientation=radard.ORIENT_CROSSING_RIGHT,
                     lane_assignment=radard.LANE_HOST)

  assert radard.is_crossing_traffic(track, 8.0, path_y_offset=0.0)


def test_urban_crossing_projected_into_path_is_kept():
  track = make_track(v_ego=8.0, d_rel=10.0, y_rel=1.1, v_rel=-5.0, yv_rel=-0.55,
                     motion_orientation=radard.ORIENT_CROSSING_LEFT,
                     lane_assignment=radard.LANE_HOST)

  assert not radard.is_crossing_traffic(track, 8.0, path_y_offset=0.0)


def test_parked_roadside_uses_path_relative_offset():
  track = make_track(v_ego=6.0, d_rel=18.0, y_rel=0.1, v_rel=-6.0, yv_rel=0.0,
                     motion_status=radard.MSTATUS_STATIONARY,
                     motion_orientation=radard.ORIENT_UNKNOWN_VAL,
                     lane_assignment=radard.LANE_UNKNOWN)

  assert radard.is_crossing_traffic(track, 6.0, path_y_offset=-0.85)


def test_host_preceding_stopped_centerline_is_kept():
  track = make_track(v_ego=5.0, d_rel=16.0, y_rel=0.2, v_rel=-5.0, yv_rel=0.0,
                     motion_status=radard.MSTATUS_STOPPED,
                     motion_orientation=radard.ORIENT_PRECEEDING,
                     lane_assignment=radard.LANE_HOST)

  assert not radard.is_crossing_traffic(track, 5.0, path_y_offset=0.0)


def test_select_best_ignores_path_relative_parked_car():
  parked = make_track(1, v_ego=6.0, d_rel=18.0, y_rel=0.1, v_rel=-6.0, yv_rel=0.0,
                      motion_status=radard.MSTATUS_STATIONARY,
                      motion_orientation=radard.ORIENT_UNKNOWN_VAL,
                      lane_assignment=radard.LANE_UNKNOWN)
  lead = make_track(2, v_ego=6.0, d_rel=24.0, y_rel=-0.85, v_rel=-1.0, yv_rel=0.0,
                    motion_status=radard.MSTATUS_MOVING,
                    motion_orientation=radard.ORIENT_PRECEEDING,
                    lane_assignment=radard.LANE_HOST)
  path_x = np.array([0.0, 20.0, 40.0])
  path_y = np.array([0.0, 0.85, 0.85])

  best = radard.select_best_radar_track({1: parked, 2: lead}, 6.0,
                                        path_x=path_x, path_y=path_y, path_valid=True)

  assert best == lead


def test_vinfast_interface_crossing_helper_keeps_centering_objects():
  assert vinfast_radar.is_moving_toward_center(1.2, 0.6)
  assert not vinfast_radar.is_moving_toward_center(1.2, -0.6)


def test_vinfast_interface_stationary_threshold_is_distance_adaptive():
  near_threshold = vinfast_radar.stationary_roadside_lat_threshold(5.0)
  far_threshold = vinfast_radar.stationary_roadside_lat_threshold(60.0)

  assert near_threshold < far_threshold
  assert near_threshold >= 1.0


def _make_flyby_track(v_ego=20.0):
  """Track matching T13 highway pass-by yRel-collapse signature."""
  track = radard.Track(13, v_ego - 3.0, radard.KalmanParams(radard.DT_MDL))
  frames = [
    (37.0, -1.84, -2.25, 0.0),
    (35.0, -1.50, -2.50, 0.2),
    (32.0, -1.10, -2.80, 0.4),
    (30.0, -0.47, -2.95, 0.31),
    (28.0, -0.35, -3.10, 0.5),
    (27.5, -0.21, -3.05, 0.74),
  ]
  for d, y, v, yv in frames:
    for _ in range(3):
      track.update(d, y, v, v_ego + v, True, yv,
                   motion_status=radard.MSTATUS_MOVING,
                   motion_orientation=radard.ORIENT_INVALID,
                   lane_assignment=radard.LANE_UNKNOWN)
  track.yvRel = 0.74
  return track


def test_lateral_flyby_rejects_yrel_collapse_artifact():
  track = _make_flyby_track()
  assert radard.is_lateral_flyby(track, path_y_offset=0.0)


def test_lateral_flyby_keeps_genuine_cutin():
  track = make_track(v_ego=12.0, d_rel=14.0, y_rel=1.1, v_rel=-4.0, yv_rel=-0.6, cnt=8,
                     motion_orientation=radard.ORIENT_DRIFTING_LEFT,
                     lane_assignment=radard.LANE_LEFT)
  assert not radard.is_lateral_flyby(track, path_y_offset=0.0)


def test_overtake_pass_criterion_covers_highway_range():
  # Before full yRel collapse — overtake-pass rejects side-offset closing tracks.
  track = make_track(v_ego=20.0, d_rel=30.0, y_rel=-1.5, v_rel=-3.0, yv_rel=0.2, cnt=10,
                     motion_orientation=radard.ORIENT_DRIFTING_RIGHT,
                     lane_assignment=radard.LANE_RIGHT)
  track.peak_abs_yRel = 1.8
  track.max_recent_abs_yRel = 1.8
  assert radard.is_crossing_traffic(track, 20.0, path_y_offset=0.0)


def test_vision_only_rejects_adjacent_lane_lead():
  class Lead:
    x = [40.0]
    y = [2.5]   # ISO left → yRel = -2.5 (right side in OP frame after negation... -(-2.5)=2.5)
    v = [15.0]
    a = [0.0]
    prob = 0.8

  lead = Lead()
  state = radard.get_RadarState_from_vision(lead, v_ego=20.0, model_v_ego=20.0)
  assert not state['status']


def make_info_track(identifier=137, v_ego=1.0, d_rel=7.5, y_rel=-1.66, v_rel=0.0, cnt=2):
  """InfoCAN FCAM track: position-only, host-lane urban queue."""
  track = radard.Track(identifier, v_ego, radard.KalmanParams(radard.DT_MDL))
  for _ in range(cnt):
    track.update(d_rel, y_rel, v_rel, v_ego, False, 0.0,
                 motion_status=radard.MSTATUS_STATIONARY,
                 motion_orientation=radard.ORIENT_PRECEEDING,
                 lane_assignment=radard.LANE_HOST)
  return track


def test_info_host_creep_keeps_offset_motorbike():
  """VF7 red-light queue: InfoCAN LatDis +1.66 m must not drop lead at creep speed."""
  class Lead:
    x = [6.0]
    y = [0.0]
    v = [0.0]
    a = [0.0]
    prob = 1.0

  tracks = {137: make_info_track()}
  path_x = np.array([0.0, 50.0])
  path_y = np.array([0.0, 0.0])
  lead = radard.get_lead(
    1.0, True, tracks, Lead(), 1.0,
    path_x=path_x, path_y=path_y, path_valid=True,
  )
  assert lead.get('status')
  assert float(lead.get('dRel', 99)) < 10.0


def test_position_only_fusion_rejects_large_distance_mismatch():
  """Highway: InfoCAN without Rel_Vx must not fuse when dRel diverges from vision."""
  class Lead:
    x = [12.0]   # vision dRel ≈ 10.48 m after RADAR_TO_CAMERA
    y = [0.0]
    v = [25.0]
    a = [0.0]
    xStd = [1.0]
    yStd = [0.5]
    vStd = [1.0]
    prob = 0.95

  track = make_info_track(identifier=200, v_ego=27.0, d_rel=18.0, y_rel=0.0, v_rel=-2.0, cnt=10)
  matched = radard.match_vision_to_track(
    27.0, Lead(), {200: track},
    path_x=np.array([0.0, 200.0]), path_y=np.array([0.0, 0.0]), path_valid=True,
  )
  assert matched is None


def _run_all():
  tests = [obj for name, obj in globals().items()
           if name.startswith("test_") and callable(obj)]
  for test_fn in tests:
    test_fn()
  print(f"OK ({len(tests)} tests)")


if __name__ == "__main__":
  _run_all()
