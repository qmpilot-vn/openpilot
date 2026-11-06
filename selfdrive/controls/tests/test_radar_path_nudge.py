"""VF9 radar nudge on 3cc56ba1a9bc3be2|0000006a--b2a4247305 segments 3 and 4.

The right-side passers are already in liveTracks. Radar calls them oncoming
(orientation 6, vLead about -6 m/s) at y ≈ +2 m. The nudge has to move left
for those, and a far left object must not cancel that into a right bias.
"""

from openpilot.selfdrive.controls.lib.radar_path_nudge import RadarPathNudge


class _Pt:
  def __init__(self, d, y, v, orient=6, lane=0, status=2, yv=0.0, measured=True, tid=1):
    self.dRel = d
    self.yRel = y
    self.vRel = v
    self.yvRel = yv
    self.motionOrientation = orient
    self.laneAssignment = lane
    self.motionStatus = status
    self.measured = measured
    self.trackId = tid


def _settle(points, v_ego, n=50):
  nudge = RadarPathNudge("VINFAST_VF9")
  for _ in range(n):
    nudge.update(points, v_ego, lat_active=True)
  return nudge


def test_seg4_right_oncoming_nudges_left():
  # Track 346, ~21 km/h, alone on the right, closing from 32 m to 10 m at y≈2.2.
  # A few centimeters is the old cap. This pass has to move the path by more than 12 cm.
  v_ego = 21.6 / 3.6
  nudge = _settle([_Pt(12.0, 2.25, -12.6, tid=346)], v_ego)
  assert nudge.y_nudge_filt < -0.12, nudge.y_nudge_filt
  assert nudge.kappa_filt < -5.0e-4, nudge.kappa_filt


def test_seg3_close_right_oncoming_nudges_left():
  # Track 284 at the bumper, d=3 m, y=2.10. The old 4 m floor dropped this.
  v_ego = 14.0 / 3.6
  nudge = _settle([_Pt(3.0, 2.10, -11.2, tid=284)], v_ego)
  assert nudge.y_nudge_filt < -0.12, nudge.y_nudge_filt


def test_seg3_far_left_does_not_override_the_right_passer():
  # t≈8 s: track 245 is 27 m ahead-left, track 258 is the right-side oncoming.
  v_ego = 19.4 / 3.6
  points = [
    _Pt(27.2, -1.30, 0.0, orient=12, tid=245),
    _Pt(15.1, 2.31, -13.8, tid=258),
  ]
  nudge = _settle(points, v_ego)
  assert nudge.y_nudge_filt < -0.08, nudge.y_nudge_filt


def test_in_path_oncoming_is_not_a_nudge():
  v_ego = 6.0
  nudge = _settle([_Pt(8.0, 0.40, -12.0)], v_ego)
  assert abs(nudge.kappa_filt) < 1e-6, nudge.kappa_filt
