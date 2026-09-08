from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.infocan_lead import InfoCanLeadHold


def _lead(drel, tid=10, vlead=0.0, radar=True, status=True):
  return {
    "status": status,
    "radar": radar,
    "dRel": drel,
    "vLead": vlead,
    "vLeadK": vlead,
    "vRel": vlead,
    "radarTrackId": tid,
  }


def _tracks(tid=10, measured=False, drel=7.2):
  return {tid: SimpleNamespace(measured=measured, dRel=drel, identifier=tid, vLeadK=0.0)}


def test_measured_radar_passes_through():
  h = InfoCanLeadHold()
  lead = _lead(7.2, tid=3)
  out = h.update(lead, _tracks(3, measured=True), v_ego=0.1)
  assert out["dRel"] == 7.2
  ghost = _lead(3.7, tid=9)
  out2 = h.update(ghost, {9: SimpleNamespace(measured=True, dRel=3.7, identifier=9, vLeadK=0.0)}, v_ego=0.1)
  assert out2["dRel"] == 3.7


def test_red_light_3m7_swap_holds_7m_infocan():
  h = InfoCanLeadHold()
  tracks = _tracks(10, measured=False, drel=7.2)
  out = h.update(_lead(7.2, tid=10), tracks, v_ego=0.05)
  assert 6.5 < out["dRel"] < 8.0
  ghost_tracks = {11: SimpleNamespace(measured=False, dRel=3.7, identifier=11, vLeadK=2.0)}
  held = h.update(_lead(3.7, tid=11, vlead=2.0), ghost_tracks, v_ego=0.05)
  assert held["radarTrackId"] == 10
  assert held["dRel"] > 6.0


def test_infocan_drel_spike_held_for_few_frames():
  h = InfoCanLeadHold()
  tracks = _tracks(10, measured=False, drel=7.3)
  h.update(_lead(7.3, tid=10), tracks, v_ego=0.1)
  spiked = h.update(_lead(3.7, tid=10), tracks, v_ego=0.1)
  assert spiked["dRel"] > 6.0


def test_moving_cutin_not_held():
  h = InfoCanLeadHold()
  tracks = _tracks(10, measured=False, drel=7.2)
  h.update(_lead(7.2, tid=10), tracks, v_ego=8.0)
  cutin = h.update(_lead(3.7, tid=11, vlead=6.0),
                   {11: SimpleNamespace(measured=False, dRel=3.7, identifier=11, vLeadK=6.0)},
                   v_ego=8.0)
  assert cutin["radarTrackId"] == 11
  assert cutin["dRel"] < 5.0


def test_brief_drop_at_standstill_holds():
  h = InfoCanLeadHold()
  tracks = _tracks(10, measured=False, drel=7.4)
  h.update(_lead(7.4, tid=10), tracks, v_ego=0.0)
  dropped = h.update({"status": False, "radar": False}, tracks, v_ego=0.0)
  assert dropped.get("status") is True
  assert dropped["radarTrackId"] == 10
