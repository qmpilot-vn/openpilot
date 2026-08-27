import pytest

from openpilot.selfdrive.controls.lib.vn_follow import (
  STOP_DISTANCE,
  VN_FOLLOW_HYST_KPH,
  effective_t_follow,
  vn_legal_follow_m,
  vn_min_follow_m,
  vn_t_follow_floor,
)

# Matches long_mpc.get_T_FOLLOW (kept here so this file stays import-light).
T_FOLLOW = {
  "aggressive": 1.25,
  "standard": 1.45,
  "relaxed": 1.75,
}


def _ms(kph):
  return kph / 3.6


class TestVnLegalTable:
  def test_below_60_has_no_floor(self):
    assert vn_legal_follow_m(59.0) is None
    assert vn_min_follow_m(_ms(50.0)) is None

  def test_official_band_edges(self):
    assert vn_legal_follow_m(60.0) == 35.0
    assert vn_legal_follow_m(80.0) == 55.0
    assert vn_legal_follow_m(100.0) == 70.0
    assert vn_legal_follow_m(120.0) == 100.0

  def test_just_inside_bands(self):
    assert vn_legal_follow_m(60.1) == 55.0
    assert vn_legal_follow_m(80.1) == 70.0
    assert vn_legal_follow_m(100.1) == 100.0
    assert vn_legal_follow_m(130.0) == 100.0


class TestVnHysteresis:
  def test_raise_floor_immediately(self):
    gap = vn_min_follow_m(_ms(79.0))
    assert gap == 55.0
    gap = vn_min_follow_m(_ms(81.0), prev_gap=gap)
    assert gap == 70.0

  def test_hold_higher_band_for_2_kph(self):
    gap = vn_min_follow_m(_ms(81.0))
    assert gap == 70.0
    # 79 is still above 80 - 2
    assert vn_min_follow_m(_ms(79.0), prev_gap=gap) == 70.0

  def test_drop_after_hysteresis(self):
    gap = vn_min_follow_m(_ms(81.0))
    drop_kph = 80.0 - VN_FOLLOW_HYST_KPH
    assert vn_min_follow_m(_ms(drop_kph), prev_gap=gap) == 55.0

  def test_drop_floor_below_58_kph(self):
    gap = vn_min_follow_m(_ms(61.0))
    assert gap == 55.0
    assert vn_min_follow_m(_ms(59.0), prev_gap=gap) == 55.0
    assert vn_min_follow_m(_ms(57.0), prev_gap=gap) is None


class TestVnTFollowFloor:
  def test_none_below_60(self):
    v = _ms(50.0)
    for t in T_FOLLOW.values():
      assert effective_t_follow(v, t) == pytest.approx(t)

  def test_100_kph_raises_all_personalities(self):
    v = _ms(100.0)
    d_vn = vn_legal_follow_m(100.0)
    t_floor = vn_t_follow_floor(v, d_vn)
    assert t_floor == pytest.approx((70.0 - STOP_DISTANCE) / v)
    for t in T_FOLLOW.values():
      t_eff = effective_t_follow(v, t, d_vn)
      assert t_eff == pytest.approx(t_floor)
      assert t_eff * v + STOP_DISTANCE == pytest.approx(70.0)

  def test_60_kph_relaxed_already_close(self):
    v = _ms(60.0)
    d_vn = vn_legal_follow_m(60.0)
    t = effective_t_follow(v, T_FOLLOW["relaxed"], d_vn)
    assert t * v + STOP_DISTANCE >= 35.0 - 1e-6
    # standard 1.45 s is below the 35 m floor
    t_std = effective_t_follow(v, T_FOLLOW["standard"], d_vn)
    assert t_std > T_FOLLOW["standard"]

  def test_highway_floors(self):
    cases = (
      (80.0, 55.0),
      (100.0, 70.0),
      (120.0, 100.0),
    )
    for kph, d_vn in cases:
      v = _ms(kph)
      t = effective_t_follow(v, T_FOLLOW["standard"], d_vn)
      assert t * v + STOP_DISTANCE == pytest.approx(d_vn)
      assert t > T_FOLLOW["relaxed"]
