"""Vietnamese legal following-distance floor (Thông tư 38/2024 Điều 11).

Steady MPC gap is T_FOLLOW * v + STOP_DISTANCE (6 m). These helpers return the
meter floor and the equivalent t_follow so VinFast alpha long never sits closer
than the dry-road table above 60 km/h. Personality may still command a longer gap.
"""

# Matches openpilot.common.constants.CV.MS_TO_KPH (kept numeric to stay import-light).
MS_TO_KPH = 3.6

# Must match long_mpc.STOP_DISTANCE.
STOP_DISTANCE = 6.0

VN_FOLLOW_MIN_KPH = 60.0
VN_FOLLOW_HYST_KPH = 2.0
# (band_top_kph, gap_m). V=60 is 35 m; each later row is (prev_top, V].
VN_FOLLOW_TABLE = (
  (60.0, 35.0),
  (80.0, 55.0),
  (100.0, 70.0),
  (120.0, 100.0),
)
# Lower edge of each gap band — used only when dropping the floor.
VN_FOLLOW_LOWER_KPH = {
  35.0: 60.0,
  55.0: 60.0,
  70.0: 80.0,
  100.0: 100.0,
}


def vn_legal_follow_m(v_kph):
  """Instantaneous legal min gap (m), or None below 60 km/h."""
  if v_kph < VN_FOLLOW_MIN_KPH:
    return None
  gap = VN_FOLLOW_TABLE[0][1]
  for edge_kph, edge_gap in VN_FOLLOW_TABLE:
    if v_kph <= edge_kph:
      return edge_gap
    gap = edge_gap
  return gap


def vn_min_follow_m(v_ego, prev_gap=None):
  """Legal min gap with ~2 km/h drop hysteresis so 80/100 km/h edges do not chatter.

  Raising the floor (entering a higher band) is immediate. Lowering it waits until
  speed is VN_FOLLOW_HYST_KPH below the edge that required the previous floor.
  """
  v_kph = float(v_ego) * MS_TO_KPH
  gap_now = vn_legal_follow_m(v_kph)
  if prev_gap is None:
    return gap_now
  if gap_now is None:
    if v_kph < VN_FOLLOW_MIN_KPH - VN_FOLLOW_HYST_KPH:
      return None
    return prev_gap
  if prev_gap is None or gap_now >= prev_gap:
    return gap_now
  edge_kph = VN_FOLLOW_LOWER_KPH.get(prev_gap, VN_FOLLOW_MIN_KPH)
  if v_kph <= edge_kph - VN_FOLLOW_HYST_KPH:
    return gap_now
  return prev_gap


def vn_t_follow_floor(v_ego, d_vn, stop_distance=STOP_DISTANCE):
  """t_follow that makes T_FOLLOW*v + stop_distance == d_vn. None if no floor."""
  if d_vn is None:
    return None
  v = max(float(v_ego), 0.1)
  return max(0.0, (float(d_vn) - stop_distance) / v)


def effective_t_follow(v_ego, t_personality, d_vn=None, stop_distance=STOP_DISTANCE):
  """Personality T_FOLLOW, raised to the VN legal floor when one applies."""
  t_floor = vn_t_follow_floor(v_ego, d_vn, stop_distance=stop_distance)
  if t_floor is None:
    return t_personality
  return max(t_personality, t_floor)
