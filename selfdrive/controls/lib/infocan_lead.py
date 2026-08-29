"""VF6/VF7 InfoCAN lead hold.

InfoCAN has no Doppler. dRel stick-slip and one-frame vision swaps (7 m <-> 3.7 m
at a red light) make long MPC and the packaged reengage chatter. VF8/VF9 tracks
are measured=True and pass through unchanged.
"""


# One-frame 3.7 m vision swap on route 8a83db1b6dedf381/0000009b.
DREL_SPIKE_M = 2.5
DREL_SPIKE_HOLD = 4
STOPPED_VLEAD = 1.2  # [m/s]
LOW_VEGO = 3.0       # [m/s]
ID_HOLD_FRAMES = 8
STABLE_DREL_MIN = 5.0
STABLE_DREL_MAX = 16.0


def _track_id(lead):
  for key in ("radarTrackId", "trackId"):
    if key in lead and lead[key] is not None:
      return int(lead[key])
  return -1


def _lookup_track(lead, tracks):
  if not tracks:
    return None
  tid = _track_id(lead)
  if tid < 0:
    return None
  return tracks.get(tid)


def is_infocan_lead(lead, tracks) -> bool:
  if not lead.get("status") or not lead.get("radar"):
    return False
  track = _lookup_track(lead, tracks)
  if track is None:
    return False
  return not bool(getattr(track, "measured", True))


def is_measured_lead(lead, tracks) -> bool:
  if not lead.get("status") or not lead.get("radar"):
    return False
  track = _lookup_track(lead, tracks)
  if track is None:
    return False
  return bool(getattr(track, "measured", True))


def _stopped_stable(lead) -> bool:
  d = float(lead.get("dRel", 0.0))
  v = float(lead.get("vLeadK", lead.get("vLead", 0.0)))
  return (STABLE_DREL_MIN <= d <= STABLE_DREL_MAX) and (abs(v) < STOPPED_VLEAD)


class InfoCanLeadHold:
  """Hold a stopped InfoCAN lead across short dRel ghosts and track drops."""

  def __init__(self):
    self._d_filt = None
    self._d_hold = 0
    self._prev = None
    self._id_hold = 0

  def update(self, lead, tracks, v_ego: float):
    lead = dict(lead)
    tracks = tracks or {}

    # VF8/VF9 Doppler tracks: never hold or rewrite.
    if is_measured_lead(lead, tracks):
      self._reset()
      return lead

    if self._keep_previous(lead, tracks, v_ego):
      self._id_hold += 1
      kept = dict(self._prev)
      return self._filter_drel(kept)

    self._id_hold = 0
    if is_infocan_lead(lead, tracks):
      if self._prev is not None and _track_id(lead) != _track_id(self._prev):
        self._d_filt = None
        self._d_hold = 0
      lead = self._filter_drel(lead)
      self._prev = dict(lead)
      return lead

    if (self._prev is not None) and (v_ego < LOW_VEGO) and (not lead.get("status")) and _stopped_stable(self._prev):
      self._id_hold = 1
      return dict(self._prev)

    if not lead.get("status"):
      self._reset()
    return lead

  def _keep_previous(self, lead, tracks, v_ego) -> bool:
    if self._prev is None or v_ego >= LOW_VEGO:
      return False
    if self._id_hold >= ID_HOLD_FRAMES:
      return False
    if not _stopped_stable(self._prev):
      return False
    if is_measured_lead(lead, tracks):
      return False
    if not lead.get("status"):
      return True
    d_new = float(lead.get("dRel", 0.0))
    d_old = float(self._prev.get("dRel", 0.0))
    jumped_closer = (d_old - d_new) > DREL_SPIKE_M
    swapped = _track_id(lead) != _track_id(self._prev) and jumped_closer
    return jumped_closer or swapped

  def _filter_drel(self, lead):
    if not lead.get("status"):
      return lead
    d = float(lead.get("dRel", 0.0))
    if self._d_filt is None:
      self._d_filt = d
      return lead
    if abs(d - self._d_filt) > DREL_SPIKE_M:
      self._d_hold += 1
      if self._d_hold < DREL_SPIKE_HOLD:
        lead = dict(lead)
        lead["dRel"] = self._d_filt
        return lead
      self._d_hold = 0
      self._d_filt = d
    else:
      self._d_hold = 0
      self._d_filt = 0.7 * self._d_filt + 0.3 * d
      lead = dict(lead)
      lead["dRel"] = self._d_filt
    return lead

  def _reset(self):
    self._d_filt = None
    self._d_hold = 0
    self._prev = None
    self._id_hold = 0
