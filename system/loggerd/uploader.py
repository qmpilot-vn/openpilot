#!/usr/bin/env python3
import os
import random
import requests
import threading
import time
import traceback
import datetime
from collections.abc import Iterator

from cereal import log
import cereal.messaging as messaging
from openpilot.common.api import Api, get_qmpilot_api_key
from openpilot.common.api.comma_connect import DEFAULT_API_HOST, get_api_host
from openpilot.common.utils import get_upload_stream
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr
from openpilot.common.swaglog import cloudlog

NetworkType = log.DeviceState.NetworkType
UPLOAD_ATTR_NAME = 'user.upload'
UPLOAD_ATTR_VALUE = b'1'

MAX_UPLOAD_SIZES = {
  "qlog": 25*1e6,  # can't be too restrictive here since we use qlogs to find
                   # bugs, including ones that can cause massive log sizes
  "qcam": 5*1e6,
}

# qmpilot-server has no athena pull — upload full segment assets when configured
QMPILOT_UPLOAD_PRIORITY = {
  "qlog": 0, "qlog.zst": 0, "qlog.bz2": 0,
  "qcamera.ts": 1,
  "rlog": 2, "rlog.zst": 2, "rlog.bz2": 2,
  "fcamera.hevc": 3,
  "ecamera.hevc": 4,
  "dcamera.hevc": 5,
}

# Small assets that make a route viewable on the server. Uploaded for every segment
# before any multi-MB asset, since on a slow link the deleter reclaims the oldest
# segment before its cameras finish, losing the whole segment.
QMPILOT_LIGHT = {"qlog", "qlog.zst", "qlog.bz2", "qcamera.ts"}
QMPILOT_HEAVY = {
  "rlog", "rlog.zst", "rlog.bz2",
  "fcamera.hevc", "dcamera.hevc", "ecamera.hevc",
}
QMPILOT_FIRST_PASS = tuple(QMPILOT_LIGHT)

# Save-gated heavy uploads: poll policy, cache briefly, honor 503 Retry-After.
# Reuses existing QmpilotApiKey — no key regen required.
QMPILOT_POLICY_CACHE_TTL = 90.0
QMPILOT_DEFER_DEFAULT_SEC = 60.0
QMPILOT_DEFER_MAX_SEC = 900.0
QMPILOT_POLICY_FAIL_DEFER_SEC = 120.0
QMPILOT_MISSING_FILE_DEFER_SEC = 300.0

allow_sleep = bool(int(os.getenv("UPLOADER_SLEEP", "1")))
force_wifi = os.getenv("FORCEWIFI") is not None
fake_upload = os.getenv("FAKEUPLOAD") is not None


def to_qmpilot_path(key: str) -> str:
  """Convert openpilot local key to qmpilot-server path.

  Local:  000003e3--d68e850585--0/qlog.zst
  Remote: 000003e3--d68e850585/0/qlog.zst
  """
  if '/' not in key:
    return key
  dirname, filename = key.rsplit('/', 1)
  if dirname in ('boot', 'crash') or dirname.startswith(('boot/', 'crash/')):
    return key
  if '--' in dirname:
    route, seg = dirname.rsplit('--', 1)
    if seg.isdigit():
      return f"{route}/{seg}/{filename}"
  return key


def qmpilot_log_id(key: str) -> str | None:
  """Extract route log_id from a local or remote-style key."""
  remote = to_qmpilot_path(key)
  parts = remote.split('/')
  if len(parts) >= 3 and parts[1].isdigit():
    return parts[0]
  if '/' in key:
    dirname = key.rsplit('/', 1)[0]
    if '--' in dirname:
      route, seg = dirname.rsplit('--', 1)
      if seg.isdigit():
        return route
  return None


def qmpilot_filename(name: str, key: str) -> str:
  """Canonical filename used for light/heavy classification."""
  base = key.rsplit('/', 1)[-1] if key else name
  if base.endswith(('qlog', 'rlog')):
    return base + ".zst"
  return base


class FakeRequest:
  def __init__(self):
    self.headers = {"Content-Length": "0"}


class FakeResponse:
  def __init__(self):
    self.status_code = 200
    self.request = FakeRequest()


def get_directory_sort(d: str) -> list[str]:
  # ensure old format is sorted sooner
  o = ["0", ] if d.startswith("2024-") else ["1", ]
  return o + [s.rjust(10, '0') for s in d.rsplit('--', 1)]

def listdir_by_creation(d: str) -> list[str]:
  if not os.path.isdir(d):
    return []

  try:
    paths = [f for f in os.listdir(d) if os.path.isdir(os.path.join(d, f))]
    paths = sorted(paths, key=get_directory_sort)
    return paths
  except OSError:
    cloudlog.exception("listdir_by_creation failed")
    return []

def clear_locks(root: str) -> None:
  for logdir in os.listdir(root):
    path = os.path.join(root, logdir)
    try:
      for fname in os.listdir(path):
        if fname.endswith(".lock"):
          os.unlink(os.path.join(path, fname))
    except OSError:
      cloudlog.exception("clear_locks failed")


class Uploader:
  def __init__(self, dongle_id: str, root: str):
    self.dongle_id = dongle_id
    self.api = Api(dongle_id)
    self.root = root

    self.params = Params()
    # Existing device token — same key for upload_url + upload_policy (no regen)
    self.qmpilot_api_key = get_qmpilot_api_key()
    self.qmpilot_mode = bool(self.qmpilot_api_key)

    # stats for last successfully uploaded file
    self.last_filename = ""

    # qmpilot Save gate: per-log_id policy cache + per-file/route deferrals (keep locals)
    self._policy_cache: dict[str, tuple[float, bool]] = {}
    self._defer_until: dict[str, float] = {}
    self._defer_log_until: dict[str, float] = {}

    if self.qmpilot_mode:
      # qmpilot-server only accepts route/seg/{qlog,rlog,cameras} — skip boot/crash
      self.immediate_folders = []
      self.immediate_priority = dict(QMPILOT_UPLOAD_PRIORITY)
      cloudlog.info("uploader qmpilot mode api_host=%s", self.api.api_host)
    else:
      self.immediate_folders = ["crash/", "boot/"]
      self.immediate_priority = {"qlog": 0, "qlog.zst": 0, "qcamera.ts": 1}
      if get_api_host() != DEFAULT_API_HOST:
        cloudlog.warning("ApiHost=%s set but QmpilotApiKey missing — upload_url will use JWT and likely 401",
                         get_api_host())

  def _defer_keys(self, key: str) -> set[str]:
    # step() may append .zst for upload while list_upload_files uses the on-disk name
    keys = {key}
    if key.endswith('.zst'):
      keys.add(key[:-4])
    else:
      keys.add(key + '.zst')
    return keys

  def _clamp_delay(self, delay_sec: float) -> float:
    return max(1.0, min(float(delay_sec), QMPILOT_DEFER_MAX_SEC))

  def _is_log_deferred(self, log_id: str | None) -> bool:
    if not log_id:
      return False
    now = time.monotonic()
    until = self._defer_log_until.get(log_id)
    if until is None:
      return False
    if now >= until:
      self._defer_log_until.pop(log_id, None)
      return False
    return True

  def _is_deferred(self, key: str) -> bool:
    if self._is_log_deferred(qmpilot_log_id(key)):
      return True
    now = time.monotonic()
    until = None
    for k in self._defer_keys(key):
      u = self._defer_until.get(k)
      if u is not None and (until is None or u > until):
        until = u
    if until is None:
      return False
    if now >= until:
      for k in self._defer_keys(key):
        self._defer_until.pop(k, None)
      return False
    return True

  def _defer(self, key: str, delay_sec: float, reason: str) -> None:
    delay = self._clamp_delay(delay_sec)
    until = time.monotonic() + delay
    for k in self._defer_keys(key):
      self._defer_until[k] = until
    cloudlog.event("upload_deferred", key=key, delay_sec=delay, reason=reason)

  def _defer_log(self, log_id: str, delay_sec: float, reason: str) -> None:
    delay = self._clamp_delay(delay_sec)
    self._defer_log_until[log_id] = time.monotonic() + delay
    cloudlog.event("upload_log_deferred", log_id=log_id, delay_sec=delay, reason=reason)

  def _fetch_allow_heavy(self, log_id: str) -> bool | None:
    """GET /v1.4/{dongle}/upload_policy/ with the existing QmpilotApiKey.

    Returns True/False from server, or None if the request failed.
    None must NOT fall through to upload_url (that hot-loops 503s on unsaved).
    """
    now = time.monotonic()
    cached = self._policy_cache.get(log_id)
    if cached is not None and now < cached[0]:
      return cached[1]

    try:
      resp = requests.get(
        f"{self.api.api_host}/v1.4/{self.dongle_id}/upload_policy/",
        params={"log_id": log_id, "access_token": self.qmpilot_api_key},
        headers={"X-Device-Key": self.qmpilot_api_key},
        timeout=10,
      )
      if resp.status_code != 200:
        cloudlog.event("upload_policy_failed", log_id=log_id, status=resp.status_code, body=resp.text[:200])
        return None
      body = resp.json()
      allow = bool(body.get("allow_heavy"))
      self._policy_cache[log_id] = (now + QMPILOT_POLICY_CACHE_TTL, allow)
      cloudlog.event("upload_policy", log_id=log_id, allow_heavy=allow, saved=body.get("saved"))
      return allow
    except Exception:
      cloudlog.exception(f"upload_policy_exception log_id={log_id}")
      return None

  def _is_heavy(self, name: str, key: str) -> bool:
    filename = qmpilot_filename(name, key)
    return filename in QMPILOT_HEAVY or name in QMPILOT_HEAVY

  def _is_light(self, name: str, key: str) -> bool:
    filename = qmpilot_filename(name, key)
    return filename in QMPILOT_LIGHT or name in QMPILOT_LIGHT

  def should_upload(self, name: str, key: str) -> bool:
    """Light files always; heavy only when route is Saved (allow_heavy)."""
    if not self.qmpilot_mode:
      return True

    if self._is_light(name, key):
      return True
    if not self._is_heavy(name, key):
      return True  # unknown: keep old behavior

    log_id = qmpilot_log_id(key)
    if log_id is None:
      return True
    if self._is_log_deferred(log_id):
      return False

    allow = self._fetch_allow_heavy(log_id)
    if allow is None:
      # Policy blip — defer route; do NOT call upload_url (avoids 503 storms)
      self._defer_log(log_id, QMPILOT_POLICY_FAIL_DEFER_SEC, "policy_unavailable")
      return False
    if not allow:
      self._defer_log(log_id, QMPILOT_POLICY_CACHE_TTL, "policy_unsaved")
      return False
    return True

  def list_upload_files(self, metered: bool) -> Iterator[tuple[str, str, str]]:
    r = self.params.get("AthenadRecentlyViewedRoutes")
    requested_routes = [] if r is None else [route for route in r.split(",") if route]

    for logdir in listdir_by_creation(self.root):
      path = os.path.join(self.root, logdir)
      try:
        names = os.listdir(path)
      except OSError:
        continue

      if any(name.endswith(".lock") for name in names):
        continue

      for name in sorted(names, key=lambda n: self.immediate_priority.get(n, 1000)):
        key = os.path.join(logdir, name)
        fn = os.path.join(path, name)
        # skip files already uploaded
        try:
          ctime = os.path.getctime(fn)
          is_uploaded = getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
        except OSError:
          cloudlog.event("uploader_getxattr_failed", key=key, fn=fn)
          # deleter could have deleted, so skip
          continue
        if is_uploaded:
          continue
        if self._is_deferred(key):
          continue

        # limit uploading on metered connections
        if metered:
          dt = datetime.timedelta(hours=12)
          if logdir in self.immediate_folders and (datetime.datetime.now() - datetime.datetime.fromtimestamp(ctime)) < dt:
            continue

          if name == "qcamera.ts" and not any(logdir.startswith(r.split('|')[-1]) for r in requested_routes):
            continue
          # on metered, skip bulky full-res assets unless explicitly viewed
          if self.qmpilot_mode and self._is_heavy(name, key):
            if not any(logdir.startswith(r.split('|')[-1]) for r in requested_routes):
              continue

        yield name, key, fn

  def next_file_to_upload(self, metered: bool) -> tuple[str, str, str] | None:
    upload_files = list(self.list_upload_files(metered))

    for name, key, fn in upload_files:
      if any(f in fn for f in self.immediate_folders):
        return name, key, fn

    if self.qmpilot_mode:
      # 1) Always finish light files first (any route)
      for name, key, fn in upload_files:
        if self._is_light(name, key):
          return name, key, fn

      # 2) Heavy only for Saved routes (allow_heavy). Unsaved / policy-fail are
      #    deferred at log_id scope so they cannot starve Saved routes or spam 503.
      for name, key, fn in upload_files:
        if not self._is_heavy(name, key):
          continue
        if self.should_upload(name, key):
          return name, key, fn

      return None

    for name, key, fn in upload_files:
      if name in self.immediate_priority:
        return name, key, fn

    return None

  def _remote_key(self, key: str) -> str:
    return to_qmpilot_path(key) if self.qmpilot_mode else key

  def _request_upload_url(self, key: str):
    """GET /v1.4/{dongle}/upload_url/ — qmpilot uses API key; comma uses JWT."""
    remote_key = self._remote_key(key)
    if self.qmpilot_mode:
      return requests.get(
        f"{self.api.api_host}/v1.4/{self.dongle_id}/upload_url/",
        params={"path": remote_key, "access_token": self.qmpilot_api_key},
        headers={"X-Device-Key": self.qmpilot_api_key},
        timeout=10,
      )
    return self.api.get(
      "v1.4/" + self.dongle_id + "/upload_url/",
      timeout=10,
      path=remote_key,
      access_token=self.api.get_token(),
    )

  def do_callback(self, key: str) -> bool:
    """POST /v1/upload/callback — required by qmpilot-server to index the file."""
    if not self.qmpilot_mode:
      return True
    remote_key = self._remote_key(key)
    try:
      resp = requests.post(
        f"{self.api.api_host}/v1/upload/callback",
        headers={
          "Content-Type": "application/json",
          "X-Device-Key": self.qmpilot_api_key,
        },
        json={"dongle_id": self.dongle_id, "path": remote_key},
        timeout=10,
      )
      if resp.status_code == 200:
        cloudlog.event("upload_callback_success", key=remote_key, status=resp.status_code)
        return True
      cloudlog.event("upload_callback_failed", key=remote_key, status=resp.status_code, body=resp.text[:200])
      return False
    except Exception:
      cloudlog.exception(f"upload_callback_exception key={remote_key}")
      return False

  def _put_timeout(self, sz: int) -> float | tuple[float, float]:
    """PUT timeout. qmpilot over CF tunnel needs long read timeouts for cameras."""
    if not self.qmpilot_mode:
      return 10
    # connect 15s; read allows ~0.25 MB/s worst-case + 60s headroom, cap 30 min
    read_s = max(120.0, (sz / 250_000.0) + 60.0)
    return (15.0, min(read_s, 1800.0))

  def do_upload(self, key: str, fn: str):
    url_resp = self._request_upload_url(key)
    # 412 = ignore; 503/403 = Save gate / defer — return as-is, do NOT parse JSON body
    if url_resp.status_code in (412, 503, 403):
      return url_resp
    if url_resp.status_code != 200:
      cloudlog.event("upload_url_bad_status", key=key, status=url_resp.status_code, body=url_resp.text[:200])
      return url_resp

    url_resp_json = url_resp.json()
    url = url_resp_json['url']
    headers = dict(url_resp_json.get('headers') or {})
    cloudlog.event("upload_url_ok", key=self._remote_key(key), url=url[:160])

    if fake_upload:
      return FakeResponse()

    # Exact sequence after 200: assert local file, PUT bytes, then caller does callback
    try:
      sz = os.path.getsize(fn)
    except OSError:
      cloudlog.event("upload_missing_local", key=key, fn=fn)
      raise FileNotFoundError(fn)
    if sz <= 0:
      cloudlog.event("upload_empty_local", key=key, fn=fn, sz=sz)
      raise FileNotFoundError(fn)

    stream = None
    try:
      compress = key.endswith('.zst') and not fn.endswith('.zst')
      stream, content_length = get_upload_stream(fn, compress)
      # MinIO/S3-compatible PUTs require Content-Length (no chunked encoding)
      headers['Content-Length'] = str(content_length)
      headers.pop('Transfer-Encoding', None)
      timeout = self._put_timeout(content_length)
      cloudlog.event("upload_put_start", key=key, fn=fn, sz=content_length, timeout=str(timeout))
      response = requests.put(url, data=stream, headers=headers, timeout=timeout)
      cloudlog.event("upload_put_done", key=key, status=response.status_code, sz=content_length)
      return response
    finally:
      if stream:
        stream.close()

  def upload(self, name: str, key: str, fn: str, network_type: int, metered: bool) -> bool:
    try:
      sz = os.path.getsize(fn)
    except OSError:
      cloudlog.exception("upload: getsize failed")
      if self.qmpilot_mode:
        self._defer(key, QMPILOT_MISSING_FILE_DEFER_SEC, "missing_local")
        return True  # keep trying other files; do NOT mark uploaded
      return False

    # Policy gate before requesting upload_url (heavy + unsaved → keep local, retry later)
    if self.qmpilot_mode and not self.should_upload(name, key):
      # should_upload already deferred log_id when unsaved / policy unavailable
      return True  # deferred intentionally — do not mark uploaded, do not hard-fail backoff

    cloudlog.event("upload_start", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered,
                   api_host=self.api.api_host, qmpilot=self.qmpilot_mode)

    if sz == 0:
      if self.qmpilot_mode and self._is_heavy(name, key):
        # Never mark empty heavy as uploaded — that permanently skips PUT/callback
        self._defer(key, QMPILOT_MISSING_FILE_DEFER_SEC, "empty_heavy")
        return True
      # tag empty light/unknown files as uploaded
      success = True
    elif name in MAX_UPLOAD_SIZES and sz > MAX_UPLOAD_SIZES[name]:
      cloudlog.event("uploader_too_large", key=key, fn=fn, sz=sz)
      success = True
    else:
      start_time = time.monotonic()

      stat = None
      last_exc = None
      try:
        stat = self.do_upload(key, fn)
      except Exception as e:
        last_exc = (e, traceback.format_exc())

      # Server Save gate: keep file on disk, defer whole route, do NOT mark uploaded
      if self.qmpilot_mode and stat is not None and stat.status_code in (503, 403):
        retry_after = QMPILOT_DEFER_DEFAULT_SEC
        try:
          retry_after = float(stat.headers.get("Retry-After", retry_after))
        except (TypeError, ValueError):
          pass
        log_id = qmpilot_log_id(key)
        if log_id is not None:
          self._policy_cache.pop(log_id, None)
          self._defer_log(log_id, retry_after, f"upload_url_{stat.status_code}")
        else:
          self._defer(key, retry_after, f"upload_url_{stat.status_code}")
        cloudlog.event("upload_gated", key=key, fn=fn, status=stat.status_code, retry_after=retry_after)
        return True  # deferred — file kept; avoid upload backoff starving other routes

      # PUT/network failure after upload_url: keep local, defer this file, continue queue
      if self.qmpilot_mode and (stat is None or stat.status_code not in (200, 201, 412)):
        self._defer(key, QMPILOT_DEFER_DEFAULT_SEC, "put_or_url_failed")
        cloudlog.event("upload_failed", stat=stat, exc=last_exc, key=key, fn=fn, sz=sz,
                       network_type=network_type, metered=metered)
        return True  # do not mark uploaded; do not exponential-backoff the whole uploader

      # comma marks auth failures as "done" to avoid retry storms; qmpilot must not
      ok_codes = (200, 201, 412) if self.qmpilot_mode else (200, 201, 401, 403, 412)
      if stat is not None and stat.status_code in ok_codes:
        self.last_filename = fn
        dt = time.monotonic() - start_time
        if stat.status_code == 412:
          cloudlog.event("upload_ignored", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
        else:
          content_length = int(stat.request.headers.get("Content-Length", 0))
          speed = (content_length / 1e6) / dt if dt > 0 else 0
          cloudlog.event("upload_success", key=key, fn=fn, sz=sz, content_length=content_length,
                         network_type=network_type, metered=metered, speed=speed)
        success = True
      else:
        success = False
        cloudlog.event("upload_failed", stat=stat, exc=last_exc, key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)

    # qmpilot-server needs the callback to index / process the route — only after successful PUT
    if success and self.qmpilot_mode and sz > 0 and not (name in MAX_UPLOAD_SIZES and sz > MAX_UPLOAD_SIZES[name]):
      if not self.do_callback(key):
        # PUT may have landed; do not mark uploaded so callback is retried
        self._defer(key, QMPILOT_DEFER_DEFAULT_SEC, "callback_failed")
        cloudlog.event("upload_callback_retry", key=key, fn=fn)
        return True

    if success:
      # tag file as uploaded only after PUT (+ callback in qmpilot mode)
      try:
        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
      except OSError:
        cloudlog.event("uploader_setxattr_failed", key=key, fn=fn, sz=sz)

    return success


  def step(self, network_type: int, metered: bool) -> bool | None:
    d = self.next_file_to_upload(metered)
    if d is None:
      return None

    name, key, fn = d

    # qlogs and bootlogs need to be compressed before uploading
    if key.endswith(('qlog', 'rlog')) or (key.startswith('boot/') and not key.endswith('.zst')):
      key += ".zst"

    return self.upload(name, key, fn, network_type, metered)


def main(exit_event: threading.Event | None = None) -> None:
  if exit_event is None:
    exit_event = threading.Event()

  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("failed to set core affinity")

  clear_locks(Paths.log_root())

  params = Params()
  dongle_id = params.get("DongleId")

  if dongle_id is None:
    cloudlog.info("uploader missing dongle_id")
    raise Exception("uploader can't start without dongle id")

  sm = messaging.SubMaster(['deviceState'])
  uploader = Uploader(dongle_id, Paths.log_root())

  backoff = 0.1
  while not exit_event.is_set():
    sm.update(0)
    offroad = params.get_bool("IsOffroad")
    network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
    if network_type == NetworkType.none:
      if allow_sleep:
        time.sleep(60 if offroad else 5)
      continue

    success = uploader.step(sm['deviceState'].networkType.raw, sm['deviceState'].networkMetered)
    if success is None:
      backoff = 60 if offroad else 5
    elif success:
      backoff = 0.1
    else:
      cloudlog.info("upload backoff %r", backoff)
      backoff = min(backoff*2, 120)
    if allow_sleep:
      time.sleep(backoff + random.uniform(0, backoff))


if __name__ == "__main__":
  main()
