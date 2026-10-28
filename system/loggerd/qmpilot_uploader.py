#!/usr/bin/env python3
"""Upload to qmpilot-connect. Comma Connect stays on system.loggerd.uploader.

This process starts only when ApiHost and QmpilotApiKey select this backend,
the same way sunnylink_uploader starts only when sunnylink upload is selected.
Each file uses its own xattr, so a comma upload does not count here.
"""
import json
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
from openpilot.common.api.comma_connect import get_qmpilot_api_host, get_qmpilot_api_key
from openpilot.common.utils import get_upload_stream
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr
from openpilot.common.swaglog import cloudlog

NetworkType = log.DeviceState.NetworkType
UPLOAD_ATTR_NAME = 'user.qmpilot.upload'
UPLOAD_ATTR_VALUE = b'1'

MAX_UPLOAD_SIZES = {
  "qlog": 25*1e6,
  "qcam": 5*1e6,
}

UPLOAD_PRIORITY = {
  "qlog": 0, "qlog.zst": 0, "qlog.bz2": 0,
  "qcamera.ts": 1,
  "rlog": 2, "rlog.zst": 2, "rlog.bz2": 2,
  "fcamera.hevc": 3,
  "ecamera.hevc": 4,
  "dcamera.hevc": 5,
}

HEAVY_NAMES = {"rlog", "rlog.zst", "rlog.bz2", "fcamera.hevc", "ecamera.hevc", "dcamera.hevc"}
ALWAYS_NAMES = {"qlog", "qlog.zst", "qlog.bz2", "qcamera.ts"}

allow_sleep = bool(int(os.getenv("UPLOADER_SLEEP", "1")))
force_wifi = os.getenv("FORCEWIFI") is not None
fake_upload = os.getenv("FAKEUPLOAD") is not None


def to_qmpilot_path(key: str) -> str:
  """Local 000003e3--d68e850585--0/qlog.zst -> 000003e3--d68e850585/0/qlog.zst."""
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


class FakeRequest:
  def __init__(self):
    self.headers = {"Content-Length": "0"}


class FakeResponse:
  def __init__(self):
    self.status_code = 200
    self.request = FakeRequest()


def _listdir_by_creation(d: str) -> list[str]:
  from openpilot.system.loggerd.uploader import listdir_by_creation
  return listdir_by_creation(d)


class Uploader:
  def __init__(self, dongle_id: str, root: str):
    self.dongle_id = dongle_id
    self.root = root
    self.params = Params()
    self.api_host = get_qmpilot_api_host()
    self.api_key = get_qmpilot_api_key()
    if not self.api_host or not self.api_key:
      raise Exception("qmpilot uploader requires ApiHost and QmpilotApiKey")

    self.last_filename = ""
    self.immediate_priority = dict(UPLOAD_PRIORITY)
    self._defer_until: dict[str, float] = {}
    cloudlog.info("qmpilot uploader selected api_host=%s", self.api_host)

  def _route_requested(self, logdir: str, requested_routes: list[str]) -> bool:
    return any(logdir.startswith(r.split('|')[-1]) for r in requested_routes)

  def list_upload_files(self, metered: bool) -> Iterator[tuple[str, str, str]]:
    r = self.params.get("AthenadRecentlyViewedRoutes")
    requested_routes = [] if r is None else [route for route in r.split(",") if route]
    now = time.monotonic()

    for logdir in _listdir_by_creation(self.root):
      if logdir in ("boot", "crash") or logdir.startswith(("boot/", "crash/")):
        continue

      path = os.path.join(self.root, logdir)
      try:
        names = os.listdir(path)
      except OSError:
        continue

      if any(name.endswith(".lock") for name in names):
        continue

      for name in sorted(names, key=lambda n: self.immediate_priority.get(n, 1000)):
        if name not in self.immediate_priority and name not in ALWAYS_NAMES and name not in HEAVY_NAMES:
          continue
        key = os.path.join(logdir, name)
        fn = os.path.join(path, name)
        try:
          is_uploaded = getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
        except OSError:
          cloudlog.event("qmpilot_uploader_getxattr_failed", key=key, fn=fn)
          continue
        if is_uploaded:
          continue
        if self._defer_until.get(fn, 0) > now:
          continue

        if metered:
          viewed = self._route_requested(logdir, requested_routes)
          if name == "qcamera.ts" and not viewed:
            continue
          if name in HEAVY_NAMES and not viewed:
            continue

        yield name, key, fn

  def next_file_to_upload(self, metered: bool) -> tuple[str, str, str] | None:
    for name, key, fn in self.list_upload_files(metered):
      if name in self.immediate_priority:
        return name, key, fn
    return None

  def _request_upload_url(self, key: str):
    remote_key = to_qmpilot_path(key)
    return requests.get(
      f"{self.api_host}/v1.4/{self.dongle_id}/upload_url/",
      params={"path": remote_key, "access_token": self.api_key},
      headers={"X-Device-Key": self.api_key},
      timeout=10,
    )

  def do_callback(self, key: str) -> bool:
    remote_key = to_qmpilot_path(key)
    try:
      resp = requests.post(
        f"{self.api_host}/v1/upload/callback",
        headers={
          "Content-Type": "application/json",
          "X-Device-Key": self.api_key,
        },
        json={"dongle_id": self.dongle_id, "path": remote_key},
        timeout=10,
      )
      if resp.status_code == 200:
        cloudlog.event("qmpilot_upload_callback_success", key=remote_key, status=resp.status_code)
        return True
      cloudlog.event("qmpilot_upload_callback_failed", key=remote_key, status=resp.status_code, body=resp.text[:200])
      return False
    except Exception:
      cloudlog.exception(f"qmpilot_upload_callback_exception key={remote_key}")
      return False

  def _put_timeout(self, sz: int) -> tuple[float, float]:
    read_s = max(120.0, (sz / 250_000.0) + 60.0)
    return (15.0, min(read_s, 1800.0))

  def _defer(self, fn: str, resp) -> None:
    retry = 60
    try:
      retry = int(resp.headers.get("Retry-After", "60"))
    except (TypeError, ValueError, AttributeError):
      retry = 60
    self._defer_until[fn] = time.monotonic() + max(1, retry)

  def do_upload(self, key: str, fn: str):
    url_resp = self._request_upload_url(key)
    if url_resp.status_code in (412, 503):
      return url_resp

    url_resp_json = json.loads(url_resp.text)
    url = url_resp_json['url']
    headers = url_resp_json['headers']
    cloudlog.debug("qmpilot upload_url v1.4 %s %s", url, str(headers))

    if fake_upload:
      return FakeResponse()

    stream = None
    try:
      compress = key.endswith('.zst') and not fn.endswith('.zst')
      stream, content_length = get_upload_stream(fn, compress)
      response = requests.put(url, data=stream, headers=headers, timeout=self._put_timeout(content_length))
      return response
    finally:
      if stream:
        stream.close()

  def upload(self, name: str, key: str, fn: str, network_type: int, metered: bool) -> bool:
    try:
      sz = os.path.getsize(fn)
    except OSError:
      cloudlog.exception("qmpilot upload: getsize failed")
      return False

    cloudlog.event("qmpilot_upload_start", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered,
                   api_host=self.api_host)

    needs_callback = False
    if sz == 0:
      success = True
    elif name in MAX_UPLOAD_SIZES and sz > MAX_UPLOAD_SIZES[name]:
      cloudlog.event("qmpilot_uploader_too_large", key=key, fn=fn, sz=sz)
      success = True
    else:
      start_time = time.monotonic()
      stat = None
      last_exc = None
      try:
        stat = self.do_upload(key, fn)
      except Exception as e:
        last_exc = (e, traceback.format_exc())

      if stat is not None and stat.status_code == 503:
        self._defer(fn, stat)
        cloudlog.event("qmpilot_upload_deferred", key=key, fn=fn, sz=sz, status=503)
        return True

      # 401 and 403 stay unmarked so the file is sent again
      if stat is not None and stat.status_code in (200, 201, 412):
        self.last_filename = fn
        dt = time.monotonic() - start_time
        if stat.status_code == 412:
          cloudlog.event("qmpilot_upload_ignored", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
        else:
          content_length = int(stat.request.headers.get("Content-Length", 0))
          speed = (content_length / 1e6) / dt if dt > 0 else 0
          cloudlog.event("qmpilot_upload_success", key=key, fn=fn, sz=sz, content_length=content_length,
                         network_type=network_type, metered=metered, speed=speed)
        success = True
        needs_callback = True
      else:
        success = False
        cloudlog.event("qmpilot_upload_failed", stat=stat, exc=last_exc, key=key, fn=fn, sz=sz,
                       network_type=network_type, metered=metered)

    if success and needs_callback and not self.do_callback(key):
      success = False

    if success:
      try:
        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
      except OSError:
        cloudlog.event("qmpilot_uploader_setxattr_failed", key=key, fn=fn, sz=sz)

    return success

  def step(self, network_type: int, metered: bool) -> bool | None:
    d = self.next_file_to_upload(metered)
    if d is None:
      return None

    name, key, fn = d
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

  params = Params()
  dongle_id = params.get("DongleId")
  if dongle_id is None:
    cloudlog.info("qmpilot uploader missing dongle_id")
    raise Exception("qmpilot uploader can't start without dongle id")

  sm = messaging.SubMaster(['deviceState'])
  uploader = Uploader(dongle_id, Paths.log_root())

  backoff = 0.1
  while not exit_event.is_set():
    host = get_qmpilot_api_host()
    key = get_qmpilot_api_key()
    if host != uploader.api_host or key != uploader.api_key:
      cloudlog.info("qmpilot credentials changed; exiting so the uploader restarts")
      return
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
      cloudlog.info("qmpilot upload backoff %r", backoff)
      backoff = min(backoff * 2, 120)
    if allow_sleep:
      time.sleep(backoff + random.uniform(0, backoff))


if __name__ == "__main__":
  main()
