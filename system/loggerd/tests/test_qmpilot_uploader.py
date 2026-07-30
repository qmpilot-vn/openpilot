import os
import json
from pathlib import Path
from unittest import mock

import pytest

from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.uploader import (
  Uploader,
  UPLOAD_ATTR_NAME,
  UPLOAD_ATTR_VALUE,
  to_qmpilot_path,
  qmpilot_log_id,
  qmpilot_filename,
)
from openpilot.system.loggerd.xattr_cache import getxattr


class _Resp:
  def __init__(self, status_code, payload=None, headers=None, text=None):
    self.status_code = status_code
    self._payload = payload if payload is not None else {}
    self.headers = headers or {}
    self.text = text if text is not None else json.dumps(self._payload)
    self.request = mock.Mock()
    self.request.headers = {"Content-Length": "4"}

  def json(self):
    return self._payload


def _write(seg_dir: str, name: str, data: bytes = b"data") -> tuple[str, str, str]:
  root = Path(Paths.log_root())
  d = root / seg_dir
  d.mkdir(parents=True, exist_ok=True)
  fn = d / name
  fn.write_bytes(data)
  key = f"{seg_dir}/{name}"
  return name, key, str(fn)


@pytest.fixture
def qmpilot_uploader(monkeypatch):
  monkeypatch.setenv("QMPILOT_API_KEY", "test-device-key")
  # Avoid real Api host resolution side effects where possible
  up = Uploader("552ca3dac9765c82", Paths.log_root())
  up.qmpilot_api_key = "test-device-key"
  up.qmpilot_mode = True
  up.api.api_host = "https://qmpilot-connect.com"
  up.immediate_folders = []
  up.immediate_priority = dict(up.immediate_priority)
  return up


def test_path_helpers():
  assert to_qmpilot_path("0000002c--8c7416dba3--0/rlog.zst") == "0000002c--8c7416dba3/0/rlog.zst"
  assert qmpilot_log_id("0000002c--8c7416dba3--0/rlog.zst") == "0000002c--8c7416dba3"
  assert qmpilot_filename("rlog", "seg/rlog") == "rlog.zst"


def test_should_upload_defers_unsaved_without_upload_url(qmpilot_uploader, monkeypatch):
  up = qmpilot_uploader
  calls = []

  def fake_get(url, **kwargs):
    calls.append(url)
    if "upload_policy" in url:
      return _Resp(200, {"allow_heavy": False, "saved": False})
    raise AssertionError(f"unexpected GET {url}")

  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.get", fake_get)
  assert up.should_upload("rlog.zst", "00000020--deadbeef--0/rlog.zst") is False
  assert any("upload_policy" in u for u in calls)
  assert not any("upload_url" in u for u in calls)
  assert up._is_log_deferred("00000020--deadbeef")


def test_next_file_prefers_saved_heavy_over_unsaved(qmpilot_uploader, monkeypatch):
  up = qmpilot_uploader
  _write("00000020--unsavedaaa--0", "rlog.zst")
  name, key, fn = _write("0000002c--8c7416dba3--0", "rlog.zst")

  def fake_get(url, **kwargs):
    params = kwargs.get("params") or {}
    if "upload_policy" in url:
      allow = params.get("log_id") == "0000002c--8c7416dba3"
      return _Resp(200, {"allow_heavy": allow, "saved": allow})
    raise AssertionError(url)

  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.get", fake_get)
  got = up.next_file_to_upload(metered=False)
  assert got is not None
  assert got[1] == key
  assert got[0] == name


def test_upload_put_and_callback_after_200(qmpilot_uploader, monkeypatch):
  up = qmpilot_uploader
  name, key, fn = _write("0000002c--8c7416dba3--0", "rlog.zst", b"rlog")
  put_calls = []
  post_calls = []

  def fake_get(url, **kwargs):
    if "upload_policy" in url:
      return _Resp(200, {"allow_heavy": True, "saved": True})
    if "upload_url" in url:
      return _Resp(200, {"url": "https://qmpilot-connect.com/openpilot-routes/x", "headers": {}})
    raise AssertionError(url)

  def fake_put(url, data=None, headers=None, timeout=None):
    # consume stream like requests would
    payload = data.read() if hasattr(data, "read") else data
    put_calls.append({"url": url, "headers": dict(headers or {}), "payload": payload, "timeout": timeout})
    return _Resp(200)

  def fake_post(url, **kwargs):
    post_calls.append({"url": url, "json": kwargs.get("json")})
    return _Resp(200)

  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.get", fake_get)
  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.put", fake_put)
  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.post", fake_post)
  monkeypatch.setattr("openpilot.system.loggerd.uploader.fake_upload", False)

  assert up.upload(name, key, fn, network_type=1, metered=False) is True
  assert len(put_calls) == 1
  assert put_calls[0]["payload"] == b"rlog"
  assert put_calls[0]["headers"].get("Content-Length") == "4"
  assert len(post_calls) == 1
  assert post_calls[0]["json"]["path"] == "0000002c--8c7416dba3/0/rlog.zst"
  assert getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE


def test_upload_url_200_missing_local_does_not_mark_uploaded(qmpilot_uploader, monkeypatch):
  up = qmpilot_uploader
  name, key, fn = _write("0000002c--8c7416dba3--1", "rlog.zst", b"rlog")
  put = mock.Mock()

  def fake_get(url, **kwargs):
    if "upload_policy" in url:
      return _Resp(200, {"allow_heavy": True, "saved": True})
    if "upload_url" in url:
      # Race: file disappears after upload_url succeeds
      os.unlink(fn)
      return _Resp(200, {"url": "https://example/put", "headers": {}})
    raise AssertionError(url)

  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.get", fake_get)
  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.put", put)
  monkeypatch.setattr("openpilot.system.loggerd.uploader.fake_upload", False)

  assert up.upload(name, key, fn, network_type=1, metered=False) is True
  put.assert_not_called()
  assert not os.path.exists(fn)


def test_503_defers_route_keeps_file(qmpilot_uploader, monkeypatch):
  up = qmpilot_uploader
  name, key, fn = _write("00000020--unsavedaaa--0", "rlog.zst", b"rlog")

  def fake_get(url, **kwargs):
    if "upload_policy" in url:
      return _Resp(200, {"allow_heavy": True, "saved": True})  # stale/wrong — server still gates
    if "upload_url" in url:
      return _Resp(503, text="unavailable", headers={"Retry-After": "90"})
    raise AssertionError(url)

  put = mock.Mock()
  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.get", fake_get)
  monkeypatch.setattr("openpilot.system.loggerd.uploader.requests.put", put)
  monkeypatch.setattr("openpilot.system.loggerd.uploader.fake_upload", False)

  assert up.upload(name, key, fn, network_type=1, metered=False) is True
  put.assert_not_called()
  assert getxattr(fn, UPLOAD_ATTR_NAME) != UPLOAD_ATTR_VALUE
  assert up._is_log_deferred("00000020--unsavedaaa")
  assert os.path.exists(fn)
