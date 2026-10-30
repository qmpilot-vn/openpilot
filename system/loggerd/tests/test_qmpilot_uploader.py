from pathlib import Path

import openpilot.system.loggerd.qmpilot_uploader as qmpilot
from openpilot.common.api.comma_connect import (
  DEFAULT_API_HOST,
  UPLOAD_COMMA,
  UPLOAD_QMPILOT,
  comma_backend_selected,
  get_api_host,
  get_athena_host,
  get_qmpilot_api_host,
  qmpilot_backend_selected,
)
from openpilot.system.loggerd.xattr_cache import getxattr


class Resp:
  def __init__(self, status_code, text="", headers=None):
    self.status_code = status_code
    self.text = text
    self.headers = headers or {}
    self.request = type("Req", (), {"headers": {"Content-Length": "4"}})()


def test_comma_host_stays_comma(monkeypatch):
  monkeypatch.setenv("API_HOST", "https://qmpilot-connect.com")
  monkeypatch.setenv("QMPILOT_API_KEY", "test-key")
  assert get_api_host() == DEFAULT_API_HOST
  assert get_qmpilot_api_host() == "https://qmpilot-connect.com"
  assert comma_backend_selected()
  assert not qmpilot_backend_selected()


def test_upload_toggle_selects_one_backend(monkeypatch):
  monkeypatch.setenv("API_HOST", "https://qmpilot-connect.com")
  monkeypatch.setenv("QMPILOT_API_KEY", "test-key")
  monkeypatch.setattr("openpilot.common.api.comma_connect.get_upload_backend", lambda: UPLOAD_COMMA)
  assert comma_backend_selected()
  assert not qmpilot_backend_selected()
  monkeypatch.setattr("openpilot.common.api.comma_connect.get_upload_backend", lambda: UPLOAD_QMPILOT)
  assert not comma_backend_selected()
  assert qmpilot_backend_selected()


def test_old_both_value_uploads_to_comma_only(monkeypatch):
  class Params:
    def check_key(self, name):
      return name == "UploadBackend"

    def get(self, name):
      return 2

  monkeypatch.setattr("openpilot.common.params.Params", Params)
  monkeypatch.setenv("API_HOST", "https://qmpilot-connect.com")
  monkeypatch.setenv("QMPILOT_API_KEY", "test-key")
  from openpilot.common.api.comma_connect import get_upload_backend
  assert get_upload_backend() == UPLOAD_COMMA
  assert comma_backend_selected()
  assert not qmpilot_backend_selected()


def test_comma_env_host_does_not_hide_saved_qmpilot_host(monkeypatch):
  monkeypatch.setenv("API_HOST", DEFAULT_API_HOST)

  def read(name):
    if name == "ApiHost":
      return "https://qmpilot-connect.com/"
    return None

  monkeypatch.setattr("openpilot.common.api.comma_connect._read_param_str", read)
  assert get_qmpilot_api_host() == "https://qmpilot-connect.com"


def test_qmpilot_not_selected_without_key(monkeypatch):
  monkeypatch.delenv("QMPILOT_API_KEY", raising=False)
  monkeypatch.delenv("API_HOST", raising=False)
  monkeypatch.setattr("openpilot.common.api.comma_connect._read_param_str", lambda name: None)
  assert get_qmpilot_api_host() is None
  assert not qmpilot_backend_selected()


def test_athena_host_follows_upload_backend(monkeypatch):
  from openpilot.common.api import comma_connect
  monkeypatch.setattr(comma_connect, "qmpilot_backend_selected", lambda: True)
  assert get_athena_host() == "wss://qmpilot-connect.com"
  monkeypatch.setattr(comma_connect, "qmpilot_backend_selected", lambda: False)
  assert get_athena_host() == "wss://athena.comma.ai"


def test_path_rewrite():
  assert qmpilot.to_qmpilot_path("000003e3--d68e850585--0/qlog.zst") == "000003e3--d68e850585/0/qlog.zst"
  assert qmpilot.to_qmpilot_path("boot/00000004--0ac3964c96/qlog.zst") == "boot/00000004--0ac3964c96/qlog.zst"


def _uploader(monkeypatch, tmp_path: Path):
  monkeypatch.setattr(qmpilot, "get_qmpilot_api_host", lambda: "https://qmpilot-connect.com")
  monkeypatch.setattr(qmpilot, "get_qmpilot_api_key", lambda: "test-key")
  monkeypatch.setattr(qmpilot, "fake_upload", True)
  return qmpilot.Uploader("0123456789abcdef", str(tmp_path))


def _segment(tmp_path: Path, name: str = "qlog") -> tuple[str, str, str]:
  seg = tmp_path / "000003e3--d68e850585--0"
  seg.mkdir(parents=True)
  fn = seg / name
  fn.write_bytes(b"log")
  key = f"000003e3--d68e850585--0/{name}"
  if name in ("qlog", "rlog"):
    key += ".zst"
  return name, key, str(fn)


def test_callback_marks_qmpilot_only(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  name, key, fn = _segment(tmp_path)
  posts = []

  def get(url, **kwargs):
    assert url == "https://qmpilot-connect.com/v1.4/0123456789abcdef/upload_url/"
    assert kwargs["params"]["path"] == "000003e3--d68e850585/0/qlog.zst"
    assert kwargs["headers"]["X-Device-Key"] == "test-key"
    return Resp(200, '{"url": "http://upload.local/obj", "headers": {}}')

  def post(url, **kwargs):
    posts.append(kwargs["json"])
    return Resp(200)

  monkeypatch.setattr(qmpilot.requests, "get", get)
  monkeypatch.setattr(qmpilot.requests, "post", post)

  assert up.upload(name, key, fn, 1, False)
  assert posts == [{"dongle_id": "0123456789abcdef", "path": "000003e3--d68e850585/0/qlog.zst"}]
  assert getxattr(fn, qmpilot.UPLOAD_ATTR_NAME) == qmpilot.UPLOAD_ATTR_VALUE
  assert qmpilot.UPLOAD_ATTR_NAME != "user.upload"


def test_saved_route_required_for_heavy_file(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  name, key, fn = _segment(tmp_path, "rlog.zst")

  def get(url, **kwargs):
    return Resp(503, headers={"Retry-After": "60"})

  monkeypatch.setattr(qmpilot.requests, "get", get)
  assert up.upload(name, key, fn, 1, False)
  assert getxattr(fn, qmpilot.UPLOAD_ATTR_NAME) is None
  assert up._defer_until[fn] > 0
  assert list(up.list_upload_files(False)) == []


def test_auth_failure_is_not_done(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  name, key, fn = _segment(tmp_path)

  monkeypatch.setattr(qmpilot.requests, "get", lambda *a, **k: Resp(401, "no"))
  assert not up.upload(name, key, fn, 1, False)
  assert getxattr(fn, qmpilot.UPLOAD_ATTR_NAME) is None


def test_boot_is_left_for_comma(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  boot = tmp_path / "boot"
  boot.mkdir()
  (boot / "00000004--0ac3964c96").write_bytes(b"boot")
  assert list(up.list_upload_files(False)) == []


def test_lte_keeps_qlog_and_holds_heavy(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  seg = tmp_path / "000003e3--d68e850585--0"
  seg.mkdir()
  (seg / "qlog").write_bytes(b"q")
  (seg / "rlog.zst").write_bytes(b"r")
  names = [name for name, _, _ in up.list_upload_files(True)]
  assert names == ["qlog"]


def test_heavy_files_are_not_auto_queued(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  older = tmp_path / "00000054--aaaaaaaaaa--30"
  newer = tmp_path / "0000005f--bbbbbbbbbb--0"
  older.mkdir()
  newer.mkdir()
  (older / "fcamera.hevc").write_bytes(b"c")
  (older / "rlog.zst").write_bytes(b"r")
  (older / "qlog.zst").write_bytes(b"q")
  (newer / "rlog.zst").write_bytes(b"r")
  (newer / "qlog.zst").write_bytes(b"q")

  keys = [key for _, key, _ in up.list_upload_files(False)]
  assert keys == [
    "00000054--aaaaaaaaaa--30/qlog.zst",
    "0000005f--bbbbbbbbbb--0/qlog.zst",
  ]


def test_heavy_timeout_defers_and_qlog_does_not(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  monkeypatch.setattr(qmpilot, "fake_upload", False)
  camera_name, camera_key, camera_fn = _segment(tmp_path, "fcamera.hevc")
  qlog_name, qlog_key, qlog_fn = _segment(tmp_path / "other", "qlog")

  def get(url, **kwargs):
    return Resp(200, '{"url": "http://upload.local/obj", "headers": {}}')

  def put(url, **kwargs):
    raise qmpilot.requests.exceptions.Timeout("write")

  monkeypatch.setattr(qmpilot.requests, "get", get)
  monkeypatch.setattr(qmpilot.requests, "put", put)

  assert up.upload(camera_name, camera_key, camera_fn, 1, False)
  assert getxattr(camera_fn, qmpilot.UPLOAD_ATTR_NAME) is None
  assert up._defer_until[camera_fn] > 0

  assert not up.upload(qlog_name, qlog_key, qlog_fn, 1, False)
  assert qlog_fn not in up._defer_until
  assert getxattr(qlog_fn, qmpilot.UPLOAD_ATTR_NAME) is None


def test_qcamera_timeout_steps_aside_for_the_next_qlog(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  monkeypatch.setattr(qmpilot, "fake_upload", False)
  older = tmp_path / "00000057--af92006b37--7"
  newer = tmp_path / "00000057--af92006b37--8"
  older.mkdir()
  newer.mkdir()
  qcamera_fn = older / "qcamera.ts"
  qlog_fn = newer / "qlog.zst"
  qcamera_fn.write_bytes(b"cam" * 1000)
  qlog_fn.write_bytes(b"qlog")

  timeouts = []

  def get(url, **kwargs):
    return Resp(200, '{"url": "http://upload.local/obj", "headers": {}}')

  def put(url, **kwargs):
    timeouts.append(kwargs["timeout"])
    raise qmpilot.requests.exceptions.Timeout("write")

  monkeypatch.setattr(qmpilot.requests, "get", get)
  monkeypatch.setattr(qmpilot.requests, "put", put)

  assert up.upload("qcamera.ts", "00000057--af92006b37--7/qcamera.ts", str(qcamera_fn), 1, False)
  assert timeouts == [(10.0, 30.0)]
  assert getxattr(str(qcamera_fn), qmpilot.UPLOAD_ATTR_NAME) is None
  assert up._defer_until[str(qcamera_fn)] > 0
  assert [name for name, _, _ in up.list_upload_files(False)] == ["qlog.zst"]

  assert not up.upload("qlog.zst", "00000057--af92006b37--8/qlog.zst", str(qlog_fn), 1, False)
  assert timeouts[1][1] >= 120.0
  assert str(qlog_fn) not in up._defer_until


def test_412_is_done_only_after_callback(monkeypatch, tmp_path):
  up = _uploader(monkeypatch, tmp_path)
  name, key, fn = _segment(tmp_path)
  posts = []

  monkeypatch.setattr(qmpilot.requests, "get", lambda *a, **k: Resp(412, "already"))

  def post(url, **kwargs):
    posts.append(kwargs["json"])
    return Resp(500, "no")

  monkeypatch.setattr(qmpilot.requests, "post", post)
  assert not up.upload(name, key, fn, 1, False)
  assert posts
  assert getxattr(fn, qmpilot.UPLOAD_ATTR_NAME) is None

  monkeypatch.setattr(qmpilot.requests, "post", lambda *a, **k: Resp(200))
  assert up.upload(name, key, fn, 1, False)
  assert getxattr(fn, qmpilot.UPLOAD_ATTR_NAME) == qmpilot.UPLOAD_ATTR_VALUE
