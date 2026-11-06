import time

from openpilot.common.api.comma_connect import UPLOAD_QMPILOT
from openpilot.common.api.qmpilot_pair import (
  PAIR_HOST,
  PairConflict,
  PairRejected,
  PairWorker,
  pair_url,
  save_pair,
)


class MemParams:
  def __init__(self):
    self.store = {}

  def put(self, key, value, block=False):
    self.store[key] = value


def test_pair_url_has_no_api_key():
  url = pair_url("0123456789abcdef", "nonce-1")
  assert url == "https://qmpilot-connect.com/pair?dongle_id=0123456789abcdef&pair_nonce=nonce-1"
  assert "api_key" not in url
  assert PAIR_HOST in url


def test_save_pair_selects_qmpilot_and_persists(tmp_path):
  params = MemParams()
  save_pair(params, "https://qmpilot-connect.com/", "secret-key", persist_dir=str(tmp_path))
  assert params.store["ApiHost"] == "https://qmpilot-connect.com"
  assert params.store["QmpilotApiKey"] == "secret-key"
  assert params.store["UploadBackend"] == UPLOAD_QMPILOT
  assert (tmp_path / "ApiHost").read_text() == "https://qmpilot-connect.com"
  assert (tmp_path / "QmpilotApiKey").read_text() == "secret-key"
  assert (tmp_path / "UploadBackend").read_text() == "1"


class FakeClient:
  def __init__(self, steps):
    self.dongle_id = "0123456789abcdef"
    self.steps = list(steps)
    self.starts = 0

  def start(self):
    self.starts += 1
    step = self.steps.pop(0)
    if isinstance(step, Exception):
      raise step
    return step

  def status(self):
    step = self.steps.pop(0)
    if isinstance(step, Exception):
      raise step
    return step


def _wait_done(worker, timeout=2.0):
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    url, message, key, host = worker.snapshot()
    if message in ("paired", "conflict", "rejected") or key:
      return url, message, key, host
    time.sleep(0.01)
  return worker.snapshot()


def test_worker_stores_key_from_status():
  client = FakeClient([
    {"api_key": None},
    ("nonce-1", 300),
    {"api_key": None},
    {"api_key": "the-key", "api_host": "https://qmpilot-connect.com"},
  ])
  worker = PairWorker(client, lambda: True, poll_s=0.01, refresh_lead_s=20)
  worker.start()
  try:
    url, message, key, host = _wait_done(worker)
  finally:
    worker.stop()
  assert message == "paired"
  assert key == "the-key"
  assert host == "https://qmpilot-connect.com"
  assert url.endswith("pair_nonce=nonce-1")
  assert "the-key" not in url


def test_worker_stops_on_conflict():
  client = FakeClient([
    {"api_key": None, "paired": False},
    PairConflict("409"),
    {"api_key": None, "paired": True},
  ])
  worker = PairWorker(client, lambda: True, poll_s=0.01)
  worker.start()
  try:
    _, message, key, _ = _wait_done(worker)
  finally:
    worker.stop()
  assert message == "conflict"
  assert key is None
  assert client.starts == 1


def test_worker_claims_key_without_a_new_qr():
  client = FakeClient([
    {"paired": True, "api_key": "already", "api_host": "https://qmpilot-connect.com"},
  ])
  worker = PairWorker(client, lambda: True, poll_s=0.01)
  worker.start()
  try:
    url, message, key, host = _wait_done(worker)
  finally:
    worker.stop()
  assert message == "paired"
  assert key == "already"
  assert host == "https://qmpilot-connect.com"
  assert url == ""
  assert client.starts == 0


def test_worker_claims_key_after_start_conflict():
  client = FakeClient([
    {"api_key": None, "paired": False},
    PairConflict("409"),
    {"paired": True, "api_key": "still-here", "api_host": PAIR_HOST},
  ])
  worker = PairWorker(client, lambda: True, poll_s=0.01)
  worker.start()
  try:
    _, message, key, host = _wait_done(worker)
  finally:
    worker.stop()
  assert message == "paired"
  assert key == "still-here"
  assert host == PAIR_HOST
  assert client.starts == 1


def test_worker_stops_on_rejected_status():
  client = FakeClient([
    {"api_key": None},
    ("nonce-1", 300),
    PairRejected("401"),
  ])
  worker = PairWorker(client, lambda: True, poll_s=0.01, refresh_lead_s=0)
  worker.start()
  try:
    _, message, _, _ = _wait_done(worker)
  finally:
    worker.stop()
  assert message == "rejected"


def test_worker_waits_for_wifi():
  online = {"ok": False}

  def wifi():
    return online["ok"]

  client = FakeClient([
    {"api_key": None},
    ("nonce-1", 300),
    {"api_key": "k", "api_host": PAIR_HOST},
  ])
  worker = PairWorker(client, wifi, poll_s=0.01)
  worker.start()
  try:
    time.sleep(0.05)
    assert client.starts == 0
    assert worker.snapshot()[1] == "wifi"
    online["ok"] = True
    _, message, key, _ = _wait_done(worker)
  finally:
    worker.stop()
  assert message == "paired"
  assert key == "k"
