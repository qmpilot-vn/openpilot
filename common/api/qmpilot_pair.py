"""Pair a Comma 4 with qmpilot-connect. The API key never goes in the QR."""
import os
import threading
import time
from urllib.parse import urlencode

from openpilot.common.api.base import BaseApi
from openpilot.common.api.comma_connect import UPLOAD_QMPILOT

PAIR_HOST = "https://qmpilot-connect.com"


class PairRejected(Exception):
  """401: the JWT or public key was rejected."""


class PairConflict(Exception):
  """409: this dongle is already registered to another key."""


def pair_url(dongle_id: str, pair_nonce: str) -> str:
  query = urlencode({"dongle_id": dongle_id, "pair_nonce": pair_nonce})
  return f"{PAIR_HOST}/pair?{query}"


def save_pair(params, api_host: str, api_key: str, persist_dir: str = "/persist/qmpilot") -> None:
  """Store the key where the uploader reads it, and select the qmpilot upload target."""
  host = str(api_host).rstrip("/")
  params.put("ApiHost", host, block=True)
  params.put("QmpilotApiKey", api_key, block=True)
  params.put("UploadBackend", UPLOAD_QMPILOT, block=True)
  os.makedirs(persist_dir, exist_ok=True)
  for name, value in (("ApiHost", host), ("QmpilotApiKey", api_key)):
    path = os.path.join(persist_dir, name)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
      f.write(value)
    os.replace(tmp, path)


class QmpilotPairClient:
  def __init__(self, dongle_id: str, api: BaseApi | None = None, public_key: str | None = None):
    self.dongle_id = dongle_id
    self.api = api or BaseApi(dongle_id, PAIR_HOST)
    if public_key is None:
      _, _, public_key = BaseApi.get_key_pair()
    self.public_key = public_key

  def _token(self) -> str:
    return self.api.get_token()

  def start(self) -> tuple[str, int]:
    if not self.public_key:
      raise PairRejected("missing public key")
    resp = self.api.post(
      "v1/pair/start",
      timeout=15,
      access_token=self._token(),
      json={"public_key": self.public_key},
    )
    self._raise_for_status(resp)
    data = resp.json()
    return str(data["pair_nonce"]), int(data.get("expires_in") or 300)

  def status(self) -> dict:
    resp = self.api.get("v1/pair/status", timeout=10, access_token=self._token())
    self._raise_for_status(resp)
    data = resp.json()
    return data if isinstance(data, dict) else {}

  @staticmethod
  def _raise_for_status(resp) -> None:
    if resp.status_code == 401:
      raise PairRejected(str(resp.status_code))
    if resp.status_code == 409:
      raise PairConflict(str(resp.status_code))
    if resp.status_code != 200:
      raise RuntimeError(f"pair http {resp.status_code}")


class PairWorker:
  """Calls pair/start, then polls pair/status until a key arrives or the nonce is about to expire."""

  def __init__(self, client: QmpilotPairClient, on_wifi, poll_s: float = 2.0, refresh_lead_s: float = 20.0):
    self.client = client
    self.on_wifi = on_wifi
    self.poll_s = poll_s
    self.refresh_lead_s = refresh_lead_s
    self._lock = threading.Lock()
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None
    self.url = ""
    self.message = ""
    self.api_key: str | None = None
    self.api_host: str | None = None

  def start(self) -> None:
    self._thread = threading.Thread(target=self._run, name="qmpilot-pair", daemon=True)
    self._thread.start()

  def stop(self) -> None:
    self._stop.set()
    if self._thread is not None:
      self._thread.join(timeout=2)

  def snapshot(self) -> tuple[str, str, str | None, str | None]:
    with self._lock:
      return self.url, self.message, self.api_key, self.api_host

  def _set(self, **kwargs) -> None:
    with self._lock:
      for name, value in kwargs.items():
        setattr(self, name, value)

  def _run(self) -> None:
    while not self._stop.is_set():
      if not self.on_wifi():
        self._set(message="wifi", url="")
        self._stop.wait(1)
        continue
      try:
        nonce, expires = self.client.start()
      except PairConflict:
        self._set(message="conflict")
        return
      except PairRejected:
        self._set(message="rejected")
        return
      except Exception:
        self._set(message="start_failed", url="")
        self._stop.wait(3)
        continue

      url = pair_url(self.client.dongle_id, nonce)
      self._set(url=url, message="")
      deadline = time.monotonic() + max(self.poll_s, float(expires) - self.refresh_lead_s)
      while time.monotonic() < deadline and not self._stop.is_set():
        try:
          body = self.client.status()
        except PairConflict:
          self._set(message="conflict")
          return
        except PairRejected:
          self._set(message="rejected")
          return
        except Exception:
          self._stop.wait(self.poll_s)
          continue
        key = body.get("api_key")
        if isinstance(key, str) and key:
          host = body.get("api_host") if isinstance(body.get("api_host"), str) else PAIR_HOST
          self._set(api_key=key, api_host=host.rstrip("/"), message="paired")
          return
        self._stop.wait(self.poll_s)
