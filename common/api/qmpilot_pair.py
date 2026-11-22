"""Pair a Comma 4 with qmpilot-connect. The API key never goes in the QR."""
import threading
import time
from urllib.parse import urlencode

from openpilot.common.api.base import BaseApi
from openpilot.common.api.comma_connect import set_upload_backend, store_qmpilot_file

PAIR_HOST = "https://qmpilot-connect.com"
# Keep the QR screen awake for a full nonce (5 min) plus Google login headroom.
PAIR_UI_TIMEOUT_S = 360


class PairRejected(Exception):
  """401: the JWT or public key was rejected."""


class PairConflict(Exception):
  """409: this dongle is already registered to another key."""


def pair_url(dongle_id: str, pair_nonce: str) -> str:
  query = urlencode({"dongle_id": dongle_id, "pair_nonce": pair_nonce})
  return f"{PAIR_HOST}/pair?{query}"


def _put_param(params, key: str, value) -> None:
  """Params may not know a key in the running params_pyx.so. The file copy still has to land."""
  try:
    params.put(key, value, block=True)
  except Exception:
    pass


def save_pair(params, api_host: str, api_key: str, persist_dir: str = "/persist/qmpilot") -> None:
  """Store the key where the uploader reads it, and select the qmpilot upload target."""
  host = str(api_host).rstrip("/")
  _put_param(params, "ApiHost", host)
  _put_param(params, "QmpilotApiKey", api_key)
  set_upload_backend(1, params, persist_dir)
  store_qmpilot_file("ApiHost", host, persist_dir)
  store_qmpilot_file("QmpilotApiKey", api_key, persist_dir)


def key_from_status(body: dict) -> tuple[str, str] | None:
  """A string api_key on pair/status is the device's key, including when paired is already true.

  paired: false with api_key: null means still waiting. Only a non-empty api_key counts.
  """
  if not isinstance(body, dict):
    return None
  key = body.get("api_key")
  if not isinstance(key, str) or not key:
    return None
  host = body.get("api_host") if isinstance(body.get("api_host"), str) else PAIR_HOST
  return key, host.rstrip("/")


class QmpilotPairClient:
  def __init__(self, dongle_id: str, api: BaseApi | None = None, public_key: str | None = None):
    self.dongle_id = dongle_id
    self.api = api or BaseApi(dongle_id, PAIR_HOST)
    if public_key is None:
      _, _, public_key = BaseApi.get_key_pair()
    self.public_key = public_key

  def _token(self) -> str:
    return self.api.get_token()

  def start(self) -> tuple[str | None, int]:
    """Return (pair_nonce, expires_in). nonce is None when the car is already owned.

    pair/start never returns the API key. When paired is already true the phone
    link may have finished; the key still comes from pair/status.
    """
    if not self.public_key:
      raise PairRejected("missing public key")
    resp = self.api.post(
      "v1/pair/start",
      timeout=15,
      access_token=self._token(),
      json={"public_key": self.public_key},
    )
    self._raise_for_status(resp)
    data = resp.json() if hasattr(resp, "json") else {}
    if not isinstance(data, dict):
      data = {}
    expires = int(data.get("expires_in") or 300)
    nonce = data.get("pair_nonce")
    if isinstance(nonce, str) and nonce:
      return nonce, expires
    # Already owned / no fresh QR: still poll status for the key.
    if data.get("paired") is True:
      return None, expires
    raise RuntimeError("pair/start missing pair_nonce")

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
  """Poll pair/status until api_key appears. Do not stop on paired: false.

  Reads status first so a car the phone already linked gets its key with no new QR.
  pair/start runs only when status has no key yet. A 409 from start checks status
  once more, because qmpilot keeps the key on status after the phone links.
  """

  def __init__(self, client: QmpilotPairClient, on_wifi, poll_s: float = 2.0,
               refresh_lead_s: float = 20.0, on_key=None):
    self.client = client
    self.on_wifi = on_wifi
    self.poll_s = poll_s
    self.refresh_lead_s = refresh_lead_s
    self.on_key = on_key
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

  def _apply_status(self, body: dict) -> bool:
    found = key_from_status(body)
    if found is None:
      return False
    key, host = found
    self._set(api_key=key, api_host=host, message="paired")
    if self.on_key is not None:
      try:
        self.on_key(host, key)
      except Exception:
        # UI may retry save from snapshot; keep message=paired so the screen can finish.
        pass
    return True

  def _read_status(self) -> str:
    """paired, rejected, or empty when there is no key yet. Network errors stay empty."""
    try:
      body = self.client.status()
    except PairRejected:
      return "rejected"
    except Exception:
      return ""
    if self._apply_status(body):
      return "paired"
    return ""

  def _poll_until_key_or_deadline(self, deadline: float) -> str:
    """Keep calling pair/status every poll_s until api_key, reject, stop, or deadline."""
    while time.monotonic() < deadline and not self._stop.is_set():
      found = self._read_status()
      if found == "paired":
        return "paired"
      if found == "rejected":
        self._set(message="rejected")
        return "rejected"
      self._stop.wait(self.poll_s)
    return ""

  def _run(self) -> None:
    while not self._stop.is_set():
      if not self.on_wifi():
        self._set(message="wifi", url="")
        self._stop.wait(1)
        continue
      found = self._read_status()
      if found == "paired":
        return
      if found == "rejected":
        self._set(message="rejected")
        return
      try:
        nonce, expires = self.client.start()
      except PairConflict:
        found = self._read_status()
        if found == "paired":
          return
        self._set(message="rejected" if found == "rejected" else "conflict")
        return
      except PairRejected:
        self._set(message="rejected")
        return
      except Exception:
        self._set(message="start_failed", url="")
        self._stop.wait(3)
        continue

      if nonce:
        url = pair_url(self.client.dongle_id, nonce)
        self._set(url=url, message="")
      else:
        # pair/start said already paired but left the key on status only.
        self._set(url="", message="")

      # Poll for nearly the full nonce life. paired:false / api_key:null stays here.
      deadline = time.monotonic() + max(self.poll_s, float(expires) - self.refresh_lead_s)
      found = self._poll_until_key_or_deadline(deadline)
      if found == "paired":
        return
      if found == "rejected":
        return
      # Deadline: outer loop calls pair/start again for a fresh nonce / QR.
