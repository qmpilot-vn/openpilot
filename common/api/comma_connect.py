import os

from openpilot.common.api.base import BaseApi

DEFAULT_API_HOST = 'https://api.commadotai.com'


def _read_str_file(path: str) -> str | None:
  try:
    with open(path) as f:
      val = f.read().strip()
      return val or None
  except OSError:
    return None


def _read_param_str(name: str) -> str | None:
  """Read a string param.

  Order: Params API (if key is known) → /data/params/d → /persist/qmpilot/
  Persist is used because manager clearAll deletes unknown keys under /data/params/d.
  """
  try:
    from openpilot.common.params import Params
    params = Params()
    if params.check_key(name):
      val = params.get(name)
      if val:
        return val.decode() if isinstance(val, bytes) else str(val)
  except Exception:
    pass

  for path in (
    os.path.join('/data/params/d', name),
    os.path.join('/data/qmpilot', name),
    os.path.join('/persist/qmpilot', name),
  ):
    val = _read_str_file(path)
    if val:
      return val
  return None


def get_api_host() -> str:
  """Comma Connect. ApiHost selects qmpilot-connect and does not replace this."""
  return DEFAULT_API_HOST


def get_qmpilot_api_host() -> str | None:
  """qmpilot-connect host from the saved ApiHost.

  launch_env.sh sets API_HOST from the Device backend at startup. A comma
  API_HOST must not hide a saved qmpilot host. API_HOST is used only when
  ApiHost is unset and it points somewhere other than comma.
  """
  host = _read_param_str('ApiHost')
  if not host:
    host = os.getenv('API_HOST')
  if not host:
    return None
  host = host.rstrip('/')
  if host == DEFAULT_API_HOST:
    return None
  return host


def get_qmpilot_api_key() -> str | None:
  return os.getenv('QMPILOT_API_KEY') or _read_param_str('QmpilotApiKey')


# Device setting UploadBackend. One target only, so a drive is not uploaded twice.
UPLOAD_COMMA = 0
UPLOAD_QMPILOT = 1


def get_upload_backend() -> int:
  """comma or qmpilot. Anything else, including the old both value, stays on comma."""
  raw = None
  try:
    from openpilot.common.params import Params
    params = Params()
    if params.check_key("UploadBackend"):
      raw = params.get("UploadBackend")
  except Exception:
    raw = None
  if raw is None or raw == "":
    raw = _read_param_str("UploadBackend")
  try:
    value = int(raw) if raw is not None and raw != "" else UPLOAD_COMMA
  except (TypeError, ValueError):
    return UPLOAD_COMMA
  if value != UPLOAD_QMPILOT:
    return UPLOAD_COMMA
  return UPLOAD_QMPILOT


def _write_text(directory: str, name: str, value: str) -> bool:
  try:
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, name)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
      f.write(value)
    os.replace(tmp, path)
    return True
  except OSError:
    return False


def qmpilot_store_dirs(persist_dir: str = "/persist/qmpilot") -> list[str]:
  """Where a qmpilot file can be saved. /persist is read-only on this device."""
  dirs = [persist_dir]
  if persist_dir == "/persist/qmpilot":
    dirs.insert(0, "/data/qmpilot")
  return dirs


def store_qmpilot_file(name: str, value: str, persist_dir: str = "/persist/qmpilot") -> None:
  for directory in qmpilot_store_dirs(persist_dir):
    _write_text(directory, name, value)


def set_upload_backend(value: int, params=None, persist_dir: str = "/persist/qmpilot") -> None:
  """Select comma (0) or qmpilot (1).

  The running params module does not know UploadBackend. /persist is mounted
  read-only, so the value is stored under /data/qmpilot, which get_upload_backend reads.
  """
  choice = UPLOAD_QMPILOT if int(value) == UPLOAD_QMPILOT else UPLOAD_COMMA
  target = params
  if target is None:
    try:
      from openpilot.common.params import Params
      target = Params()
      target.check_key("UploadBackend")
    except Exception:
      target = None
  if target is not None:
    try:
      target.put("UploadBackend", choice, block=True)
    except Exception:
      pass
  store_qmpilot_file("UploadBackend", str(choice), persist_dir)


def comma_backend_selected() -> bool:
  return get_upload_backend() == UPLOAD_COMMA


def qmpilot_backend_selected() -> bool:
  """qmpilot-connect uploader. The Device toggle must be qmpilot, with ApiHost and QmpilotApiKey."""
  if get_upload_backend() != UPLOAD_QMPILOT:
    return False
  return bool(get_qmpilot_api_key()) and get_qmpilot_api_host() is not None


def get_athena_host() -> str:
  """Comma Athena, or qmpilot-connect when that upload backend is selected.

  The socket is wss://<host>/ws/v2/<dongle_id>. uploadFilesToUrls only arrives
  on whichever host this returns.
  """
  if qmpilot_backend_selected():
    return os.getenv("QMPILOT_ATHENA_HOST", "wss://qmpilot-connect.com").rstrip("/")
  return os.getenv("ATHENA_HOST", "wss://athena.comma.ai").rstrip("/")


class CommaConnectApi(BaseApi):
  def __init__(self, dongle_id):
    super().__init__(dongle_id, get_api_host())
    self.user_agent = "openpilot-"
