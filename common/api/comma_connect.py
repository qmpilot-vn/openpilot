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
  """qmpilot-connect host. Priority: API_HOST env, then ApiHost param/persist.

  Unset, or the comma host, means this backend is not selected.
  """
  host = os.getenv('API_HOST') or _read_param_str('ApiHost')
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


def comma_backend_selected() -> bool:
  return get_upload_backend() == UPLOAD_COMMA


def qmpilot_backend_selected() -> bool:
  """qmpilot-connect uploader. The Device toggle must be qmpilot, with ApiHost and QmpilotApiKey."""
  if get_upload_backend() != UPLOAD_QMPILOT:
    return False
  return bool(get_qmpilot_api_key()) and get_qmpilot_api_host() is not None


class CommaConnectApi(BaseApi):
  def __init__(self, dongle_id):
    super().__init__(dongle_id, get_api_host())
    self.user_agent = "openpilot-"
