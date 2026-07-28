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
  # Priority: env > ApiHost param/persist > comma default
  host = os.getenv('API_HOST') or _read_param_str('ApiHost') or DEFAULT_API_HOST
  return host.rstrip('/')


def get_qmpilot_api_key() -> str | None:
  return os.getenv('QMPILOT_API_KEY') or _read_param_str('QmpilotApiKey')


class CommaConnectApi(BaseApi):
  def __init__(self, dongle_id):
    super().__init__(dongle_id, get_api_host())
    self.user_agent = "openpilot-"
