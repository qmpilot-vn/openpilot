import os

from openpilot.common.params_pyx import Params as _Params, ParamKeyFlag, ParamKeyType, UnknownKeyName
from openpilot.common.swaglog import cloudlog

assert ParamKeyFlag
assert ParamKeyType
assert UnknownKeyName

# clearAll unlinks every on-disk file it does not recognize. The key table
# lives in params_pyx.so, so a newly dropped .so (empty table, or built from
# another params_keys.h) would wipe /data/params on the next manager start.
# The cython type is immutable, so the guard is a Python subclass. It is live
# as soon as manager imports Params, including with a .so that does not yet
# contain the same check in params.cc.
_MIN_KEY_TABLE = 50
_UNKNOWN_WIPE_COUNT = 8


def clear_would_wipe(known_keys, on_disk_names) -> bool:
  known = set(known_keys)
  names = [n for n in on_disk_names if n and not str(n).startswith(".")]
  if len(known) < _MIN_KEY_TABLE:
    return True
  unknown = sum(1 for n in names if n not in known)
  return bool(names) and unknown >= _UNKNOWN_WIPE_COUNT and unknown * 2 > len(names)


def _key_names(params):
  try:
    raw = params.all_keys()
  except Exception:
    cloudlog.exception("params: key table unreadable; refusing clear_all")
    return None
  names = set()
  for key in raw:
    names.add(key.decode() if isinstance(key, bytes) else str(key))
  return names


class Params(_Params):
  def clear_all(self, tx_flag=ParamKeyFlag.ALL):
    known = _key_names(self)
    if known is None:
      return None
    try:
      on_disk = os.listdir(self.get_param_path())
    except OSError:
      on_disk = []
    if clear_would_wipe(known, on_disk):
      unknown = [n for n in on_disk if n and not n.startswith(".") and n not in known]
      cloudlog.error(
        "params: refusing clear_all, key table has %d entries, %d of %d on-disk params are unknown",
        len(known), len(unknown), len([n for n in on_disk if n and not n.startswith(".")]),
      )
      return None
    return super().clear_all(tx_flag)

if __name__ == "__main__":
  import sys

  params = Params()
  key = sys.argv[1]
  assert params.check_key(key), f"unknown param: {key}"

  if len(sys.argv) == 3:
    val = sys.argv[2]
    print(f"SET: {key} = {val}")
    params.put(key, val, block=True)
  elif len(sys.argv) == 2:
    print(f"GET: {key} = {params.get(key)}")
