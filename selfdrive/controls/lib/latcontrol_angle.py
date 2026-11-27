"""_NATIVE_SHIM — logic is loaded from latcontrol_angle_impl*.so (same Python/ABI as build)."""
import importlib.machinery
import importlib.util
import sys
from pathlib import Path

_IMPL_STEM = "latcontrol_angle_impl"
_dir = Path(__file__).resolve().parent
_so = next((p for suf in importlib.machinery.EXTENSION_SUFFIXES if (p := _dir / f"{_IMPL_STEM}{suf}").is_file()), None)
if _so is None:
  raise ImportError(
    "Missing native %s*.so — rebuild with opendbc_repo/scripts/package_vinfast_so.sh using Python %s"
    % (_IMPL_STEM, ".".join(map(str, sys.version_info[:3])))
  )
# Load under the extension's short name so PyInit_<stem> matches, then publish as this module.
_spec = importlib.util.spec_from_file_location(_IMPL_STEM, _so)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
sys.modules[_IMPL_STEM] = _mod
try:
  _spec.loader.exec_module(_mod)
finally:
  sys.modules.pop(_IMPL_STEM, None)
# Cython tags classes with __module__=_IMPL_STEM; car_helpers needs opendbc.car.<brand>.*
for _attr in dir(_mod):
  _obj = getattr(_mod, _attr)
  if getattr(_obj, "__module__", None) == _IMPL_STEM:
    try:
      _obj.__module__ = __name__
    except (AttributeError, TypeError):
      pass
_mod.__name__ = __name__
_mod.__package__ = __package__
_mod.__file__ = str(_so)
_mod.__loader__ = _spec.loader
_mod.__spec__ = _spec
sys.modules[__name__] = _mod

# VF8 Plus uses VF6 safety, but the angle PID stays on the VF8 tune.
_orig_is_vf6 = getattr(_mod, "is_vf6_safety_platform", None)
if _orig_is_vf6 is not None:
  def _is_vf6_safety_platform(candidate):
    name = candidate if isinstance(candidate, str) else getattr(candidate, "name", None)
    if name == "VINFAST_VF8_PLUS":
      return False
    return _orig_is_vf6(candidate)
  _mod.is_vf6_safety_platform = _is_vf6_safety_platform
