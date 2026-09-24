"""_BYTECODE_SHIM — logic is loaded from __pycache__/latcontrol_angle_impl.*.pyc (same Python minor as build)."""
import importlib.util
import sys
from pathlib import Path

_IMPL_STEM = "latcontrol_angle_impl"
_pyc = Path(__file__).resolve().parent / "__pycache__" / f"{_IMPL_STEM}.{sys.implementation.cache_tag}.pyc"
if not _pyc.is_file():
  raise ImportError(
    "Missing bytecode %s — rebuild with package_vinfast_modules_bytecode.sh using Python %s"
    % (_pyc, ".".join(map(str, sys.version_info[:3])))
  )
_spec = importlib.util.spec_from_file_location(__name__, _pyc)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
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
