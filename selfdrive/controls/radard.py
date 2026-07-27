"""_BYTECODE_SHIM — logic is loaded from __pycache__/radard_impl.*.pyc (same Python minor as build)."""
import importlib.util
import sys
from pathlib import Path

_IMPL_STEM = "radard_impl"
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
