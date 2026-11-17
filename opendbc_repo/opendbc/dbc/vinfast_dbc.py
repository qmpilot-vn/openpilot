"""Load packed VinFast DBC blobs from vinfast_dbc_impl*.so."""
import importlib.machinery
import importlib.util
import sys
from pathlib import Path

_IMPL_STEM = "vinfast_dbc_impl"
_dir = Path(__file__).resolve().parent
_so = next((p for suf in importlib.machinery.EXTENSION_SUFFIXES if (p := _dir / f"{_IMPL_STEM}{suf}").is_file()), None)
if _so is None:
  raise ImportError(f"Missing native {_IMPL_STEM}*.so")

_spec = importlib.util.spec_from_file_location(_IMPL_STEM, _so)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
sys.modules[_IMPL_STEM] = _mod
_spec.loader.exec_module(_mod)
sys.modules.pop(_IMPL_STEM, None)

def get(name: str) -> bytes | None:
  return _mod.get(name)
