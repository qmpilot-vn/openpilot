import os
import tempfile
import unittest

from opendbc import DBC_PATH
from opendbc.can.dbc import DBC
from opendbc.can.parser import CANParser


VINFAST_DBCS = (
  "vinfast_vf6_chassis_can",
  "vinfast_vf6_info_can",
  "vinfast_vf8_chassis_can",
)


def _dbc_paths(name: str) -> tuple[str, str]:
  return (
    os.path.join(DBC_PATH, f"{name}.dbc"),
    os.path.join(DBC_PATH, f"{name}.dbc.cache"),
  )


def _load_reference_dbc(name: str) -> DBC:
  plain_path, cache_path = _dbc_paths(name)
  if os.path.isfile(plain_path):
    return DBC(name)
  if os.path.isfile(cache_path):
    return DBC(cache_path)
  raise unittest.SkipTest(f"{name} not present (need .dbc or .dbc.cache)")


def _dbc_present(name: str) -> bool:
  plain_path, cache_path = _dbc_paths(name)
  return os.path.isfile(plain_path) or os.path.isfile(cache_path)


class TestVinFastDbcCache(unittest.TestCase):
  def test_cache_matches_reference(self):
    for name in VINFAST_DBCS:
      with self.subTest(dbc=name):
        if not _dbc_present(name):
          self.skipTest(f"{name} not present")
        reference = _load_reference_dbc(name)
        with tempfile.TemporaryDirectory() as tmp:
          cache_path = os.path.join(tmp, f"{name}.dbc.cache")
          with open(cache_path, "wb") as f:
            f.write(DBC.cache_blob(reference))

          cached = DBC(cache_path)
          self.assertEqual(cached.name, reference.name)
          self.assertEqual(len(cached.msgs), len(reference.msgs))
          self.assertEqual(set(cached.msgs), set(reference.msgs))

  def test_parser_loads_from_cache_without_plaintext(self):
    for name in VINFAST_DBCS:
      with self.subTest(dbc=name):
        if not _dbc_present(name):
          self.skipTest(f"{name} not present")
        reference = _load_reference_dbc(name)
        import opendbc
        import opendbc.can.dbc as dbc_mod

        with tempfile.TemporaryDirectory() as tmp:
          cache_path = os.path.join(tmp, f"{name}.dbc.cache")
          with open(cache_path, "wb") as f:
            f.write(DBC.cache_blob(reference))

          old = opendbc.DBC_PATH
          opendbc.DBC_PATH = tmp
          dbc_mod.DBC_PATH = tmp
          try:
            msg_name = next(iter(reference.msgs.values())).name
            parser = CANParser(name, [(msg_name, 20)], 0)
            self.assertEqual(parser.dbc.name, name)
          finally:
            opendbc.DBC_PATH = old
            dbc_mod.DBC_PATH = old

  def test_committed_cache_loads(self):
    for name in VINFAST_DBCS:
      with self.subTest(dbc=name):
        _, cache_path = _dbc_paths(name)
        if not os.path.isfile(cache_path):
          self.skipTest(f"committed {name}.dbc.cache not present")
        loaded = DBC(name)
        self.assertEqual(loaded.name, name)
        self.assertGreater(len(loaded.msgs), 0)
