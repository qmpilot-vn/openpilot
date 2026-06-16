import os
import tempfile
import unittest

from opendbc import DBC_PATH
from opendbc.can.dbc import DBC
from opendbc.can.parser import CANParser


VINFAST_DBC = "vinfast_vf8_chassis_can"
PLAIN_PATH = os.path.join(DBC_PATH, f"{VINFAST_DBC}.dbc")
CACHE_PATH = os.path.join(DBC_PATH, f"{VINFAST_DBC}.dbc.cache")


def _load_reference_dbc() -> DBC:
  if os.path.isfile(PLAIN_PATH):
    return DBC(VINFAST_DBC)
  if os.path.isfile(CACHE_PATH):
    return DBC(CACHE_PATH)
  raise unittest.SkipTest("VinFast DBC not present (need .dbc or .dbc.cache)")


@unittest.skipUnless(
  os.path.isfile(PLAIN_PATH) or os.path.isfile(CACHE_PATH),
  "VinFast DBC not present",
)
class TestVinFastDbcCache(unittest.TestCase):
  def test_cache_matches_reference(self):
    reference = _load_reference_dbc()
    with tempfile.TemporaryDirectory() as tmp:
      cache_path = os.path.join(tmp, f"{VINFAST_DBC}.dbc.cache")
      with open(cache_path, "wb") as f:
        f.write(DBC.cache_blob(reference))

      cached = DBC(cache_path)
      self.assertEqual(cached.name, reference.name)
      self.assertEqual(len(cached.msgs), len(reference.msgs))
      self.assertEqual(set(cached.msgs), set(reference.msgs))

  def test_parser_loads_from_cache_without_plaintext(self):
    reference = _load_reference_dbc()
    import opendbc
    import opendbc.can.dbc as dbc_mod

    with tempfile.TemporaryDirectory() as tmp:
      cache_path = os.path.join(tmp, f"{VINFAST_DBC}.dbc.cache")
      with open(cache_path, "wb") as f:
        f.write(DBC.cache_blob(reference))

      old = opendbc.DBC_PATH
      opendbc.DBC_PATH = tmp
      dbc_mod.DBC_PATH = tmp
      try:
        msg_name = next(iter(reference.msgs.values())).name
        parser = CANParser(VINFAST_DBC, [(msg_name, 20)], 0)
        self.assertEqual(parser.dbc.name, VINFAST_DBC)
      finally:
        opendbc.DBC_PATH = old
        dbc_mod.DBC_PATH = old

  def test_committed_cache_loads(self):
    if not os.path.isfile(CACHE_PATH):
      self.skipTest("committed .dbc.cache not present")
    loaded = DBC(VINFAST_DBC)
    self.assertEqual(loaded.name, VINFAST_DBC)
    self.assertGreater(len(loaded.msgs), 0)
