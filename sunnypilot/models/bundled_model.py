"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import hashlib
import os
import shutil
from pathlib import Path

# The in-tree stock driving pickles were compiled against the tinygrad vendored before
# 741af84; they cannot be unpickled by the tinygrad vendored now. Until they are rebuilt,
# resolving to the stock runner is a guaranteed modeld crash, so every path that used to
# fall through to stock resolves here instead.
#
# Keep this in sync with driving_models_v18.json: minimumSelectorVersion must equal
# REQUIRED_JSON_VERSION in helpers.py, and the artifact sha256 must match the catalog.
# The chunk files live in sunnypilot/models/bundled/ so a fresh install can drive
# without waiting for ModelManager to fetch GitLab.

BUNDLED_MODEL = "PMV2"
BUNDLED_PKL_NAME = "driving_pmv2_tinygrad.pkl"
BUNDLED_PKL_DIR = Path(__file__).resolve().parent / "bundled"
BUNDLED_PKL_SHA256 = "2a752c929d4fec2f3757d3c3d82d7507ed2aa313621109ca3f2c03108a84a873"

BUNDLED_CHUNKS = [
  {"file_name": "driving_pmv2_tinygrad.pkl.chunk01of04",
   "sha256": "29dbf1b2194042d72febaa566b131888153507a059ade783228da6b8b9e3329a"},
  {"file_name": "driving_pmv2_tinygrad.pkl.chunk02of04",
   "sha256": "88a8c6738def8873879366a519de6d50136de384880abd9781a301078c892859"},
  {"file_name": "driving_pmv2_tinygrad.pkl.chunk03of04",
   "sha256": "b42d073dd733059624fc38171c72fd1435214912dda5e2827561683e87483ef0"},
  {"file_name": "driving_pmv2_tinygrad.pkl.chunk04of04",
   "sha256": "3fb153d083c4eabb16edc57def2ecb3926164b1a3446a32cab5e505d878b3b83"},
]

BUNDLED_BUNDLE = {
  "index": 59,
  "internalName": "PMV2",
  "displayName": "Pop Model v2 (March 24, 2026)",
  "models": [
    {
      "type": "supercombo",
      "artifact": {
        "fileName": BUNDLED_PKL_NAME,
        "downloadUri": {
          "uri": "https://gitlab.com/sunnypilot/public/docs.sunnypilot.ai8/-/raw/main/models/recompiled18/"
                 "model-Pop%20Model%20v2%20%28March%2024%2C%202026%29-115/driving_pmv2_tinygrad.pkl",
          "sha256": BUNDLED_PKL_SHA256,
        },
      },
    },
  ],
  "status": "downloaded",
  "generation": 12,
  "environment": "development",
  "runner": "tinygrad",
  "is20hz": True,
  "ref": "62bf6fb072880905a4c490f0f4f4a6b3c23346ec",
  "minimumSelectorVersion": 16,
  "overrides": [
    {"key": "folder", "value": "2026 World Models"},
    {"key": "lat", "value": ".15"},
    {"key": "long", "value": ".3"},
  ],
}


def bundled_pkl_path() -> str:
  return str(BUNDLED_PKL_DIR / BUNDLED_PKL_NAME)


def _sha256_file(path: str):
  try:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
      for block in iter(lambda: f.read(1024 * 1024), b""):
        digest.update(block)
    return digest.hexdigest()
  except FileNotFoundError:
    return None


def bundled_pkl_ready(directory=None) -> bool:
  root = Path(directory) if directory is not None else BUNDLED_PKL_DIR
  if not root.is_dir():
    return False
  for chunk in BUNDLED_CHUNKS:
    got = _sha256_file(str(root / chunk["file_name"]))
    if got != chunk["sha256"]:
      return False
  manifest = root / f"{BUNDLED_PKL_NAME}.chunkmanifest"
  return manifest.is_file() and manifest.read_text().strip() == str(len(BUNDLED_CHUNKS))


def ensure_bundled_model(destination=None):
  """Copy the shipped PMV2 chunks into model_root so ModelManager and modeld agree.

  Returns the pkl path that should be loaded (model_root if seeded, else in-tree).
  """
  if not bundled_pkl_ready(BUNDLED_PKL_DIR):
    return None

  if destination is None:
    from openpilot.system.hardware.hw import Paths
    destination = Paths.model_root()

  os.makedirs(destination, exist_ok=True)
  if not bundled_pkl_ready(destination):
    for chunk in BUNDLED_CHUNKS:
      shutil.copy2(BUNDLED_PKL_DIR / chunk["file_name"], os.path.join(destination, chunk["file_name"]))
    manifest_name = f"{BUNDLED_PKL_NAME}.chunkmanifest"
    shutil.copy2(BUNDLED_PKL_DIR / manifest_name, os.path.join(destination, manifest_name))

  dest_pkl = os.path.join(destination, BUNDLED_PKL_NAME)
  if bundled_pkl_ready(destination):
    return dest_pkl
  return bundled_pkl_path()
