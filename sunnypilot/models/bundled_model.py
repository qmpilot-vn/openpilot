"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The in-tree stock driving pickles were compiled against the tinygrad vendored before
# 741af84; they cannot be unpickled by the tinygrad vendored now. Until they are rebuilt,
# resolving to the stock runner is a guaranteed modeld crash, so every path that used to
# fall through to stock resolves here instead.
#
# Keep this in sync with driving_models_v18.json: minimumSelectorVersion must equal
# REQUIRED_JSON_VERSION in helpers.py, and the artifact sha256 must match the catalog.

BUNDLED_MODEL = "PMV2"

BUNDLED_BUNDLE = {
  "index": 59,
  "internalName": "PMV2",
  "displayName": "Pop Model v2 (March 24, 2026)",
  "models": [
    {
      "type": "supercombo",
      "artifact": {
        "fileName": "driving_pmv2_tinygrad.pkl",
        "downloadUri": {
          "uri": "https://gitlab.com/sunnypilot/public/docs.sunnypilot.ai8/-/raw/main/models/recompiled18/"
                 "model-Pop%20Model%20v2%20%28March%2024%2C%202026%29-115/driving_pmv2_tinygrad.pkl",
          "sha256": "2a752c929d4fec2f3757d3c3d82d7507ed2aa313621109ca3f2c03108a84a873",
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
