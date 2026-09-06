"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import hashlib
import os
import pickle
from pathlib import Path
import numpy as np

from typing import Optional

from cereal import custom
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.models.bundled_model import BUNDLED_BUNDLE, BUNDLED_MODEL
from openpilot.sunnypilot.models.constants import Meta, MetaTombRaider, MetaSimPose
from openpilot.system.hardware.hw import Paths

# v18 catalog requires an exact selector match
REQUIRED_JSON_VERSION = 16

CUSTOM_MODEL_PATH = Paths.model_root()
METADATA_PATH = Path(__file__).parent / '../models/supercombo_metadata.pkl'
ModelManager = custom.ModelManagerSP
_LAST_VALIDATED_RAW = None

PMV2_MIN_LAT_SMOOTH = 0.15


def get_lat_smooth_seconds(bundle: Optional[custom.ModelManagerSP.ModelBundle]) -> float:
  overrides = {override.key: override.value for override in bundle.overrides} if bundle else {}
  lat = float(overrides.get("lat", "0"))
  if bundle is not None and bundle.internalName == "PMV2" and lat < PMV2_MIN_LAT_SMOOTH:
    lat = PMV2_MIN_LAT_SMOOTH
  return lat


def ensure_pmv2_lat_override(overrides_data: dict[str, str], short_name: str) -> dict[str, str]:
  if short_name == "PMV2" and float(overrides_data.get("lat", "0")) < PMV2_MIN_LAT_SMOOTH:
    overrides_data = dict(overrides_data)
    overrides_data["lat"] = ".15"
  return overrides_data


def _compute_hash(file_path: str) -> str | None:
  from openpilot.common.file_chunker import open_file_chunked
  try:
    with open_file_chunked(file_path) as file:
      return hashlib.file_digest(file, "sha256").hexdigest().lower()
  except FileNotFoundError:
    return None


async def verify_file(file_path: str, expected_hash: str) -> bool:
  file_hash = _compute_hash(file_path)
  return file_hash == expected_hash.lower() if file_hash else False


def _verify_file(file_path: str, expected_hash: str) -> bool:
  file_hash = _compute_hash(file_path)
  return file_hash == expected_hash.lower() if file_hash else False


def is_bundle_version_compatible(bundle: dict) -> bool:
  """Bundle must match REQUIRED_JSON_VERSION (driving_models_v18.json)."""
  return bundle.get("minimumSelectorVersion", 0) == REQUIRED_JSON_VERSION


def _bundle_artifacts(bundle: custom.ModelManagerSP.ModelBundle) -> list[tuple[str, str]]:
  artifacts = []
  from sunnypilot.models.fetcher import get_artifact_chunks
  from openpilot.common.file_chunker import get_chunk_name
  for model in getattr(bundle, 'models', []) or []:
    for artifact in (getattr(model, 'artifact', None),):
      if artifact and getattr(artifact, 'fileName', None):
        chunks = get_artifact_chunks(artifact.fileName)
        if chunks:
          for i, chunk in enumerate(chunks):
            sha256 = chunk.get("sha256") if isinstance(chunk, dict) else None
            if sha256:
              artifacts.append((get_chunk_name(artifact.fileName, i, len(chunks)), sha256))
        elif getattr(artifact, 'downloadUri', None):
          sha256 = getattr(artifact.downloadUri, 'sha256', None)
          if sha256:
            artifacts.append((artifact.fileName, sha256))
  return artifacts


def _bundle_is_valid_locally(bundle: custom.ModelManagerSP.ModelBundle) -> bool:
  model_root = Paths.model_root()
  return all(_verify_file(os.path.join(model_root, file_name), expected_hash)
             for file_name, expected_hash in _bundle_artifacts(bundle))


def _bundle_needs_reset(active_bundle: custom.ModelManagerSP.ModelBundle, available_bundles: list[custom.ModelManagerSP.ModelBundle] | None) -> bool:
  if active_bundle is None:
    return False

  if available_bundles is not None:
    matching_bundle = None
    for bundle in available_bundles:
      if getattr(active_bundle, 'ref', None) and getattr(bundle, 'ref', None):
        if active_bundle.ref == bundle.ref:
          matching_bundle = bundle
          break
      elif getattr(active_bundle, 'internalName', None) == getattr(bundle, 'internalName', None):
        matching_bundle = bundle
        break

    if matching_bundle is None:
      return True
    if active_bundle.minimumSelectorVersion != matching_bundle.minimumSelectorVersion:
      return True

    active_runner = getattr(active_bundle, 'runner', None)
    matching_runner = getattr(matching_bundle, 'runner', None)
    if active_runner is not None and matching_runner is not None:
      if getattr(active_runner, 'raw', active_runner) != getattr(matching_runner, 'raw', matching_runner):
        return True
    if set(_bundle_artifacts(active_bundle)) != set(_bundle_artifacts(matching_bundle)):
      return True

  return not _bundle_is_valid_locally(active_bundle)


def validate_active_bundle(params: Params, available_bundles: list[custom.ModelManagerSP.ModelBundle] | None = None) -> None:
  global _LAST_VALIDATED_RAW

  raw_bundle = params.get("ModelManager_ActiveBundle")
  if not raw_bundle:
    return

  if raw_bundle == _LAST_VALIDATED_RAW:
    return

  active_bundle = get_active_bundle(params, raw_bundle_dict=raw_bundle, fallback=False)
  if active_bundle is None or _bundle_needs_reset(active_bundle, available_bundles):
    cloudlog.warning(f"Active model bundle invalid; falling back to bundled {BUNDLED_MODEL}")
    params.remove("ModelManager_ActiveBundle")
    # let the runner recompute from the bundled fallback instead of pinning stock, whose
    # in-tree pickles no longer match the vendored tinygrad
    params.remove("ModelRunnerTypeCache")
    _LAST_VALIDATED_RAW = None
  else:
    _LAST_VALIDATED_RAW = raw_bundle


def get_active_bundle(params: Params = None, raw_bundle_dict: dict | bytes | None = None,
                      fallback: bool = True) -> custom.ModelManagerSP.ModelBundle:
  """Gets the active model bundle from cache

  Falls back to BUNDLED_BUNDLE when nothing valid is selected. The stock driving pickles in-tree
  were compiled against an older tinygrad and no longer unpickle, so returning None here would
  route the runner to a modeld that cannot start. Pass fallback=False when the caller needs to
  distinguish "nothing selected" from "bundled default".
  """
  if params is None:
    params = Params()

  try:
    active_bundle_dict = raw_bundle_dict if raw_bundle_dict is not None else (params.get("ModelManager_ActiveBundle") or {})
    if isinstance(active_bundle_dict, dict) and active_bundle_dict and is_bundle_version_compatible(active_bundle_dict):
      return custom.ModelManagerSP.ModelBundle(**active_bundle_dict)
  except Exception:
    pass

  if not fallback:
    return None

  try:
    from openpilot.sunnypilot.models.bundled_model import ensure_bundled_model
    ensure_bundled_model()
  except Exception:
    cloudlog.exception("failed to seed bundled PMV2 into model_root")

  try:
    if not params.get("ModelManager_ActiveBundle"):
      params.put("ModelManager_ActiveBundle", BUNDLED_BUNDLE)
    return custom.ModelManagerSP.ModelBundle(**BUNDLED_BUNDLE)
  except Exception:
    cloudlog.exception("bundled fallback model bundle is malformed")
    return None


def get_active_model_runner(params: Params = None, force_check=False) -> custom.ModelManagerSP.Runner:
  if params is None:
    params = Params()

  cached_runner_type = params.get("ModelRunnerTypeCache")
  if cached_runner_type is not None and not force_check:
    if isinstance(cached_runner_type, str) and cached_runner_type.isdigit():
      return int(cached_runner_type)
    return cached_runner_type

  runner_type = custom.ModelManagerSP.Runner.stock

  if active_bundle := get_active_bundle(params):
    runner_type = active_bundle.runner.raw

  if cached_runner_type != runner_type:
    params.put("ModelRunnerTypeCache", int(runner_type))

  return runner_type


def _get_model():
  if bundle := get_active_bundle():
    drive_model = next(model for model in bundle.models if model.type == ModelManager.Model.Type.supercombo)
    return drive_model

  return None


def load_metadata():
  metadata_path = METADATA_PATH

  if model := _get_model():
    metadata_path = f"{CUSTOM_MODEL_PATH}/{model.metadata.fileName}"

  with open(metadata_path, 'rb') as f:
    return pickle.load(f)


def prepare_inputs(model_metadata) -> dict[str, np.ndarray]:
  inputs = {
    k: np.zeros(v, dtype=np.float32).flatten()
    for k, v in model_metadata['input_shapes'].items()
    if 'img' not in k
  }

  return inputs


def load_meta_constants(model_metadata):
  meta = Meta

  if 'sim_pose' in model_metadata['input_shapes'].keys():
    meta = MetaSimPose
  else:
    meta_slice = model_metadata['output_slices']['meta']
    meta_tf_slice = slice(5868, 5921, None)

    if (
            meta_slice.start == meta_tf_slice.start and
            meta_slice.stop == meta_tf_slice.stop and
            meta_slice.step == meta_tf_slice.step
    ):
      meta = MetaTombRaider

  return meta


def plan_x_idxs_helper(constants, plan, model_output) -> list[float]:
  LINE_T_IDXS = [np.nan] * constants.IDX_N
  LINE_T_IDXS[0] = 0.0
  plan_x = model_output['plan'][0, :, plan.POSITION][:, 0].tolist()
  for xidx in range(1, constants.IDX_N):
    tidx = 0
    while tidx < constants.IDX_N - 1 and plan_x[tidx + 1] < constants.X_IDXS[xidx]:
      tidx += 1
    if tidx == constants.IDX_N - 1:
      LINE_T_IDXS[xidx] = constants.T_IDXS[constants.IDX_N - 1]
      break
    current_x_val = plan_x[tidx]
    next_x_val = plan_x[tidx + 1]
    p = (constants.X_IDXS[xidx] - current_x_val) / (next_x_val - current_x_val) if abs(
      next_x_val - current_x_val) > 1e-9 else float('nan')
    LINE_T_IDXS[xidx] = p * constants.T_IDXS[tidx + 1] + (1 - p) * constants.T_IDXS[tidx]
  return LINE_T_IDXS
