"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

CameraOffset + per-device ecam/fcam focal-length overrides for non-Comma C3XL
clones (JSON via camera_fl_params, defaults 650 / 2700). CameraOffset is lateral
shear (meters). Tune FL via /data/qmpilot/camera_focal_length.json.
"""
import numpy as np

from openpilot.common.transformations.camera import DEVICE_CAMERAS


def intrinsics_with_fl(base_intrinsics: np.ndarray, fl: float) -> np.ndarray:
  """Return a copy of K with fx/fy replaced when fl > 0; otherwise return base."""
  if fl is None or fl <= 0:
    return base_intrinsics
  K = np.array(base_intrinsics, dtype=np.float64, copy=True)
  K[0, 0] = float(fl)
  K[1, 1] = float(fl)
  return K


class CameraOffsetHelper:
  def __init__(self):
    self.camera_offset = 0.0
    self.actual_camera_offset = 0.0
    self.ecam_fl = 0.0
    self.fcam_fl = 0.0

  @staticmethod
  def apply_camera_offset(model_transform, intrinsics, height, offset_param):
    cy = intrinsics[1, 2]
    shear = np.eye(3, dtype=np.float32)
    shear[0, 1] = offset_param / height
    shear[0, 2] = -offset_param / height * cy
    model_transform = (shear @ model_transform).astype(np.float32)
    return model_transform

  def set_offset(self, offset):
    self.camera_offset = offset

  def set_focal_lengths(self, ecam_fl: float, fcam_fl: float):
    self.ecam_fl = float(ecam_fl or 0.0)
    self.fcam_fl = float(fcam_fl or 0.0)

  def get_intrinsics(self, sm, main_wide_camera: bool):
    """Stock DEVICE_CAMERAS intrinsics with optional FL overrides applied."""
    dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
    ecam_K = intrinsics_with_fl(dc.ecam.intrinsics, self.ecam_fl)
    fcam_K = intrinsics_with_fl(dc.fcam.intrinsics, self.fcam_fl)
    return (ecam_K if main_wide_camera else fcam_K), ecam_K

  def update(self, model_transform_main, model_transform_extra, sm, main_wide_camera):
    self.actual_camera_offset = (0.9 * self.actual_camera_offset) + (0.1 * self.camera_offset)
    height = sm["liveCalibration"].height[0] if sm['liveCalibration'].height else 1.22

    intrinsics_main, intrinsics_extra = self.get_intrinsics(sm, main_wide_camera)
    model_transform_main = self.apply_camera_offset(model_transform_main, intrinsics_main, height, self.actual_camera_offset)
    model_transform_extra = self.apply_camera_offset(model_transform_extra, intrinsics_extra, height, self.actual_camera_offset)
    return model_transform_main, model_transform_extra
