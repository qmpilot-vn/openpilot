"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.sunnypilot.modeld_v2.camera_offset_helper import CameraOffsetHelper, intrinsics_with_fl


class MockStruct:
  def __init__(self, **kwargs):
    for k, v in kwargs.items():
      setattr(self, k, v)

  def __getitem__(self, item):
    return getattr(self, item)


class TestCameraOffset:
  def setup_method(self):
    self.camera_offset = CameraOffsetHelper()
    self.dc = DEVICE_CAMERAS[('mici', 'os04c10')]
    self.dc_tici = DEVICE_CAMERAS[('tici', 'ox03c10')]

  def test_intrinsics_with_fl_noop(self):
    base = self.dc_tici.ecam.intrinsics
    np.testing.assert_array_equal(intrinsics_with_fl(base, 0.0), base)
    np.testing.assert_array_equal(intrinsics_with_fl(base, -1.0), base)
    assert intrinsics_with_fl(base, 0.0) is base

  def test_intrinsics_with_fl_replaces_fx_fy(self):
    base = self.dc_tici.ecam.intrinsics
    fl = 660.0
    K = intrinsics_with_fl(base, fl)
    assert K[0, 0] == fl
    assert K[1, 1] == fl
    assert K[0, 2] == base[0, 2]
    assert K[1, 2] == base[1, 2]
    # base unchanged
    assert base[0, 0] != fl

  def test_get_intrinsics_respects_fl_params(self):
    sm = MockStruct(
      deviceState=MockStruct(deviceType='tici'),
      roadCameraState=MockStruct(sensor='ox03c10'),
      liveCalibration=MockStruct(rpyCalib=[0.0, 0.0, 0.0], height=[1.22])
    )
    self.camera_offset.set_focal_lengths(660.0, 2800.0)
    main_K, extra_K = self.camera_offset.get_intrinsics(sm, main_wide_camera=False)
    assert main_K[0, 0] == 2800.0
    assert extra_K[0, 0] == 660.0
    main_w, extra_w = self.camera_offset.get_intrinsics(sm, main_wide_camera=True)
    assert main_w[0, 0] == 660.0
    assert extra_w[0, 0] == 660.0

  def test_smoothing(self):
    self.camera_offset.set_offset(0.2)

    sm = MockStruct(
      deviceState=MockStruct(deviceType='mici'),
      roadCameraState=MockStruct(sensor='os04c10'),
      liveCalibration=MockStruct(rpyCalib=[0.0, 0.0, 0.0], height=[1.22])
    )

    intrinsics_main = self.dc.fcam.intrinsics
    intrinsics_extra = self.dc.ecam.intrinsics
    device_from_calib_euler = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    main_transform = get_warp_matrix(device_from_calib_euler, intrinsics_main, False).astype(np.float32)
    extra_transform = get_warp_matrix(device_from_calib_euler, intrinsics_extra, True).astype(np.float32)

    self.camera_offset.update(main_transform, extra_transform, sm, False)
    np.testing.assert_almost_equal(self.camera_offset.actual_camera_offset, 0.02)
    self.camera_offset.update(main_transform, extra_transform, sm, False)
    np.testing.assert_almost_equal(self.camera_offset.actual_camera_offset, 0.038)

  def test_camera_offset_(self):
    intrinsics = self.dc.fcam.intrinsics
    transform = np.eye(3, dtype=np.float32)
    height = 1.22
    offset = 0.1

    cy = intrinsics[1, 2]
    expected_shear = np.eye(3, dtype=np.float32)
    expected_shear[0, 1] = offset / height
    expected_shear[0, 2] = -offset / height * cy

    result = CameraOffsetHelper.apply_camera_offset(transform, intrinsics, height, offset)
    np.testing.assert_array_almost_equal(result, expected_shear)

  def test_update(self):
    sm = MockStruct(
      deviceState=MockStruct(deviceType='mici'),
      roadCameraState=MockStruct(sensor='os04c10'),
      liveCalibration=MockStruct(rpyCalib=[0.0, 0.0, 0.0], height=[1.22])
    )
    intrinsics_main = self.dc.fcam.intrinsics
    intrinsics_extra = self.dc.ecam.intrinsics
    device_from_calib_euler = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    main_transform = get_warp_matrix(device_from_calib_euler, intrinsics_main, False).astype(np.float32)
    extra_transform = get_warp_matrix(device_from_calib_euler, intrinsics_extra, True).astype(np.float32)

    self.camera_offset.set_offset(0.0)  # test default offset doesn't change transformation
    main_out, extra_out = self.camera_offset.update(main_transform, extra_transform, sm, False)
    np.testing.assert_array_equal(main_out, main_transform)
    np.testing.assert_array_equal(extra_out, extra_transform)

    self.camera_offset.set_offset(0.2)  # test valid offset changes transformation
    main_out, extra_out = self.camera_offset.update(main_transform, extra_transform, sm, False)
    assert not np.array_equal(main_out, main_transform)
    assert not np.array_equal(extra_out, extra_transform)
    assert main_out[0, 1] != 0.0
    assert main_out[0, 2] != 0.0

  def test_update_uses_fl_override_for_shear_cy(self):
    """CameraOffset shear cy must come from FL-overridden intrinsics."""
    sm = MockStruct(
      deviceState=MockStruct(deviceType='tici'),
      roadCameraState=MockStruct(sensor='ox03c10'),
      liveCalibration=MockStruct(rpyCalib=[0.0, 0.0, 0.0], height=[1.22])
    )
    fl = 660.0
    self.camera_offset.set_focal_lengths(fl, 0.0)
    self.camera_offset.set_offset(0.2)
    # force actual offset to target for a deterministic shear
    self.camera_offset.actual_camera_offset = 0.0

    device_from_calib_euler = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    ecam_K = intrinsics_with_fl(self.dc_tici.ecam.intrinsics, fl)
    main_transform = get_warp_matrix(device_from_calib_euler, ecam_K, False).astype(np.float32)
    extra_transform = get_warp_matrix(device_from_calib_euler, ecam_K, True).astype(np.float32)

    main_out, _ = self.camera_offset.update(main_transform, extra_transform, sm, True)
    # one smoothing step: actual = 0.1 * 0.2
    actual = 0.02
    height = 1.22
    cy = ecam_K[1, 2]
    expected_shear = np.eye(3, dtype=np.float32)
    expected_shear[0, 1] = actual / height
    expected_shear[0, 2] = -actual / height * cy
    np.testing.assert_array_almost_equal(main_out, (expected_shear @ main_transform).astype(np.float32))
