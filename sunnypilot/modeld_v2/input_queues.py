#!/usr/bin/env python3
"""Input queues for combined tinygrad JITs. Extracted from compile_modeld.py
so modeld can load without stock compile_modeld / NV12Frame."""
from functools import partial  # noqa: F401
import numpy as np

from tinygrad.device import Device
from tinygrad.tensor import Tensor


def _detect_desire_key(shapes: dict) -> str | None:
  return next((key for key in shapes if key.startswith('desire')), None)


def _detect_vision_keys(shapes: dict) -> tuple[str | None, str | None]:
  img_keys = sorted(key for key in shapes if 'img' in key)
  return (
    next((key for key in img_keys if 'big' not in key), None),
    next((key for key in img_keys if 'big' in key), None)
  )


def derive_frame_skip(vision_input_shapes: dict, policy_input_shapes: dict) -> int:
  features_buffer = policy_input_shapes.get('features_buffer')
  return 1 if not features_buffer or features_buffer[1] >= 99 else 4


def get_policy_npy_shapes(input_shapes: dict, is_supercombo: bool = False) -> tuple[dict, list[int]]:
  desire_key = _detect_desire_key(input_shapes)
  shapes = {}
  if desire_key:
    shapes['desire'] = (input_shapes[desire_key][2],)

  if is_supercombo and 'features_buffer' in input_shapes:
    fb = input_shapes['features_buffer']
    shapes['prev_feat'] = (fb[0], fb[2])

  for key, shape in input_shapes.items():
    if key not in (desire_key, 'features_buffer') and 'img' not in key:
      shapes[key] = tuple(shape)

  sizes = [int(np.prod(size)) for size in shapes.values()]
  return shapes, sizes


def generate_queues_and_npy(input_shapes: dict, frame_skip: int, device: str = Device.DEFAULT,
                            is_supercombo: bool = False, use_packed: bool = True) -> tuple[dict, dict]:
  road_key, _ = _detect_vision_keys(input_shapes)
  if not road_key:
    raise ValueError("Vision road key missing from input shapes.")

  img_shape = input_shapes[road_key]
  n_frames = img_shape[1] // 6
  img_buf_shape = (frame_skip * (n_frames - 1) + 1, 6, img_shape[2], img_shape[3])

  desire_key = _detect_desire_key(input_shapes)
  if not desire_key:
    raise ValueError("Desire key missing from input shapes.")

  desire_shape = input_shapes[desire_key]
  features_buffer = input_shapes.get('features_buffer')

  if use_packed:  # remove packed detection block after all models are recompiled
    npy_arrays = {
      'tfm': np.zeros((3, 3), dtype=np.float32),
      'big_tfm': np.zeros((3, 3), dtype=np.float32)
    }

    shapes, sizes = get_policy_npy_shapes(input_shapes, is_supercombo=is_supercombo)
    packed_npy_inputs = np.zeros(sum(sizes), dtype=np.float32)

    split_indices = np.cumsum(sizes[:-1]) if len(sizes) > 1 else []
    split_views = np.split(packed_npy_inputs, split_indices) if len(sizes) > 0 else []
    for (k, s), v in zip(shapes.items(), split_views, strict=True):
      npy_arrays[k] = v.reshape(s)

    queues = {
      'img_q': Tensor(np.zeros(img_buf_shape, dtype=np.uint8), device=device).contiguous().realize(),
      'big_img_q': Tensor(np.zeros(img_buf_shape, dtype=np.uint8), device=device).contiguous().realize(),
      'desire_q': Tensor(np.zeros((frame_skip * desire_shape[1], desire_shape[0], desire_shape[2]),
                    dtype=np.float32), device=device).contiguous().realize(),
      'packed_npy_inputs': Tensor(packed_npy_inputs, device='NPY').realize(),
    }

    if features_buffer:
      queues['feat_q'] = Tensor(np.zeros((frame_skip * (features_buffer[1] - 1) + 1, features_buffer[0], features_buffer[2]),
                         dtype=np.float32), device=device).contiguous().realize()

    queues.update({key: Tensor(value, device='NPY').realize() for key, value in npy_arrays.items() if key in ('tfm', 'big_tfm')})
  else:
    npy_arrays = {
      'desire': np.zeros(desire_shape[2], dtype=np.float32),
      'tfm': np.zeros((3, 3), dtype=np.float32),
      'big_tfm': np.zeros((3, 3), dtype=np.float32)
    }

    for key, shape in input_shapes.items():
      if key not in npy_arrays and 'img' not in key and key not in ('features_buffer', desire_key):
        npy_arrays[key] = np.zeros(shape, dtype=np.float32)

    queues = {
      'img_q': Tensor(np.zeros(img_buf_shape, dtype=np.uint8), device=device).contiguous().realize(),
      'big_img_q': Tensor(np.zeros(img_buf_shape, dtype=np.uint8), device=device).contiguous().realize(),
      'desire_q': Tensor(np.zeros((frame_skip * desire_shape[1], desire_shape[0], desire_shape[2]),
                    dtype=np.float32), device=device).contiguous().realize()
    }

    if features_buffer:
      queues['feat_q'] = Tensor(np.zeros((frame_skip * (features_buffer[1] - 1) + 1, features_buffer[0], features_buffer[2]),
                         dtype=np.float32), device=device).contiguous().realize()

    queues.update({key: Tensor(value, device='NPY').realize() for key, value in npy_arrays.items()})

  return queues, npy_arrays


def make_split_input_queues(vision_input_shapes: dict, policy_input_shapes: dict,
                            frame_skip: int, device: str = Device.DEFAULT, use_packed: bool = True) -> tuple[dict, dict]:
  return generate_queues_and_npy({**vision_input_shapes, **policy_input_shapes}, frame_skip, device, is_supercombo=False, use_packed=use_packed)


def make_supercombo_input_queues(input_shapes: dict, frame_skip: int,
                                 device: str = Device.DEFAULT, use_packed: bool = True) -> tuple[dict, dict]:
  return generate_queues_and_npy(input_shapes, frame_skip, device, is_supercombo=True, use_packed=use_packed)
