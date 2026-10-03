"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from pathlib import Path

from openpilot.common.params import Params
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp

PARAM = "VinFastReengage"
PARAM_PATH = Path("/data/params/d") / PARAM
TITLE = tr_noop("VinFast - Auto Reengage")
DESC = tr_noop(
  "Automatically resume after a full stop in traffic when the car ahead starts moving. "
  "Turn this off if you want to press Resume / ACC yourself. The car will stay stopped "
  "at a light until you resume."
)


def _reengage_enabled() -> bool:
  """Default ON. Avoid Params.get_bool — a missing file reads as False."""
  try:
    val = Params().get(PARAM, return_default=True)
    if val is None:
      return True
    if isinstance(val, bool):
      return val
    return str(val).strip().lower() in ("1", "true")
  except Exception:
    try:
      return PARAM_PATH.read_text(encoding="utf-8").strip().lower() in ("1", "true")
    except OSError:
      return True


def _set_reengage(on: bool) -> None:
  try:
    Params().put_bool(PARAM, on)
    return
  except Exception:
    pass
  try:
    PARAM_PATH.parent.mkdir(parents=True, exist_ok=True)
    PARAM_PATH.write_text("1" if on else "0", encoding="utf-8")
  except OSError:
    pass


class VinFastSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.reengage_toggle = toggle_item_sp(
      lambda: tr(TITLE),
      description=lambda: tr(DESC),
      initial_state=_reengage_enabled(),
      callback=_set_reengage,
      enabled=lambda: not ui_state.engaged,
    )
    self.items = [self.reengage_toggle]

  def update_settings(self):
    self.reengage_toggle.action_item.set_enabled(not ui_state.engaged)
