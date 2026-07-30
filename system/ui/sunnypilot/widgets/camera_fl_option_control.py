"""Option +/- control backed by camera_fl_params JSON file (not Params)."""
from collections.abc import Callable

import pyray as rl

from openpilot.sunnypilot.modeld_v2.camera_fl_params import get_focal_lengths, set_one_focal_length
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.sunnypilot.widgets.option_control import LABEL_WIDTH, OptionControlSP
from openpilot.system.ui.widgets.list_view import ItemAction, ListItemSP


class CameraFlOptionControl(OptionControlSP):
  """Same UI as OptionControlSP; persists via camera_fl_params JSON file."""

  def __init__(self, which: str, min_value: int, max_value: int,
               value_change_step: int = 10, enabled: bool | Callable[[], bool] = True,
               label_width: int = LABEL_WIDTH,
               label_callback: Callable[[int], str] | None = None):
    assert which in ("ecam", "fcam")
    ItemAction.__init__(self, enabled=enabled)
    self.which = which
    self.params = None
    self.param_key = which
    self.min_value = min_value
    self.max_value = max_value
    self.value_change_step = value_change_step
    self._minus_enabled = True
    self._plus_enabled = True
    self.on_value_changed = None
    self.value_map = None
    self.label_width = label_width
    self.use_float_scaling = False
    self.label_callback = label_callback

    ecam, fcam = get_focal_lengths()
    raw = ecam if which == "ecam" else fcam
    self.current_value = max(min_value, min(max_value, int(round(raw))))

    self._font = gui_app.font(FontWeight.MEDIUM)
    self.minus_btn_rect = rl.Rectangle(0, 0, 0, 0)
    self.plus_btn_rect = rl.Rectangle(0, 0, 0, 0)

  def set_value(self, value: int):
    if not (self.min_value <= value <= self.max_value):
      return
    if value == self.current_value:
      return
    self.current_value = value
    set_one_focal_length(self.which, float(value))


def camera_fl_option_item(title, which: str, min_value: int, max_value: int, description,
                          value_change_step: int = 10, label_width: int = LABEL_WIDTH,
                          label_callback=None) -> ListItemSP:
  action = CameraFlOptionControl(which, min_value, max_value, value_change_step,
                                 True, label_width, label_callback)
  return ListItemSP(title=title, description=description, action_item=action)
