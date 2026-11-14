"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable
import pyray as rl

from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, option_item_sp, LineSeparatorSP
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class LaneCenteringSettingsLayout(Widget):
  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=False, spacing=0)

  def _initialize_items(self):
    self._lane_centering_toggle = toggle_item_sp(
      param="LaneCentering",
      title=lambda: tr("Lane Centering"),
      description=lambda: tr("Bias the model command toward the detected lane center. Requires two confident lane lines and stays inside normal curvature and jerk limits."),
    )
    self._lane_center_offset = option_item_sp(
      param="LaneCenterOffset",
      title=lambda: tr("Lane Center Offset"),
      description=lambda: tr("Shift the lane target left or right. The controller reduces this offset automatically when the detected lane is narrow."),
      min_value=-30,
      max_value=30,
      value_change_step=1,
      use_float_scaling=True,
      label_callback=lambda offset: f"{offset / 100.0:+.2f} m",
    )
    self._lane_centering_e2e = option_item_sp(
      param="LaneCenteringE2EAuthority",
      title=lambda: tr("E2E Override Strength"),
      description=lambda: tr("How strongly a confident end-to-end path can override lane centering when it deliberately leaves the lane target. 1.0 gives the model full authority; 0.0 disables break-in."),
      min_value=0,
      max_value=100,
      value_change_step=5,
      use_float_scaling=True,
      label_callback=lambda authority: f"{authority / 100.0:.2f}",
    )

    return [
      self._lane_centering_toggle,
      LineSeparatorSP(40),
      self._lane_center_offset,
      self._lane_centering_e2e,
    ]

  def _update_state(self):
    super()._update_state()
    enabled = self._lane_centering_toggle.action_item.get_state()
    self._lane_center_offset.set_visible(enabled)
    self._lane_centering_e2e.set_visible(enabled)

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()
    content_rect = rl.Rectangle(rect.x, rect.y + self._back_button.rect.height + 40, rect.width, rect.height - self._back_button.rect.height - 40)
    self._scroller.render(content_rect)

  def show_event(self):
    self._scroller.show_event()
