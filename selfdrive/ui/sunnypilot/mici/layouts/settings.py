"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.layouts.settings import settings as OP
from openpilot.selfdrive.ui.mici.layouts.settings.settings import SettingsBigButton
from openpilot.selfdrive.ui.mici.layouts.settings.device import DeviceLayoutMici
import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.button import BigCircleButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigDialog
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.sunnylink import SunnylinkLayoutMici
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.vehicle import VehicleLayoutMici, current_vehicle_text
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.label import UnifiedLabel

_ENABLE_OFFROAD = "enable\nalways\noffroad"
_EXIT_OFFROAD = "exit\nalways\noffroad"


class _OffroadCircleButton(BigCircleButton):
  """The stock icons have the English words baked in. Draw the translated label instead."""

  def __init__(self, label: str, red: bool):
    super().__init__(gui_app.texture("icons_mici/buttons/toggle_dot_enabled.png", 1, 1), red=red)
    self._label = UnifiedLabel(
      lambda: tr(label),
      font_size=26,
      font_weight=FontWeight.BOLD,
      text_color=rl.Color(255, 255, 255, 235),
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
      alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE,
      wrap_text=False,
      elide=False,
      line_height=0.92,
    )

  def _draw_content(self, btn_y: float):
    rect = rl.Rectangle(self._rect.x + 6, btn_y + 22, self._rect.width - 12, self._rect.height - 44)
    self._label.render(rect)

ICON_SIZE = 70
BIG_ICON_SIZE = 110


class SettingsLayoutSP(OP.SettingsLayout):
  def __init__(self):
    OP.SettingsLayout.__init__(self)

    device_panel = DeviceLayoutMici()
    self._scroller._items[2].set_click_callback(lambda: gui_app.push_widget(device_panel))

    self.icon_offroad_slider = gui_app.texture("icons_mici/settings/device/lkas.png", BIG_ICON_SIZE, BIG_ICON_SIZE)

    sunnylink_panel = SunnylinkLayoutMici(back_callback=gui_app.pop_widget)
    sunnylink_btn = SettingsBigButton(tr("sunnylink"), "", gui_app.texture("icons_mici/settings/developer/ssh.png", 55, 55))
    sunnylink_btn.set_click_callback(lambda: gui_app.push_widget(sunnylink_panel))

    models_panel = ModelsLayoutMici(back_callback=gui_app.pop_widget)
    models_btn = SettingsBigButton(tr("models"), "", gui_app.texture("../../sunnypilot/selfdrive/assets/offroad/icon_models.png", ICON_SIZE, ICON_SIZE))
    models_btn.set_click_callback(lambda: gui_app.push_widget(models_panel))

    vehicle_panel = VehicleLayoutMici(back_callback=gui_app.pop_widget)
    self._vehicle_btn = SettingsBigButton(tr("vehicle"), current_vehicle_text(),
                                          gui_app.texture("../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png", ICON_SIZE, ICON_SIZE))
    self._vehicle_btn.set_click_callback(lambda: gui_app.push_widget(vehicle_panel))

    # onroad: enable button sits at the front (left of toggles)
    self._enable_offroad_btn_onroad = _OffroadCircleButton(_ENABLE_OFFROAD, red=True)
    self._enable_offroad_btn_onroad.set_click_callback(lambda: self._handle_always_offroad(True))
    self._enable_offroad_btn_onroad.set_visible(lambda: ui_state.started and not ui_state.always_offroad)

    # offroad: enable button sits at the end (right of developer)
    self._enable_offroad_btn_offroad = _OffroadCircleButton(_ENABLE_OFFROAD, red=True)
    self._enable_offroad_btn_offroad.set_click_callback(lambda: self._handle_always_offroad(True))
    self._enable_offroad_btn_offroad.set_visible(lambda: not ui_state.started and not ui_state.always_offroad)

    self._disable_offroad_btn = _OffroadCircleButton(_EXIT_OFFROAD, red=False)
    self._disable_offroad_btn.set_click_callback(lambda: self._handle_always_offroad(False))
    self._disable_offroad_btn.set_visible(lambda: ui_state.always_offroad)

    items = self._scroller._items.copy()

    items.insert(1, sunnylink_btn)
    items.insert(2, models_btn)
    items.insert(3, self._vehicle_btn)

    # front slots (only one ever visible at a time): exit-always-offroad, then enable-onroad
    items.insert(0, self._enable_offroad_btn_onroad)
    items.insert(0, self._disable_offroad_btn)
    # end slot: enable-offroad (right of developer)
    items.append(self._enable_offroad_btn_offroad)

    self._scroller._items.clear()
    for item in items:
      self._scroller.add_widget(item)

  def _update_state(self):
    super()._update_state()
    self._vehicle_btn.set_value(current_vehicle_text())

  def _handle_always_offroad(self, enable: bool):

    def _set_offroad_status(status: bool):
      if not ui_state.engaged:
        ui_state.params.put_bool("OffroadMode", status)
        ui_state.always_offroad = status

    if not enable:
      dlg = BigConfirmationDialog(tr("slide to exit always offroad"), self.icon_offroad_slider, red=False,
                                  confirm_callback=lambda: _set_offroad_status(False))
    else:
      if ui_state.engaged:
        gui_app.push_widget(BigDialog(tr("disengage to enable always offroad"), "", ))
        return

      dlg = BigConfirmationDialog(tr("slide to force offroad"), self.icon_offroad_slider, red=True,
                                  confirm_callback=lambda: _set_offroad_status(True))
    gui_app.push_widget(dlg)
