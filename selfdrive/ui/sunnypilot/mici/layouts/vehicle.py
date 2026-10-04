"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
from collections.abc import Callable

from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.platform_selector import CAR_LIST_JSON_OUT
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


def current_vehicle_text() -> str:
  bundle = ui_state.params.get("CarPlatformBundle") or {}
  if bundle:
    return str(bundle.get("model") or bundle.get("name", "")).lower()
  if ui_state.CP is not None and ui_state.CP.carFingerprint != "MOCK":
    return str(ui_state.CP.carFingerprint).lower()
  return tr("not set")


class VehicleLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)

    with open(CAR_LIST_JSON_OUT) as f:
      platforms = json.load(f)
    self._vinfast = {name: data for name, data in sorted(platforms.items()) if data.get("brand") == "vinfast"}

    self._model_btns: dict[str, BigButton] = {}
    for name, data in self._vinfast.items():
      btn = BigButton(str(data.get("model", name)).lower(), name.lower())
      btn.set_click_callback(lambda n=name: self._select(n))
      self._model_btns[name] = btn

    self._auto_btn = BigButton(tr("auto detect"), tr("use fingerprint"))
    self._auto_btn.set_click_callback(self._select_auto)

    self._scroller.add_widgets([*self._model_btns.values(), self._auto_btn])

  def _select(self, name: str):
    if ui_state.is_offroad():
      ui_state.params.put("CarPlatformBundle", {**self._vinfast[name], "name": name})

  def _select_auto(self):
    if ui_state.is_offroad():
      ui_state.params.remove("CarPlatformBundle")

  def _update_state(self):
    super()._update_state()
    offroad = ui_state.is_offroad()
    selected = (ui_state.params.get("CarPlatformBundle") or {}).get("name")
    for name, btn in self._model_btns.items():
      btn.set_enabled(offroad)
      btn.set_value(tr("selected") if name == selected else (name.lower() if offroad else tr("offroad only")))
    self._auto_btn.set_enabled(offroad)
    self._auto_btn.set_value(tr("selected") if selected is None else (tr("use fingerprint") if offroad else tr("offroad only")))
