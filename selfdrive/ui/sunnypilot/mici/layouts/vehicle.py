"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
from collections.abc import Callable
from functools import cache

from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigToggle, GreyBigButton
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.vinfast import _reengage_enabled, _set_reengage
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.platform_selector import CAR_LIST_JSON_OUT
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller

MODEL_ORDER = ("VF6", "VF7", "VF7 Plus", "VF8", "VF8 Plus", "VF9")


@cache
def vinfast_platforms() -> dict[str, dict]:
  with open(CAR_LIST_JSON_OUT) as f:
    platforms = json.load(f)
  vinfast = {name: data for name, data in platforms.items() if data.get("brand") == "vinfast"}

  def order(item):
    model = str(item[1].get("model", item[0]))
    return (MODEL_ORDER.index(model) if model in MODEL_ORDER else len(MODEL_ORDER), model)

  return dict(sorted(vinfast.items(), key=order))


def model_label(data: dict) -> str:
  return str(data.get("model", "")).lower()


def years_label(data: dict) -> str:
  years = [str(y) for y in data.get("year", []) if str(y).isdigit()]
  if not years:
    return ""
  if len(years) == 1:
    return years[0]
  return f"{years[0]}–{years[-1][-2:]}"


def _model_for_fingerprint(fingerprint: str) -> str:
  for data in vinfast_platforms().values():
    if data.get("platform") == fingerprint:
      return model_label(data)
  return fingerprint.lower().removeprefix("vinfast_").replace("_", " ")


def current_vehicle_text() -> str:
  bundle = ui_state.params.get("CarPlatformBundle") or {}
  if bundle:
    return model_label(bundle) or str(bundle.get("name", "")).lower()
  if ui_state.CP is not None and ui_state.CP.carFingerprint != "MOCK":
    return f"{_model_for_fingerprint(str(ui_state.CP.carFingerprint))} {tr('(auto)')}"
  return tr("not set")


class VinFastPicker(NavScroller):
  """Opened from the vehicle page. Scroll up and down to pick a model."""

  def __init__(self):
    super().__init__(horizontal=False, spacing=12, pad=8)
    self._vinfast = vinfast_platforms()

    self._auto_btn = BigButton(tr("auto detect"), tr("use fingerprint"))
    self._auto_btn.set_click_callback(self._select_auto)

    self._model_btns: dict[str, BigButton] = {}
    for name, data in self._vinfast.items():
      btn = BigButton(model_label(data), years_label(data))
      btn.set_click_callback(lambda n=name: self._select(n))
      self._model_btns[name] = btn

    self._scroller.add_widgets([self._auto_btn, *self._model_btns.values()])

  def _select(self, name: str):
    if ui_state.is_offroad():
      ui_state.params.put("CarPlatformBundle", {**self._vinfast[name], "name": name})
      self.dismiss()

  def _select_auto(self):
    if ui_state.is_offroad():
      ui_state.params.remove("CarPlatformBundle")
      self.dismiss()

  def _update_state(self):
    super()._update_state()
    offroad = ui_state.is_offroad()
    selected = (ui_state.params.get("CarPlatformBundle") or {}).get("name")

    auto_value = f"✓ {tr('selected')}" if selected is None else tr("use fingerprint")
    if self._auto_btn.get_value() != auto_value:
      self._auto_btn.set_value(auto_value)
    self._auto_btn.set_enabled(offroad)

    for name, btn in self._model_btns.items():
      years = years_label(self._vinfast[name])
      value = f"✓ {years}" if name == selected else years
      if btn.get_value() != value:
        btn.set_value(value)
      btn.set_enabled(offroad)


class VehicleLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)

    self._reengage_toggle = BigToggle(tr("auto reengage"), initial_state=_reengage_enabled(), toggle_callback=_set_reengage)
    self._reengage_toggle.set_enabled(lambda: not ui_state.engaged)

    self._status_card = GreyBigButton(tr("car model"), "")

    self._picker = VinFastPicker()
    self._vinfast_btn = BigButton(tr("vinfast"), "")
    self._vinfast_btn.set_click_callback(lambda: gui_app.push_widget(self._picker))

    self._scroller.add_widgets([self._reengage_toggle, self._status_card, self._vinfast_btn])

  def show_event(self):
    super().show_event()
    self._reengage_toggle.set_checked(_reengage_enabled())

  def _status_text(self, selected: str | None, offroad: bool) -> str:
    platforms = vinfast_platforms()
    if selected in platforms:
      text = f"{model_label(platforms[selected])} • {tr('fixed')}"
    elif ui_state.CP is not None and ui_state.CP.carFingerprint != "MOCK":
      text = f"{_model_for_fingerprint(str(ui_state.CP.carFingerprint))} • {tr('auto detected')}"
    else:
      text = tr("auto detect")
    return text if offroad else f"{text}\n{tr('change when offroad')}"

  def _update_state(self):
    super()._update_state()
    offroad = ui_state.is_offroad()
    selected = (ui_state.params.get("CarPlatformBundle") or {}).get("name")

    self._reengage_toggle.set_value(tr("on") if self._reengage_toggle._checked else tr("off"))

    status = self._status_text(selected, offroad)
    if self._status_card.get_value() != status:
      self._status_card.set_value(status)

    choice = current_vehicle_text()
    if self._vinfast_btn.get_value() != choice:
      self._vinfast_btn.set_value(choice)
