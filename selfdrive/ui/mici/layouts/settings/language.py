from collections.abc import Callable

from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import UNIFONT_LANGUAGES, multilang, tr
from openpilot.system.ui.widgets.scroller import NavScroller


def current_language_name() -> str:
  return multilang.codes.get(multilang.language, multilang.language)


def set_language_font(label, code: str) -> None:
  # language names outside Latin script only exist in the unifont atlas
  label.set_font_weight(FontWeight.UNIFONT if code in UNIFONT_LANGUAGES else FontWeight.BOLD)


class LanguageButton(BigButton):
  def __init__(self, name: str, code: str):
    super().__init__(name)
    self.code = code
    set_language_font(self._label, code)

  def refresh(self):
    self.set_value(tr("selected") if self.code == multilang.language else "")


class LanguageLayoutMici(NavScroller):
  def __init__(self, on_change: Callable[[], None] | None = None):
    super().__init__()
    self._on_change = on_change
    self._buttons = [LanguageButton(name, code) for name, code in multilang.languages.items()]
    for btn in self._buttons:
      btn.set_click_callback(lambda code=btn.code: self._select(code))
    self._scroller.add_widgets(self._buttons)

  def show_event(self):
    super().show_event()
    for btn in self._buttons:
      btn.refresh()

  def _select(self, code: str):
    if code == multilang.language:
      self.dismiss()
      return

    multilang.change_language(code)
    if self._on_change:
      self._on_change()
    if ui_state.started:
      self.dismiss()
    else:
      # most labels are translated when built; manager restarts the ui with the new language
      gui_app.request_close()
