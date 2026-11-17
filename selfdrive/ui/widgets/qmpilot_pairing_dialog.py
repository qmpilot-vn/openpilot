import time

import numpy as np
import pyray as rl
import qrcode
from cereal import log

from openpilot.common.api.qmpilot_pair import PairWorker, QmpilotPairClient, save_pair
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wrap_text import wrap_text
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import IconButton

NetworkType = log.DeviceState.NetworkType


def _dongle_id(params: Params) -> str:
  raw = params.get("DongleId")
  if isinstance(raw, bytes):
    raw = raw.decode()
  return (raw or "").strip()


def _on_wifi() -> bool:
  try:
    return ui_state.sm["deviceState"].networkType == NetworkType.wifi
  except Exception:
    return False


class QmpilotPairingDialog(Widget):
  """QR pair screen for qmpilot-connect. The API key is never drawn."""

  def __init__(self):
    super().__init__()
    self.params = Params()
    self.dongle_id = _dongle_id(self.params)
    self.qr_texture: rl.Texture | None = None
    self._drawn_url = ""
    self._saved = False
    self._close_at = 0.0
    self._worker: PairWorker | None = None
    self._close_btn = IconButton(gui_app.texture("icons/close.png", 80, 80))
    self._close_btn.set_click_callback(self._close)

  def show_event(self):
    super().show_event()
    if self._worker is not None or not self.dongle_id:
      return
    client = QmpilotPairClient(self.dongle_id)
    self._worker = PairWorker(client, _on_wifi)
    self._worker.start()

  def hide_event(self):
    if self._worker is not None:
      self._worker.stop()
      self._worker = None
    super().hide_event()

  def _close(self):
    gui_app.pop_widget()

  def _update_state(self):
    if self._saved:
      if self._close_at and time.monotonic() >= self._close_at:
        gui_app.pop_widget()
      return
    if self._worker is None:
      return
    url, message, api_key, api_host = self._worker.snapshot()
    if message == "paired" and isinstance(api_key, str) and api_host:
      try:
        save_pair(self.params, api_host, api_key)
      except Exception:
        cloudlog.exception("qmpilot pair save failed")
        return
      self._saved = True
      self._close_at = time.monotonic() + 1.2
      return
    if url and url != self._drawn_url:
      self._draw_qr(url)

  def _draw_qr(self, url: str) -> None:
    if "api_key" in url or "QmpilotApiKey" in url:
      cloudlog.error("refusing a pair QR that contains a key")
      return
    try:
      qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_L, box_size=10, border=4)
      qr.add_data(url)
      qr.make(fit=True)
      pil_img = qr.make_image(fill_color="black", back_color="white").convert("RGBA")
      img_array = np.array(pil_img, dtype=np.uint8)
      if self.qr_texture and self.qr_texture.id != 0:
        rl.unload_texture(self.qr_texture)
      rl_image = rl.Image()
      rl_image.data = rl.ffi.cast("void *", img_array.ctypes.data)
      rl_image.width = pil_img.width
      rl_image.height = pil_img.height
      rl_image.mipmaps = 1
      rl_image.format = rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
      self.qr_texture = rl.load_texture_from_image(rl_image)
      self._drawn_url = url
    except Exception:
      cloudlog.exception("qmpilot QR failed")
      self.qr_texture = None

  def _status_text(self) -> str:
    if not self.dongle_id:
      return tr("This device has no dongle id.")
    if self._saved:
      return tr("Paired. Uploads go to qmpilot-connect.")
    if self._worker is None:
      return ""
    _, message, _, _ = self._worker.snapshot()
    if message == "wifi":
      return tr("Connect the car to Wi-Fi, then this code will appear.")
    if message == "conflict":
      return tr("This dongle is already registered to another key.")
    if message == "rejected":
      return tr("qmpilot-connect rejected the device key.")
    if message == "start_failed":
      return tr("Cannot reach qmpilot-connect.com. Check Wi-Fi.")
    if message == "paired":
      return tr("Paired. Uploads go to qmpilot-connect.")
    return tr("On your phone, scan the code. It opens qmpilot-connect.com.")

  def _render(self, rect: rl.Rectangle) -> int:
    rl.clear_background(rl.Color(224, 224, 224, 255))
    margin = 70
    content = rl.Rectangle(rect.x + margin, rect.y + margin, rect.width - 2 * margin, rect.height - 2 * margin)
    close_rect = rl.Rectangle(content.x - 20, content.y - 20, 120, 120)
    self._close_btn.render(close_rect)

    title_font = gui_app.font(FontWeight.BOLD)
    title = tr("Pair qmpilot")
    rl.draw_text_ex(title_font, title, rl.Vector2(content.x, content.y + 100), 80, 0.0, rl.BLACK)

    id_font = gui_app.font(FontWeight.NORMAL)
    dongle = self.dongle_id or tr("no dongle id")
    rl.draw_text_ex(id_font, dongle, rl.Vector2(content.x, content.y + 200), 64, 0.0, rl.BLACK)

    body_font = gui_app.font(FontWeight.NORMAL)
    left_width = int(content.width * 0.48)
    wrapped = wrap_text(body_font, self._status_text(), 42, left_width)
    rl.draw_text_ex(body_font, "\n".join(wrapped), rl.Vector2(content.x, content.y + 300), 42, 0.0, rl.Color(40, 40, 40, 255))

    hint = tr("The code does not contain the API key.")
    rl.draw_text_ex(body_font, hint, rl.Vector2(content.x, content.y + content.height - 60), 32, 0.0, rl.Color(80, 80, 80, 255))

    qr_size = min(content.width * 0.42, content.height - 80)
    qr_rect = rl.Rectangle(content.x + content.width - qr_size, content.y + 80, qr_size, qr_size)
    self._render_qr(qr_rect)
    return -1

  def _render_qr(self, rect: rl.Rectangle) -> None:
    if not self.qr_texture:
      rl.draw_rectangle_rounded(rect, 0.02, 8, rl.WHITE)
      font = gui_app.font(FontWeight.BOLD)
      label = tr("Waiting")
      size = measure_text_cached(font, label, 36)
      rl.draw_text_ex(font, label, rl.Vector2(rect.x + (rect.width - size.x) / 2, rect.y + rect.height / 2), 36, 0.0, rl.BLACK)
      return
    source = rl.Rectangle(0, 0, self.qr_texture.width, self.qr_texture.height)
    rl.draw_texture_pro(self.qr_texture, source, rect, rl.Vector2(0, 0), 0, rl.WHITE)

  def __del__(self):
    if self.qr_texture and self.qr_texture.id != 0:
      rl.unload_texture(self.qr_texture)
