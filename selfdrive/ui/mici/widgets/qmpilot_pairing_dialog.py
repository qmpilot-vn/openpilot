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
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets.nav_widget import NavWidget

NetworkType = log.DeviceState.NetworkType


def _dongle_id(params: Params) -> str:
  raw = params.get("DongleId")
  if isinstance(raw, bytes):
    raw = raw.decode()
  return (raw or "").strip()


def _has_saved_key(params: Params) -> bool:
  raw = params.get("QmpilotApiKey")
  if isinstance(raw, bytes):
    raw = raw.decode()
  if (raw or "").strip():
    return True
  try:
    with open("/data/qmpilot/QmpilotApiKey") as f:
      return bool(f.read().strip())
  except OSError:
    return False


def _on_wifi() -> bool:
  try:
    return ui_state.sm["deviceState"].networkType == NetworkType.wifi
  except Exception:
    return False


class QmpilotPairingDialog(NavWidget):
  """Small-screen QR pair for qmpilot-connect. The API key is never drawn."""

  def __init__(self):
    super().__init__()
    self.params = Params()
    self.dongle_id = _dongle_id(self.params)
    self._qr_texture: rl.Texture | None = None
    self._drawn_url = ""
    self._saved = False
    self._save_failed = False
    self._opened_paired = _has_saved_key(self.params)
    self._worker: PairWorker | None = None

    sub = rl.Color(255, 255, 255, 170)
    self._title = UnifiedLabel(tr("pair qmpilot"), 42, font_weight=FontWeight.BOLD, line_height=0.9, wrap_text=True)
    self._dongle = UnifiedLabel(self.dongle_id or tr("no dongle id"), 28, font_weight=FontWeight.ROMAN, text_color=sub, wrap_text=False)
    self._status = UnifiedLabel(self._status_text, 26, font_weight=FontWeight.NORMAL, line_height=0.95, wrap_text=True)

  def show_event(self):
    super().show_event()
    if self._worker is not None or not self.dongle_id:
      return
    self._worker = PairWorker(QmpilotPairClient(self.dongle_id), _on_wifi)
    self._worker.start()

  def hide_event(self):
    if self._worker is not None:
      self._worker.stop()
      self._worker = None
    super().hide_event()

  def _update_state(self):
    super()._update_state()
    if self._saved or self._worker is None:
      return
    url, message, api_key, api_host = self._worker.snapshot()
    if message == "paired" and isinstance(api_key, str) and api_key and api_host:
      try:
        save_pair(self.params, api_host, api_key)
      except Exception:
        cloudlog.exception("qmpilot pair save failed")
        self._save_failed = True
        return
      self._saved = True
      # The first link closes itself. Opening an already paired car stays here,
      # otherwise the status key comes back immediately and the screen pops.
      if not self._opened_paired:
        self.dismiss()
      return
    if url and url != self._drawn_url:
      self._draw_qr(url)
    elif not url and self._drawn_url:
      self._clear_qr()

  def _draw_qr(self, url: str) -> None:
    if "api_key" in url or "QmpilotApiKey" in url:
      cloudlog.error("refusing a pair QR that contains a key")
      return
    try:
      qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, box_size=8, border=2)
      qr.add_data(url)
      qr.make(fit=True)
      pil_img = qr.make_image(fill_color="black", back_color="white").convert("RGBA")
      img_array = np.array(pil_img, dtype=np.uint8)
      if self._qr_texture and self._qr_texture.id != 0:
        rl.unload_texture(self._qr_texture)
      rl_image = rl.Image()
      rl_image.data = rl.ffi.cast("void *", img_array.ctypes.data)
      rl_image.width = pil_img.width
      rl_image.height = pil_img.height
      rl_image.mipmaps = 1
      rl_image.format = rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
      self._qr_texture = rl.load_texture_from_image(rl_image)
      self._drawn_url = url
    except Exception:
      cloudlog.exception("qmpilot QR failed")
      self._qr_texture = None

  def _clear_qr(self) -> None:
    if self._qr_texture and self._qr_texture.id != 0:
      rl.unload_texture(self._qr_texture)
    self._qr_texture = None
    self._drawn_url = ""

  def _status_text(self) -> str:
    if not self.dongle_id:
      return tr("This device has no dongle id.")
    if self._save_failed:
      return tr("Paired, but saving the key failed.")
    if self._saved:
      if self._opened_paired:
        return tr("Already paired. A new QR is only for a car that has no key yet.")
      return tr("Paired. Uploads go to qmpilot.")
    if self._worker is None:
      return ""
    _, message, _, _ = self._worker.snapshot()
    if message == "wifi":
      return tr("Connect to Wi-Fi. The code shows up after that.")
    if message == "conflict":
      return tr("This dongle is already registered to another key.")
    if message == "rejected":
      return tr("qmpilot-connect rejected the device key.")
    if message == "start_failed":
      return tr("Cannot reach qmpilot-connect.com.")
    if message == "paired":
      return tr("Paired. Uploads go to qmpilot.")
    return tr("Scan with your phone. It opens qmpilot-connect.com.")

  def _render(self, rect: rl.Rectangle):
    qr_size = rect.height - 16
    qr_rect = rl.Rectangle(rect.x + 8, rect.y + 8, qr_size, qr_size)
    self._render_qr(qr_rect)

    text_x = qr_rect.x + qr_rect.width + 16
    text_w = max(40, int(rect.x + rect.width - text_x - 8))
    self._title.set_max_width(text_w)
    self._dongle.set_max_width(text_w)
    self._status.set_max_width(text_w)
    self._title.set_position(text_x, rect.y + 18)
    self._title.render()
    self._dongle.set_position(text_x, rect.y + 78)
    self._dongle.render()
    self._status.set_position(text_x, rect.y + 118)
    self._status.render()

  def _render_qr(self, rect: rl.Rectangle) -> None:
    if not self._qr_texture:
      rl.draw_rectangle_rounded(rect, 0.05, 6, rl.Color(255, 255, 255, 30))
      return
    source = rl.Rectangle(0, 0, self._qr_texture.width, self._qr_texture.height)
    rl.draw_texture_pro(self._qr_texture, source, rect, rl.Vector2(0, 0), 0, rl.WHITE)

  def __del__(self):
    if self._qr_texture is not None and self._qr_texture.id != 0:
      rl.unload_texture(self._qr_texture)
