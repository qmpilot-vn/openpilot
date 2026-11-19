"""
VinFast MADS is independent of ACC main. The wheel HWA button is stock
hands-on highway assist — do not map it to ButtonType.lkas.

While ACC is not controlling (ADAS_ACC_Mode != 4):
  SPD+ (SpdSet_upper) or MFS SET (setCruise) while MADS is off → ENABLE
  SPD- while MADS is on → USER_DISABLE
  MFS SET while MADS is already on → leave for long engage
While ACC is controlling, SPD± / SET change speed / long only.
SpdSet feeds are often silent until the car has been SET once — that is why
SET also arms MADS on the first onroad press.
"""

ENABLE = "enable"
DISABLE = "disable"


def cruise_available(acc_main_mode, acc_mode) -> bool:
  """ACC main on. Standby/active alone must not keep available stuck true
  (00000342: mode 2/3/4 with main=1, cancel never dropped available)."""
  mode = int(acc_mode or 0)
  if mode == 7:
    return False
  return int(acc_main_mode or 0) == 1


def mads_button_action(enabled: bool, long_enabled: bool, is_set_plus: bool, is_set_minus: bool) -> str | None:
  if is_set_plus and not enabled:
    return ENABLE
  if is_set_minus and enabled and not long_enabled:
    return DISABLE
  return None
