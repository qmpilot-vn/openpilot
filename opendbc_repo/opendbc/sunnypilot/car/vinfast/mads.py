"""
VinFast MADS is independent of ACC main. The wheel HWA button is stock
hands-on highway assist — do not map it to ButtonType.lkas.

SPD+/SPD- arm and disarm lateral while ACC is not controlling. ACC main
does not need to be on (first onroad can arm lane-keep before any set speed):
  long off + SPD+ while MADS is off → ENABLE
  long off + SPD- while MADS is on → USER_DISABLE
  long on  + SPD+ or SPD- → speed only, MADS stays
  disabled + SPD- → no-op
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
