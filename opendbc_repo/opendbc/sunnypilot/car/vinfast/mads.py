"""
VinFast MADS is ACC-main only. The wheel HWA button is stock hands-on
highway assist — do not map it to ButtonType.lkas.

SET+ arms, SET- disarms, and only while ACC is not controlling:
  long off + SET+ while MADS is off → ENABLE
  long off + SET- while MADS is on → USER_DISABLE
  long on  + SET+ or SET- → speed only, MADS stays
  disabled + SET- → no-op
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
