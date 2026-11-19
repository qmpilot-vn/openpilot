"""
VinFast MADS is independent of ACC main. The wheel HWA button is stock
hands-on highway assist — do not map it to ButtonType.lkas.

SPD± = MFS_CruiseSpeedIncrease/Decrease (raw paddles, work with ACC off)
     OR ADAS_ACC_SpdSet_*_Feed (ACC HUD echo, mainly while ACC controlling).

While ACC is not controlling (ADAS_ACC_Mode != 4):
  SPD+ while MADS is off → ENABLE
  SPD- while MADS is on → USER_DISABLE
While ACC is controlling, SPD± only change set speed (MADS ignores them).
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
