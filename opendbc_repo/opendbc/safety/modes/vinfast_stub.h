#pragma once

// Host / openpilot tree builds without private opendbc/safety/modes/vinfast*.h headers.
// VinFast enforcement lives in flashed Panda firmware (build panda with the real headers present).
#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

static safety_config vinfast_stub_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = true;
  return (safety_config){NULL, 0, NULL, 0, false}; // NOLINT(readability/braces)
}

static bool vinfast_stub_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}

const safety_hooks vinfast_hooks = {
  .init = vinfast_stub_init,
  .rx = default_rx_hook,
  .tx = vinfast_stub_tx_hook,
};

const safety_hooks vinfast_vf6_hooks = {
  .init = vinfast_stub_init,
  .rx = default_rx_hook,
  .tx = vinfast_stub_tx_hook,
};
