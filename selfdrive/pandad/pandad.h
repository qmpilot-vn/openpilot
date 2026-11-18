#pragma once

#include <string>
#include <vector>

#include "common/params.h"
#include "selfdrive/pandad/panda.h"

void pandad_main_thread(std::vector<std::string> serials);

// deprecated devices
static const std::vector<cereal::PandaState::PandaType> SUPPORTED_PANDA_TYPES = {
  cereal::PandaState::PandaType::RED_PANDA,
  cereal::PandaState::PandaType::TRES,
  cereal::PandaState::PandaType::CUATRO,
};


class PandaSafety {
public:
  PandaSafety(const std::vector<Panda *> &pandas);
  void configureSafetyMode(bool is_onroad);
  bool getOffroadMode();

private:
  void updateMultiplexingMode();
  std::vector<std::string> fetchCarParams();
  void setSafetyMode(const std::vector<std::string> &params_string);
  void applySafetyToPandas(const cereal::CarParams::Reader &car_params,
                           const cereal::CarParamsSP::Reader *car_params_sp_or_null);
  void invalidateSafetyCache();

  bool initialized_ = false;
  bool log_once_ = false;
  bool safety_configured_ = false;
  bool prev_obd_multiplexing_ = false;
  std::vector<Panda *> pandas_;
  Params params_;

  // Skip redundant set_safety_model / set_alternative_experience (10Hz) — cuts SPI/USB load on Panda[0].
  struct CachedSafety {
    bool valid = false;
    cereal::CarParams::SafetyModel model = cereal::CarParams::SafetyModel::SILENT;
    uint16_t safety_param = 0;
    uint16_t alternative_experience = 0;
    uint16_t safety_param_sp = 0;
  };
  std::vector<CachedSafety> cached_safety_;
};
