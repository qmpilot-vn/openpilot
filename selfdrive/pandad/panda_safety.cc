#include "selfdrive/pandad/pandad.h"
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"

PandaSafety::PandaSafety(const std::vector<Panda *> &pandas) : pandas_(pandas) {
  cached_safety_.resize(pandas_.size());
}

void PandaSafety::invalidateSafetyCache() {
  for (auto &c : cached_safety_) {
    c.valid = false;
  }
}

void PandaSafety::applySafetyToPandas(const cereal::CarParams::Reader &car_params,
                                      const cereal::CarParamsSP::Reader *car_params_sp_or_null) {
  if (cached_safety_.size() != pandas_.size()) {
    cached_safety_.resize(pandas_.size());
  }

  auto safety_configs = car_params.getSafetyConfigs();
  uint16_t alternative_experience = car_params.getAlternativeExperience();
  uint16_t safety_param_sp = 0U;
  if (car_params_sp_or_null != nullptr) {
    safety_param_sp = car_params_sp_or_null->getSafetyParam();
  }

  for (int i = 0; i < (int)pandas_.size(); ++i) {
    cereal::CarParams::SafetyModel safety_model = cereal::CarParams::SafetyModel::SILENT;
    uint16_t safety_param = 0U;
    if (i < (int)safety_configs.size()) {
      safety_model = safety_configs[i].getSafetyModel();
      safety_param = safety_configs[i].getSafetyParam();
    }

    CachedSafety &c = cached_safety_[i];
    if (c.valid && c.model == safety_model && c.safety_param == safety_param &&
        c.alternative_experience == alternative_experience && c.safety_param_sp == safety_param_sp) {
      continue;
    }

    LOGD("Panda %d: setting safety model: %d, param: %d, alternative experience: %d, param_sp: %d", i,
         (int)safety_model, safety_param, alternative_experience, safety_param_sp);
    pandas_[i]->set_alternative_experience(alternative_experience, safety_param_sp);
    pandas_[i]->set_safety_model(safety_model, safety_param);
    c.valid = true;
    c.model = safety_model;
    c.safety_param = safety_param;
    c.alternative_experience = alternative_experience;
    c.safety_param_sp = safety_param_sp;
  }
}

void PandaSafety::configureSafetyMode(bool is_onroad) {
  // Same latch / Params gating as pandad_backup_working_mici_spi_only_20260422 (string "1" in fetchCarParams).
  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params[0].size());
      LOGW("got %lu bytes CarParamsSP", car_params[1].size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (is_onroad && safety_configured_) {
    static int skip_logged = 0;
    if (++skip_logged % 200 == 1) {
      LOGW("configureSafetyMode: skipping apply (safety_configured_=true). Go offroad once to reset latch.");
    }
  } else if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
    invalidateSafetyCache();
  }
}

void PandaSafety::updateMultiplexingMode() {
  // Initialize to ELM327 without OBD multiplexing for initial fingerprinting
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    for (int i = 0; i < pandas_.size(); ++i) {
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    }
    initialized_ = true;
    invalidateSafetyCache();
  }

  // Switch between multiplexing modes based on the OBD multiplexing request
  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    for (int i = 0; i < pandas_.size(); ++i) {
      const uint16_t safety_param = (i > 0 || !obd_multiplexing_requested) ? 1U : 0U;
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
    }
    prev_obd_multiplexing_ = obd_multiplexing_requested;
    params_.putBool("ObdMultiplexingChanged", true);
    invalidateSafetyCache();
  }
}

// TODO-SP: Use structs instead of vector
std::vector<std::string> PandaSafety::fetchCarParams() {
  // NOTE: getBool() returns false on missing/empty reads too; log raw value to
  // distinguish "0" from "empty" (read failure / wrong prefix) when debugging.
  std::string fwq_raw = params_.get("FirmwareQueryDone");
  if (fwq_raw != "1") {
    static int n = 0;
    if (++n % 200 == 1) {
      LOGW("fetchCarParams: blocked — FirmwareQueryDone raw='%s' (len=%zu) path=%s",
           fwq_raw.c_str(), fwq_raw.size(), params_.getParamPath("FirmwareQueryDone").c_str());
    }
    return {};
  }

  if (!log_once_) {
    LOGW("Finished FW query, Waiting for params to set safety model");
    log_once_ = true;
  }

  std::string cr_raw = params_.get("ControlsReady");
  if (cr_raw != "1") {
    static int n = 0;
    if (++n % 200 == 1) {
      LOGW("fetchCarParams: blocked — ControlsReady raw='%s' (len=%zu) path=%s",
           cr_raw.c_str(), cr_raw.size(), params_.getParamPath("ControlsReady").c_str());
    }
    return {};
  }

  std::string cp = params_.get("CarParams");
  std::string sp = params_.get("CarParamsSP");
  if (cp.empty() || sp.empty()) {
    static int n = 0;
    if (++n % 200 == 1) {
      LOGW("fetchCarParams: blocked — empty blob CarParams=%zu bytes CarParamsSP=%zu bytes", cp.size(), sp.size());
    }
    return {};
  }
  return {std::move(cp), std::move(sp)};
}

// TODO-SP: Use structs instead of vector
void PandaSafety::setSafetyMode(const std::vector<std::string> &params_string) {
  AlignedBuffer aligned_buf;
  AlignedBuffer aligned_buf_sp;

  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string[0].data(), params_string[0].size()));
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  capnp::FlatArrayMessageReader cmsg_sp(aligned_buf_sp.align(params_string[1].data(), params_string[1].size()));
  cereal::CarParamsSP::Reader car_params_sp = cmsg_sp.getRoot<cereal::CarParamsSP>();

  applySafetyToPandas(car_params, &car_params_sp);
}

bool PandaSafety::getOffroadMode() {
  auto offroad_mode = params_.getBool("OffroadMode");
  return offroad_mode;
}
