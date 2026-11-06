#include "selfdrive/pandad/pandad.h"
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/util.h"

// Matches opendbc.car.vinfast.values.VinFastFlags (Honda/Hyundai style).
static constexpr uint32_t VINFAST_FLAG_RADAR_CANFD = 1U;
static constexpr uint32_t VINFAST_FLAG_RADAR_CANFD_SECOND = 2U;
static constexpr uint16_t VINFAST_RADAR_BUS = 1;
static constexpr uint16_t VINFAST_RADAR_ARB_KBPS = 500;
static constexpr uint16_t VINFAST_RADAR_DATA_KBPS = 2000;

static bool is_vinfast(const cereal::CarParams::Reader &cp) {
  const std::string brand = cp.getBrand().cStr();
  const std::string fp = cp.getCarFingerprint().cStr();
  return brand == "vinfast" || fp.rfind("VINFAST_", 0) == 0;
}

// Radar is CAN-FD on bus 1; chassis/cam stay classic. 0xf9 after every
// set_safety_model — panda firmware can_init_all drops the data bitrate.
static void apply_vinfast_radar_canfd(const std::vector<Panda *> &pandas,
                                      const cereal::CarParams::Reader &cp) {
  if (pandas.empty() || !is_vinfast(cp)) {
    return;
  }
  const uint32_t flags = cp.getFlags();
  const std::string fp = cp.getCarFingerprint().cStr();
  const bool src1 = (flags & VINFAST_FLAG_RADAR_CANFD) || fp.rfind("VINFAST_VF", 0) == 0;
  const bool src5 = (flags & VINFAST_FLAG_RADAR_CANFD_SECOND) ||
                    fp.rfind("VINFAST_VF6", 0) == 0 || fp.rfind("VINFAST_VF7", 0) == 0;
  if (src1) {
    pandas[0]->set_canfd_restore(VINFAST_RADAR_BUS, VINFAST_RADAR_ARB_KBPS, VINFAST_RADAR_DATA_KBPS);
  }
  if (src5 && pandas.size() > 1) {
    pandas[1]->set_canfd_restore(VINFAST_RADAR_BUS, VINFAST_RADAR_ARB_KBPS, VINFAST_RADAR_DATA_KBPS);
  }
}

static bool apply_vinfast_radar_canfd_blob(const std::vector<Panda *> &pandas, const std::string &blob) {
  if (blob.size() < 8) {
    return false;
  }
  try {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(blob.data(), blob.size()));
    auto cp = cmsg.getRoot<cereal::CarParams>();
    if (!is_vinfast(cp)) {
      return false;
    }
    apply_vinfast_radar_canfd(pandas, cp);
    return true;
  } catch (...) {
    return false;
  }
}

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

  apply_vinfast_radar_canfd(pandas_, car_params);

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
  if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
    invalidateSafetyCache();
    return;
  }

  // Always attempt apply onroad; CachedSafety skips redundant SPI when HW already matches.
  // Do not latch-skip after offroad NO_OUTPUT — cache may still show vinfast while panda is 19.
  updateMultiplexingMode();

  auto car_params = fetchCarParams();
  if (!car_params.empty()) {
    if (!safety_configured_) {
      LOGW("got %lu bytes CarParams", car_params[0].size());
      LOGW("got %lu bytes CarParamsSP", car_params[1].size());
    }
    setSafetyMode(car_params);
    safety_configured_ = true;
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

void PandaSafety::applyCanFdFromStoredParams() {
  for (const char *key : {"CarParams", "CarParamsCache", "CarParamsPersistent", "CarParamsPrevRoute"}) {
    if (apply_vinfast_radar_canfd_blob(pandas_, params_.get(key))) {
      return;
    }
  }
}

bool PandaSafety::getOffroadMode() {
  auto offroad_mode = params_.getBool("OffroadMode");
  return offroad_mode;
}
