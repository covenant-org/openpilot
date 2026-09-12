#pragma once

#include <string>

#include "common/params.h"
#include "selfdrive/pandad/panda.h"

void pandad_main_thread(std::string serial);

class PandaSafety {
public:
  PandaSafety(Panda *panda) : panda_(panda) {}
  // obd_capture: covenant read-only J1939 capture — route the OBD bus via ELM327
  // multiplexing even when not onroad (see pandad.cc). Never transmits.
  void configureSafetyMode(bool is_onroad, bool obd_capture = false);

private:
  void updateMultiplexingMode();
  std::string fetchCarParams();
  void setSafetyMode(const std::string &params_string);

  bool initialized_ = false;
  bool log_once_ = false;
  bool safety_configured_ = false;
  bool prev_obd_multiplexing_ = false;
  Panda *panda_;
  Params params_;
};
