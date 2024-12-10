#include "cereal/messaging/messaging.h"

int main() {
  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto pss = evt.initPandaStates(1);
  auto ps = pss[0];
  ps.setVoltage(12);
  ps.setCurrent(2);
  ps.setUptime(0);
  ps.setSafetyTxBlocked(0);
  ps.setSafetyRxInvalid(0);
  ps.setIgnitionLine(0);
  ps.setIgnitionCan(0);
  ps.setControlsAllowed(1);
  ps.setTxBufferOverflow(0);
  ps.setRxBufferOverflow(0);
  ps.setPandaType(cereal::PandaState::PandaType::DOS);
  ps.setSafetyModel(cereal::CarParams::SafetyModel(0));
  ps.setSafetyParam(0);
  ps.setFaultStatus(cereal::PandaState::FaultStatus(0));
  ps.setPowerSaveEnabled((bool)(0));
  ps.setHeartbeatLost((bool)(0));
  ps.setAlternativeExperience(0);
  ps.setHarnessStatus(
      cereal::PandaState::HarnessStatus(1));
  ps.setInterruptLoad(0);
  ps.setFanPower(1);
  ps.setFanStallCount(0);
  ps.setSafetyRxChecksInvalid((bool)(0));
  ps.setSpiChecksumErrorCount(0);
  ps.setSbu1Voltage(12000 / 1000.0f);
  ps.setSbu2Voltage(12000 / 1000.0f);
  PubMaster pm({"pandaStates"});
  pm.send("pandaStates", msg);
  ps.setUptime(10);
  ps.setIgnitionLine(1);
  ps.setIgnitionCan(1);
  pm.send("pandaStates", msg);
  return 0;
}
