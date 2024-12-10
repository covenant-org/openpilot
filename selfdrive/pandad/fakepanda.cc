#include "cereal/messaging/messaging.h"

int main() {
  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto pss = evt.initPandaStates(1);
  auto ps = pss[0];
  //  ps.setVoltage(12);
  //  ps.setCurrent(2);
  //  ps.setUptime(0);
  //  ps.setSafetyTxBlocked(0);
  //  ps.setSafetyRxInvalid(health.safety_rx_invalid_pkt);
  ps.setIgnitionLine(1);
  ps.setIgnitionCan(1);
  //  ps.setControlsAllowed(health.controls_allowed_pkt);
  //  ps.setTxBufferOverflow(health.tx_buffer_overflow_pkt);
  //  ps.setRxBufferOverflow(health.rx_buffer_overflow_pkt);
  //  ps.setPandaType(panda->hw_type);
  //  ps.setSafetyModel(cereal::CarParams::SafetyModel(health.safety_mode_pkt));
  //  ps.setSafetyParam(health.safety_param_pkt);
  //  ps.setFaultStatus(cereal::PandaState::FaultStatus(health.fault_status_pkt));
  //  ps.setPowerSaveEnabled((bool)(health.power_save_enabled_pkt));
  //  ps.setHeartbeatLost((bool)(health.heartbeat_lost_pkt));
  //  ps.setAlternativeExperience(health.alternative_experience_pkt);
  //  ps.setHarnessStatus(
  //      cereal::PandaState::HarnessStatus(health.car_harness_status_pkt));
  //  ps.setInterruptLoad(health.interrupt_load_pkt);
  //  ps.setFanPower(health.fan_power);
  //  ps.setFanStallCount(health.fan_stall_count);
  //  ps.setSafetyRxChecksInvalid((bool)(health.safety_rx_checks_invalid_pkt));
  //  ps.setSpiChecksumErrorCount(health.spi_checksum_error_count_pkt);
  //  ps.setSbu1Voltage(health.sbu1_voltage_mV / 1000.0f);
  //  ps.setSbu2Voltage(health.sbu2_voltage_mV / 1000.0f);
  PubMaster pm({"pandaStates"});
  pm.send("pandaStates", msg);
  return 0;
}
