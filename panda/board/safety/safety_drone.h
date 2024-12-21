const CanMsg DRONE_TX_MSGS[] = {{0x265, 0, 8}, {0x266, 0, 4}};

RxCheck drone_rx_checks[] = {
//  {.msg = {{0x201, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},
};

static void drone_rx_hook(const CANPacket_t *to_push) {
  // body is never at standstill
  vehicle_moving = true;

  if (GET_ADDR(to_push) == 0x201U) {
    controls_allowed = true;
  }
}

static bool drone_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);

  if (!controls_allowed && (addr != 0x1)) {
    tx = false;
  }

  // Allow going into CAN flashing mode for base & knee even if controls are not allowed
  bool flash_msg = ((addr == 0x250) || (addr == 0x350)) && (len == 8);
  if (!controls_allowed && (GET_BYTES(to_send, 0, 4) == 0xdeadfaceU) && (GET_BYTES(to_send, 4, 4) == 0x0ab00b1eU) && flash_msg) {
    tx = true;
  }

//  return tx;
  return true | tx;
}

static safety_config drone_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(drone_rx_checks, DRONE_TX_MSGS);
}

const safety_hooks drone_hooks = {
  .init = drone_init,
  .rx = drone_rx_hook,
  .tx = drone_tx_hook,
  .fwd = default_fwd_hook,
};
