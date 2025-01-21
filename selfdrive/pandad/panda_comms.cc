#include "panda_comms.h"
#include "common/util.h"
#include "panda.h"
#include "selfdrive/pandad/panda.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/system.h>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <tuple>
#include <unistd.h>

#include "common/swaglog.h"

static libusb_context *init_usb_ctx() {
  libusb_context *context = nullptr;
  int err = libusb_init(&context);
  if (err != 0) {
    LOGE("libusb initialization error");
    return nullptr;
  }

#if LIBUSB_API_VERSION >= 0x01000106
  libusb_set_option(context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
#else
  libusb_set_debug(context, 3);
#endif
  return context;
}

PandaUsbHandle::PandaUsbHandle(std::string serial) : PandaCommsHandle(serial) {
  // init libusb
  ssize_t num_devices;
  libusb_device **dev_list = NULL;
  int err = 0;
  ctx = init_usb_ctx();
  if (!ctx) {
    goto fail;
  }

  // connect by serial
  num_devices = libusb_get_device_list(ctx, &dev_list);
  if (num_devices < 0) {
    goto fail;
  }
  for (size_t i = 0; i < num_devices; ++i) {
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev_list[i], &desc);
    if (desc.idVendor == 0x3801 && desc.idProduct == 0xddcc) {
      int ret = libusb_open(dev_list[i], &dev_handle);
      if (dev_handle == NULL || ret < 0) {
        goto fail;
      }

      unsigned char desc_serial[26] = {0};
      ret = libusb_get_string_descriptor_ascii(
          dev_handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      if (ret < 0) {
        goto fail;
      }

      hw_serial = std::string((char *)desc_serial, ret);
      if (serial.empty() || serial == hw_serial) {
        break;
      }
      libusb_close(dev_handle);
      dev_handle = NULL;
    }
  }
  if (dev_handle == NULL)
    goto fail;
  libusb_free_device_list(dev_list, 1);
  dev_list = nullptr;

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    libusb_detach_kernel_driver(dev_handle, 0);
  }

  err = libusb_set_configuration(dev_handle, 1);
  if (err != 0) {
    goto fail;
  }

  err = libusb_claim_interface(dev_handle, 0);
  if (err != 0) {
    goto fail;
  }

  return;

fail:
  if (dev_list != NULL) {
    libusb_free_device_list(dev_list, 1);
  }
  cleanup();
  throw std::runtime_error("Error connecting to panda");
}

PandaUsbHandle::~PandaUsbHandle() {
  std::lock_guard lk(hw_lock);
  cleanup();
  connected = false;
}

void PandaUsbHandle::cleanup() {
  if (dev_handle) {
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
  }

  if (ctx) {
    libusb_exit(ctx);
  }
}

std::vector<std::string> PandaUsbHandle::list() {
  static std::unique_ptr<libusb_context, decltype(&libusb_exit)> context(
      init_usb_ctx(), libusb_exit);
  // init libusb
  ssize_t num_devices;
  libusb_device **dev_list = NULL;
  std::vector<std::string> serials;
  if (!context) {
    return serials;
  }

  num_devices = libusb_get_device_list(context.get(), &dev_list);
  if (num_devices < 0) {
    LOGE("libusb can't get device list");
    goto finish;
  }
  for (size_t i = 0; i < num_devices; ++i) {
    libusb_device *device = dev_list[i];
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(device, &desc);
    if (desc.idVendor == 0x3801 && desc.idProduct == 0xddcc) {
      libusb_device_handle *handle = NULL;
      int ret = libusb_open(device, &handle);
      if (ret < 0) {
        goto finish;
      }

      unsigned char desc_serial[26] = {0};
      ret = libusb_get_string_descriptor_ascii(
          handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      libusb_close(handle);
      if (ret < 0) {
        goto finish;
      }

      serials.push_back(std::string((char *)desc_serial, ret));
    }
  }

finish:
  if (dev_list != NULL) {
    libusb_free_device_list(dev_list, 1);
  }
  return serials;
}

void PandaUsbHandle::handle_usb_issue(int err, const char func[]) {
  LOGE_100("usb error %d \"%s\" in %s", err,
           libusb_strerror((enum libusb_error)err), func);
  if (err == LIBUSB_ERROR_NO_DEVICE) {
    LOGE("lost connection");
    connected = false;
  }
  // TODO: check other errors, is simply retrying okay?
}

int PandaUsbHandle::control_write(uint8_t bRequest, uint16_t wValue,
                                  uint16_t wIndex, unsigned int timeout) {
  int err;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_OUT |
                                LIBUSB_REQUEST_TYPE_VENDOR |
                                LIBUSB_RECIPIENT_DEVICE;

  if (!connected) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue,
                                  wIndex, NULL, 0, timeout);
    if (err < 0)
      handle_usb_issue(err, __func__);
  } while (err < 0 && connected);

  return err;
}

int PandaUsbHandle::control_read(uint8_t bRequest, uint16_t wValue,
                                 uint16_t wIndex, unsigned char *data,
                                 uint16_t wLength, unsigned int timeout) {
  int err;
  const uint8_t bmRequestType =
      LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;

  if (!connected) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue,
                                  wIndex, data, wLength, timeout);
    if (err < 0)
      handle_usb_issue(err, __func__);
  } while (err < 0 && connected);

  return err;
}

int PandaUsbHandle::bulk_write(unsigned char endpoint, unsigned char *data,
                               int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected) {
    return 0;
  }

  std::lock_guard lk(hw_lock);
  do {
    // Try sending can messages. If the receive buffer on the panda is full it
    // will NAK and libusb will try again. After 5ms, it will time out. We will
    // drop the messages.
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred,
                               timeout);

    if (err == LIBUSB_ERROR_TIMEOUT) {
      LOGW("Transmit buffer full");
      break;
    } else if (err != 0 || length != transferred) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected);

  return transferred;
}

int PandaUsbHandle::bulk_read(unsigned char endpoint, unsigned char *data,
                              int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected) {
    return 0;
  }

  std::lock_guard lk(hw_lock);

  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred,
                               timeout);

    if (err == LIBUSB_ERROR_TIMEOUT) {
      break; // timeout is okay to exit, recv still happened
    } else if (err == LIBUSB_ERROR_OVERFLOW) {
      comms_healthy = false;
      LOGE_100("overflow got 0x%x", transferred);
    } else if (err != 0) {
      handle_usb_issue(err, __func__);
    }

  } while (err != 0 && connected);

  return transferred;
}

bool PandaMavlinkHandle::connect_autopilot() {
  if (this->mavsdk_system.has_value()) {
    return true;
  }
  char *uri = std::getenv("MAVLINK_CONNECTION_URI");
  if (uri != nullptr) {
    this->mavlink_uri = uri;
  }

  this->mavsdk_connection_result =
      this->mavsdk.add_any_connection(this->mavlink_uri);
  if (this->mavsdk_connection_result != mavsdk::ConnectionResult::Success) {
    LOGE("MAVSDK: connection failed");
    return false;
  }

  this->mavsdk_system = this->mavsdk.first_autopilot(3.0);
  if (!this->mavsdk_system) {
    LOGE("MAVSDK: no autopilot found");
    return false;
  }
  this->mavsdk_action_plugin =
      std::make_shared<mavsdk::Action>(this->mavsdk_system.value());
  this->mavsdk_offboard_plugin =
      std::make_shared<mavsdk::Offboard>(this->mavsdk_system.value());
  this->mavsdk_telemetry_plugin =
      std::make_shared<mavsdk::Telemetry>(this->mavsdk_system.value());

  this->mavsdk_telemetry_plugin->subscribe_odometry(
      [&](mavsdk::Telemetry::Odometry odometry) {
        this->mavsdk_telemetry_messages.odometry = odometry;
      });
  this->mavsdk_telemetry_plugin->subscribe_position_velocity_ned(
      [&](mavsdk::Telemetry::PositionVelocityNed position_velocity_ned) {
        this->mavsdk_telemetry_messages.position_velocity_ned =
            position_velocity_ned;
      });
  this->mavsdk_telemetry_plugin->subscribe_position(
      [&](mavsdk::Telemetry::Position position) {
        this->mavsdk_telemetry_messages.position = position;
      });
  this->mavsdk_telemetry_plugin->subscribe_armed([&](bool armed) {
    this->ignited = armed;
    if (!armed){
      this->should_start_offboard = true;
    }
    if (this->should_start_offboard && armed &&
        this->mavsdk_telemetry_messages.position.relative_altitude_m <= 1.0) {
      this->mavsdk_action_plugin->set_takeoff_altitude(this->min_height + 1);
      this->mavsdk_action_plugin->takeoff_async(
          [](mavsdk::Action::Result success) { (void)success; });
    }
  });
  return true;
}

PandaMavlinkHandle::PandaMavlinkHandle(std::string serial)
    : PandaCommsHandle(serial),
      mavsdk{mavsdk::Mavsdk{mavsdk::Mavsdk::Configuration(
          mavsdk::Mavsdk::ComponentType::GroundStation)}} {
  this->hw_serial = serial;
  if (!this->connect_autopilot()) {
    throw std::runtime_error("Failed to connect to autopilot");
  }
}

PandaMavlinkHandle::~PandaMavlinkHandle() { this->connected = false; }

std::vector<std::string> PandaMavlinkHandle::list() {
  return PandaUsbHandle::list();
}

int PandaMavlinkHandle::control_write(uint8_t bRequest, uint16_t wValue,
                                      uint16_t wIndex, unsigned int timeout) {
  switch (bRequest) {
  case PandaEndpoints::SET_SAFETY_MODEL:
    this->safety_model = wValue;
    return 0;
  case PandaEndpoints::SET_ALTERNATIVE_EXPERIENCE:
    this->alternative_experience = wValue;
    return 0;
  case PandaEndpoints::SET_FAN_SPEED:
    this->fan_speed = wValue;
    return 0;
  case PandaEndpoints::SET_IR_PWR:
    this->ir_pwr = wValue;
    return 0;
  case PandaEndpoints::SET_LOOPBACK:
    this->loopback = wValue;
    return 0;
  case PandaEndpoints::SET_POWER_SAVING:
    this->power_saving = wValue;
    return 0;
  case PandaEndpoints::SET_CAN_SPEED:
    this->can_speed[wValue] = wIndex;
    return 0;
  case PandaEndpoints::SET_DATA_SPEED:
    this->data_speed[wValue] = wIndex;
    return 0;
  case PandaEndpoints::SET_CANFD_NON_ISO:
    this->canfd[wValue] = wIndex;
    return 0;
  case PandaEndpoints::ENABLE_DEEPSLEEP:
    this->deepsleep = wValue;
    return 0;
  case PandaEndpoints::SEND_HEARTBEAT:
    this->engaged = wValue;
    return 0;
  default:
    return 0;
  }
}

int PandaMavlinkHandle::control_read(uint8_t bRequest, uint16_t wValue,
                                     uint16_t wIndex, unsigned char *data,
                                     uint16_t wLength, unsigned int timeout) {
  switch (bRequest) {
  case PandaEndpoints::GET_FAN_SPEED:
    *data = this->fan_speed;
    break;
  case PandaEndpoints::GET_HW_TYPE:
    *data = 1;
    break;
  case PandaEndpoints::GET_FIRMWARE_VERSION_FIRST: {
    auto content = util::read_file(std::string("../../panda/board/obj/") +
                                   "panda.bin.signed");
    memcpy(data, content.data() + content.size() - 128, 64);
    break;
  }
  case PandaEndpoints::GET_FIRMWARE_VERSION_SECOND: {
    auto content = util::read_file(std::string("../../panda/board/obj/") +
                                   "panda.bin.signed");
    memcpy(data, content.data() + content.size() - 64, 64);
    break;
  }
  case PandaEndpoints::GET_STATE: {
    // uptime
    this->uptime += 100;
    health_t info;
    info.uptime_pkt = this->uptime;
    info.voltage_pkt = 12;
    info.current_pkt = 1 * 1000;
    info.safety_tx_blocked_pkt = 0;
    info.safety_rx_invalid_pkt = 0;
    info.tx_buffer_overflow_pkt = 0;
    info.rx_buffer_overflow_pkt = 0;
    info.faults_pkt = 0;
    info.ignition_line_pkt = this->ignited;
    info.ignition_can_pkt = this->ignited;
    info.controls_allowed_pkt = 1;
    info.car_harness_status_pkt = 1;
    info.safety_mode_pkt = this->safety_model;
    info.safety_param_pkt = 0;
    info.fault_status_pkt = 0;
    info.power_save_enabled_pkt = this->power_saving;
    info.heartbeat_lost_pkt = 0;
    info.alternative_experience_pkt = this->alternative_experience;
    info.interrupt_load_pkt = 0;
    info.fan_power = 0;
    info.safety_rx_checks_invalid_pkt = 0;
    info.spi_checksum_error_count_pkt = 0;
    info.fan_stall_count = 0;
    info.sbu1_voltage_mV = 12000;
    info.sbu2_voltage_mV = 12000;
    info.som_reset_triggered = 0;
    memcpy(data, (char *)&info, sizeof(health_t));
    break;
  }
  case PandaEndpoints::GET_CAN_STATE: {
    can_health_t info;
    info.bus_off = 10;
    info.bus_off_cnt = 0;
    info.error_warning = 0;
    info.error_passive = 0;
    info.last_error = 0;
    info.last_stored_error = 0;
    info.last_data_error = 0;
    info.last_data_stored_error = 0;
    info.receive_error_cnt = 0;
    info.transmit_error_cnt = 0;
    info.total_error_cnt = 0;
    info.total_tx_lost_cnt = 0;
    info.total_rx_lost_cnt = 0;
    info.total_tx_cnt = 0;
    info.total_rx_cnt = 0;
    info.total_fwd_cnt = 0;
    info.total_tx_checksum_error_cnt = 0;
    info.can_speed = 1000;
    info.can_data_speed = 1000;
    info.canfd_enabled = 1;
    info.brs_enabled = 0;
    info.canfd_non_iso = 0;
    info.irq0_call_rate = 1000000;
    info.irq1_call_rate = 1000000;
    info.irq2_call_rate = 1000000;
    info.can_core_reset_cnt = 0;
    memcpy(data, (char *)&info, sizeof(can_health_t));
    break;
  }
  }
  return wLength;
}

uint8_t Panda::calculate_checksum(uint8_t *data, uint32_t len) {
  uint8_t checksum = 0U;
  for (uint32_t i = 0U; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

bool unpack_can_buffer(uint8_t *data, uint32_t &size,
                       std::vector<can_frame> &out_vec) {
  int pos = 0;

  while (pos <= size - sizeof(can_header)) {
    can_header header;
    memcpy(&header, &data[pos], sizeof(can_header));

    const uint8_t data_len = dlc_to_len[header.data_len_code];
    if (pos + sizeof(can_header) + data_len > size) {
      // we don't have all the data for this message yet
      break;
    }

    if (calculate_checksum(&data[pos], sizeof(can_header) + data_len) != 0) {
      LOGE("Panda CAN checksum failed");
      size = 0;
      return false;
    }

    can_frame &canData = out_vec.emplace_back();
    canData.busTime = 0;
    canData.address = header.addr;
    canData.src = header.bus;
    if (header.rejected) {
      canData.src += CAN_REJECTED_BUS_OFFSET;
    }
    if (header.returned) {
      canData.src += CAN_RETURNED_BUS_OFFSET;
    }

    canData.dat.assign((char *)&data[pos + sizeof(can_header)], data_len);

    pos += sizeof(can_header) + data_len;
  }

  // move the overflowing data to the beginning of the buffer for the next round
  memmove(data, &data[pos], size - pos);
  size -= pos;

  return true;
}

static uint8_t len_to_dlc(uint8_t len) {
  if (len <= 8) {
    return len;
  }
  if (len <= 24) {
    return 8 + ((len - 8) / 4) + ((len % 4) ? 1 : 0);
  } else {
    return 11 + (len / 16) + ((len % 16) ? 1 : 0);
  }
}

uint32_t pack_can_msg(uint8_t bus, uint32_t address, const std::string &data,
                      unsigned char *buf) {
  uint8_t buffer[2 * USB_TX_SOFT_LIMIT];
  can_header header = {};
  uint8_t len_code = len_to_dlc(data.size());
  header.addr = address;
  header.extended = address > 0x7FF;
  header.data_len_code = len_code;
  header.bus = bus;
  header.checksum = 0;

  memcpy(&buffer[0], (uint8_t *)&header, sizeof(can_header));
  memcpy(&buffer[sizeof(can_header)], (uint8_t *)data.data(), data.size());
  uint32_t msg_size = sizeof(can_header) + data.size();
  ((can_header *)&buffer[0])->checksum =
      calculate_checksum(&buffer[0], msg_size);
  memcpy(buf, &buffer[0], msg_size);
  return msg_size;
}

#define CAN_MAX_DATA_SIZE 8
#define ISOTP_SINGLE_FRAME 0
#define ISOTP_FIRST_FRAME 1
#define ISOTP_CONT_FRAME 2

uint32_t to_isotp_frame(const std::string &response_code,
                        const std::string &data,
                        std::vector<std::string> &segments) {
  size_t total_data_len = response_code.size() + data.size();
  if (total_data_len < CAN_MAX_DATA_SIZE) {
    char type_and_len = (ISOTP_SINGLE_FRAME << 4) | total_data_len;
    std::string &segment = segments.emplace_back();
    segment.resize(sizeof(type_and_len) + total_data_len);
    memcpy(segment.data(), &type_and_len, sizeof(type_and_len));
    memcpy(segment.data() + sizeof(type_and_len), &response_code,
           sizeof(response_code));
    memcpy(segment.data() + sizeof(response_code) + sizeof(type_and_len),
           data.data(), data.size());
    return 1;
  }
  std::string full_msg = std::string(response_code);
  full_msg.append(data);
  size_t current_pos = 0;
  char packet_idx = 0;
  size_t offset = 0;
  size_t segment_size = 0;
  while (current_pos < full_msg.size()) {
    std::string &segment = segments.emplace_back();
    if (current_pos == 0) {
      segment_size = CAN_MAX_DATA_SIZE;
      segment.resize(segment_size);
      segment[0] =
          (char)(ISOTP_FIRST_FRAME << 4) | ((total_data_len & 0xF00) >> 8);
      segment[1] = (char)(total_data_len & 0xFF);
      offset = 2;
    } else {
      packet_idx++;
      segment_size = full_msg.size() - current_pos + 1;
      if (segment_size > CAN_MAX_DATA_SIZE) {
        segment_size = CAN_MAX_DATA_SIZE;
      }
      segment.resize(segment_size);
      segment[0] = (char)(ISOTP_CONT_FRAME << 4) | (packet_idx & 0xF);
      offset = 1;
    }
    char bytes_to_copy = segment_size - offset;
    memcpy(segment.data() + offset, full_msg.data() + current_pos,
           bytes_to_copy);
    current_pos += bytes_to_copy;
  }
  return packet_idx;
}

int PandaMavlinkHandle::bulk_write(unsigned char endpoint, unsigned char *data,
                                   int length, unsigned int timeout) {
  std::vector<can_frame> output;
  uint32_t size = (uint32_t)length;
  unpack_can_buffer(data, size, output);
  for (const can_frame &frame : output) {
    printf("Address %02lx: ", frame.address);
    if (frame.address == 0x265) {
      for (int i = 0; i < frame.dat.size(); i++) {
        printf("%02x ", frame.dat[i]);
      }
      printf("\n");
      int16_t angle = frame.dat[0] << 8 | frame.dat[1];
      int16_t speed = frame.dat[2] << 8 | frame.dat[3];
      printf("angle %d, speed %d\n", angle, speed);
      if (this->mavsdk_telemetry_messages.position.relative_altitude_m >=
              this->min_height &&
          !this->mavsdk_offboard_plugin->is_active()) {
        mavsdk::Offboard::VelocityBodyYawspeed stay{};
        this->mavsdk_offboard_plugin->set_velocity_body(stay);
        mavsdk::Offboard::Result result = this->mavsdk_offboard_plugin->start();
        if (result != mavsdk::Offboard::Result::Success) {
          LOGE("failed to start offboard");
        } else {
          this->should_start_offboard = false;
        }
      }
      if (!this->mavsdk_offboard_plugin->is_active()) continue;
      mavsdk::Offboard::VelocityBodyYawspeed command{};
      if (this->mavsdk_telemetry_messages.position.relative_altitude_m >
          this->min_height) {
        command.down_m_s = 0.5;
      }
      if (this->mavsdk_telemetry_messages.position.relative_altitude_m <
          this->min_height) {
        command.down_m_s = -0.5;
      }
      command.forward_m_s = speed / 100.0;
      this->mavsdk_offboard_plugin->set_velocity_body(command);
    }
    if (frame.address != 0x7DF && frame.address != this->ecu_add) {
      continue;
    }
    std::string frame_dat = frame.dat;
    char len = frame_dat[0] & 0x0F;
    if (frame_dat[1] == CANServiceTypes::READ_DATA_BY_IDENTIFIER) {
      uint32_t identifier = 0;
      for (size_t i = 2; i < 2 + len - 1; i++) {
        identifier <<= 8;
        identifier |= frame_dat[i];
      }
      if (identifier == CANIdentifiers::VIN) {
        {
          std::lock_guard lk(this->msg_lock);
          std::string response_code = {
              CANServiceTypes::READ_DATA_BY_IDENTIFIER + 0x40,
              (char)((CANIdentifiers::VIN & 0xFF00) >> 8),
              (char)(CANIdentifiers::VIN & 0xFF)};
          std::vector<std::string> segments;
          to_isotp_frame(response_code, this->vin, segments);
          for (const std::string &segment : segments) {
            this->msg_queue.emplace(std::make_tuple(0, 0x7e0 + 8, segment));
          }
        }
        this->msg_cv.notify_one();
      }

      if (identifier == CANIdentifiers::APPLICATION_SOFTWARE_IDENTIFICATION) {
        {
          std::lock_guard lk(this->msg_lock);
          std::string response_code = {
              CANServiceTypes::READ_DATA_BY_IDENTIFIER + 0x40,
              (char)((CANIdentifiers::APPLICATION_SOFTWARE_IDENTIFICATION &
                      0xFF00) >>
                     8),
              (char)(CANIdentifiers::APPLICATION_SOFTWARE_IDENTIFICATION &
                     0xFF)};
          std::vector<std::string> segments;
          to_isotp_frame(response_code, this->fw_version, segments);
          for (const std::string &segment : segments) {
            this->msg_queue.emplace(
                std::make_tuple(0, this->ecu_add + 8, segment));
          }
        }
        this->msg_cv.notify_one();
      }
    }
    if (frame_dat[1] == CANServiceTypes::TESTER_PRESENT) {
      std::string segment = {0x02, CANServiceTypes::TESTER_PRESENT + 0x40,
                             0x00};
      segment.resize(CAN_MAX_DATA_SIZE, 0);
      {
        std::lock_guard lk(this->msg_lock);
        this->msg_queue.emplace(std::make_tuple(0, this->ecu_add + 8, segment));
      }
      this->msg_cv.notify_one();
    }
    printf("\n");
  }
  return length;
}

int PandaMavlinkHandle::bulk_read(unsigned char endpoint, unsigned char *data,
                                  int length, unsigned int timeout) {
  int total_read = 0;
  std::unique_lock<std::mutex> lk(this->msg_lock);
  this->msg_cv.wait_for(lk, std::chrono::milliseconds(100),
                        [this] { return !this->msg_queue.empty(); });
  while (lk.owns_lock() && !this->msg_queue.empty()) {
    std::tuple<uint8_t, uint32_t, std::string> msg = this->msg_queue.front();
    if ((std::get<2>(msg).size() + sizeof(can_header) + total_read) >= length) {
      break;
    }
    this->msg_queue.pop();
    total_read += pack_can_msg(std::get<0>(msg), std::get<1>(msg),
                               std::get<2>(msg), data + total_read);
  }
  if (total_read + sizeof(can_header) + 2 < length) {
    mavsdk::Telemetry::VelocityBody velocity_body{
        this->mavsdk_telemetry_messages.odometry.velocity_body};
    float speed = std::sqrt(velocity_body.x_m_s * velocity_body.x_m_s +
                            velocity_body.y_m_s * velocity_body.y_m_s);
    uint16_t speed_mps = static_cast<uint16_t>(speed * 100);
    float yaw_rate = this->mavsdk_telemetry_messages.odometry
                         .angular_velocity_body.yaw_rad_s;
    yaw_rate *= 180 / M_PI;
    uint16_t yaw_rate_deg = static_cast<uint16_t>(yaw_rate * 10);
    std::string content = {
        (char)((yaw_rate_deg & 0xFF00) >> 8),
        (char)(yaw_rate_deg & 0xFF),
        (char)((speed_mps & 0xFF00) >> 8),
        (char)(speed_mps & 0xFF),
    };
    total_read += pack_can_msg(0, 0x266, content, data + total_read);
  }
  return total_read;
}
