#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#ifndef __APPLE__
#include <linux/spi/spidev.h>
#endif

#include <libusb-1.0/libusb.h>


#define TIMEOUT 0
#define SPI_BUF_SIZE 2048


// comms base class
class PandaCommsHandle {
public:
  PandaCommsHandle(std::string serial) {}
  virtual ~PandaCommsHandle() {}
  virtual void cleanup() = 0;

  std::string hw_serial;
  std::atomic<bool> connected = true;
  std::atomic<bool> comms_healthy = true;
  static std::vector<std::string> list();

  // HW communication
  virtual int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT) = 0;
  virtual int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT) = 0;
  virtual int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) = 0;
  virtual int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) = 0;
};

class PandaUsbHandle : public PandaCommsHandle {
public:
  PandaUsbHandle(std::string serial);
  ~PandaUsbHandle();
  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT);
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT);
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  void cleanup();

  static std::vector<std::string> list();

private:
  libusb_context *ctx = NULL;
  libusb_device_handle *dev_handle = NULL;
  std::recursive_mutex hw_lock;
  void handle_usb_issue(int err, const char func[]);
};

#ifndef __APPLE__
struct __attribute__((packed)) spi_header {
  uint8_t sync;
  uint8_t endpoint;
  uint16_t tx_len;
  uint16_t max_rx_len;
};

class PandaSpiHandle : public PandaCommsHandle {
public:
  PandaSpiHandle(std::string serial);
  ~PandaSpiHandle();
  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT);
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT);
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  void cleanup(){};

  static std::vector<std::string> list();

private:
  int spi_fd = -1;
  uint8_t tx_buf[SPI_BUF_SIZE];
  uint8_t rx_buf[SPI_BUF_SIZE];
  inline static std::recursive_mutex hw_lock;

  int wait_for_ack(uint8_t ack, uint8_t tx, unsigned int timeout, unsigned int length);
  int bulk_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len, unsigned int timeout);
  int spi_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout);
  int spi_transfer_retry(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout);
  int lltransfer(spi_ioc_transfer &t);

  spi_header header;
  uint32_t xfer_count = 0;
};

class PandaFakeHandle : public PandaCommsHandle {
public:
  PandaFakeHandle(std::string serial);
  ~PandaFakeHandle();
  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT);
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT);
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  void cleanup(){};

  static std::vector<std::string> list();

private:
  uint16_t safety_model = 0;
  uint16_t alternative_experience = 0;
  uint16_t fan_speed = 0;
  uint16_t ir_pwr = 0;
  uint16_t loopback = 0;
  uint32_t ecu_add = 0x722;
  bool deepsleep = 0;
  bool engaged = 0;
  bool power_saving = 0;
  uint16_t can_speed[4] = {0, 0, 0, 0};
  uint16_t data_speed[4] = {0, 0, 0, 0};
  uint16_t canfd[4] = {0, 0, 0, 0};
  uint32_t uptime = 0;
  bool ignited = 0;
  std::string vin = "JH4DB1542MS007683";
  std::queue<std::tuple<uint8_t, uint32_t, std::string>> msg_queue;
  std::mutex msg_lock;
  std::condition_variable msg_cv;
};

namespace PandaEndpoints{
  static const uint8_t SET_SAFETY_MODEL = 0xdc;
  static const uint8_t SET_ALTERNATIVE_EXPERIENCE = 0xdf;
  static const uint8_t SET_FAN_SPEED = 0xb1;
  static const uint8_t SET_IR_PWR = 0xb0;
  static const uint8_t SET_LOOPBACK = 0xe5;
  static const uint8_t SET_POWER_SAVING = 0xe7;
  static const uint8_t SET_CAN_SPEED = 0xde;
  static const uint8_t SET_DATA_SPEED = 0xf9;
  static const uint8_t SET_CANFD_NON_ISO = 0xfc;

  static const uint8_t CAN_RESET_COMMS = 0xc0;

  static const uint8_t ENABLE_DEEPSLEEP = 0xfb;
  static const uint8_t SEND_HEARTBEAT = 0xf3;

  static const uint8_t GET_HW_TYPE = 0xc1;
  static const uint8_t GET_FAN_SPEED = 0xb2;
  static const uint8_t GET_STATE= 0xd2;
  static const uint8_t GET_CAN_STATE= 0xc2;
  static const uint8_t GET_FIRMWARE_VERSION_FIST = 0xd3;
  static const uint8_t GET_FIRMWARE_VERSION_SECOND = 0xd4;

  static const unsigned char CAN_BULK_WRITE = 3;
  static const unsigned char CAN_JUNK_READ = 0xab;
  static const unsigned char CAN_BULK_READ = 0xa8;
}

namespace CANIdentifiers {
static const uint32_t VIN = 0xF190;
}

namespace CANServiceTypes {
  static const uint8_t READ_DATA_BY_IDENTIFIER = 0x22;
}

#endif
