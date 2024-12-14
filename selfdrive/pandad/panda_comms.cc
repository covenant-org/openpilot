#include "panda_comms.h"
#include "selfdrive/pandad/panda.h"
#include "common/util.h"

#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <memory>

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
  if (!ctx) { goto fail; }

  // connect by serial
  num_devices = libusb_get_device_list(ctx, &dev_list);
  if (num_devices < 0) { goto fail; }
  for (size_t i = 0; i < num_devices; ++i) {
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev_list[i], &desc);
    if (desc.idVendor == 0xbbaa && desc.idProduct == 0xddcc) {
      int ret = libusb_open(dev_list[i], &dev_handle);
      if (dev_handle == NULL || ret < 0) { goto fail; }

      unsigned char desc_serial[26] = { 0 };
      ret = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      if (ret < 0) { goto fail; }

      hw_serial = std::string((char *)desc_serial, ret);
      if (serial.empty() || serial == hw_serial) {
        break;
      }
      libusb_close(dev_handle);
      dev_handle = NULL;
    }
  }
  if (dev_handle == NULL) goto fail;
  libusb_free_device_list(dev_list, 1);
  dev_list = nullptr;

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    libusb_detach_kernel_driver(dev_handle, 0);
  }

  err = libusb_set_configuration(dev_handle, 1);
  if (err != 0) { goto fail; }

  err = libusb_claim_interface(dev_handle, 0);
  if (err != 0) { goto fail; }

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
  static std::unique_ptr<libusb_context, decltype(&libusb_exit)> context(init_usb_ctx(), libusb_exit);
  // init libusb
  ssize_t num_devices;
  libusb_device **dev_list = NULL;
  std::vector<std::string> serials;
  if (!context) { return serials; }

  num_devices = libusb_get_device_list(context.get(), &dev_list);
  if (num_devices < 0) {
    LOGE("libusb can't get device list");
    goto finish;
  }
  for (size_t i = 0; i < num_devices; ++i) {
    libusb_device *device = dev_list[i];
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(device, &desc);
    if (desc.idVendor == 0xbbaa && desc.idProduct == 0xddcc) {
      libusb_device_handle *handle = NULL;
      int ret = libusb_open(device, &handle);
      if (ret < 0) { goto finish; }

      unsigned char desc_serial[26] = { 0 };
      ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      libusb_close(handle);
      if (ret < 0) { goto finish; }

      serials.push_back(std::string((char *)desc_serial, ret).c_str());
    }
  }

finish:
  if (dev_list != NULL) {
    libusb_free_device_list(dev_list, 1);
  }
  return serials;
}

void PandaUsbHandle::handle_usb_issue(int err, const char func[]) {
  LOGE_100("usb error %d \"%s\" in %s", err, libusb_strerror((enum libusb_error)err), func);
  if (err == LIBUSB_ERROR_NO_DEVICE) {
    LOGE("lost connection");
    connected = false;
  }
  // TODO: check other errors, is simply retrying okay?
}

int PandaUsbHandle::control_write(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned int timeout) {
  int err;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;

  if (!connected) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, NULL, 0, timeout);
    if (err < 0) handle_usb_issue(err, __func__);
  } while (err < 0 && connected);

  return err;
}

int PandaUsbHandle::control_read(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout) {
  int err;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;

  if (!connected) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout);
    if (err < 0) handle_usb_issue(err, __func__);
  } while (err < 0 && connected);

  return err;
}

int PandaUsbHandle::bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected) {
    return 0;
  }

  std::lock_guard lk(hw_lock);
  do {
    // Try sending can messages. If the receive buffer on the panda is full it will NAK
    // and libusb will try again. After 5ms, it will time out. We will drop the messages.
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);

    if (err == LIBUSB_ERROR_TIMEOUT) {
      LOGW("Transmit buffer full");
      break;
    } else if (err != 0 || length != transferred) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected);

  return transferred;
}

int PandaUsbHandle::bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected) {
    return 0;
  }

  std::lock_guard lk(hw_lock);

  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);

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

PandaFakeHandle::PandaFakeHandle(std::string serial): PandaCommsHandle(serial) {
    this->hw_serial = serial;
}

PandaFakeHandle::~PandaFakeHandle() {
    this->connected = false;
}

std::vector<std::string> PandaFakeHandle::list() {
  std::vector<std::string> serials;
  serials.push_back("fake_1");
  return serials;
}

int PandaFakeHandle::control_write(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned int timeout) {
    switch (bRequest){
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

int PandaFakeHandle::control_read(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout) {
    switch (bRequest){
        case PandaEndpoints::GET_FAN_SPEED:
            *data = this->fan_speed;
            break;
        case PandaEndpoints::GET_HW_TYPE:
            *data = 1;
            break;
        case PandaEndpoints::GET_FIRMWARE_VERSION_FIST:
            {
                auto content = util::read_file(std::string("../../panda/board/obj/") + "panda.bin.signed");
                memcpy(data, content.data()+content.size()-128, 64);
                break;
           }
        case PandaEndpoints::GET_FIRMWARE_VERSION_SECOND:
            {
                auto content = util::read_file(std::string("../../panda/board/obj/") + "panda.bin.signed");
                memcpy(data, content.data()+content.size()-64, 64);
                break;
           }
        case PandaEndpoints::GET_STATE: {
            // uptime
            this->uptime += 100;
            data[0] = (this->uptime & 0xFF000000) >> 24;
            data[1] = (this->uptime & 0xFF0000) >> 16;
            data[2] = (this->uptime & 0xFF00) >> 8;
            data[3] = this->uptime & 0xFF;

            // voltage
            data[4] = 0;
            data[5] = 0;
            data[6] = 0;
            data[7] = 12;

            // current
            data[8] = 0;
            data[9] = 0;
            data[10] = 0;
            data[11] = 0;

            // tx_blocked
            data[12] = 0;
            data[13] = 0;
            data[14] = 0;
            data[15] = 0;

            // rx_invalid
            data[16] = 0;
            data[17] = 0;
            data[18] = 0;
            data[19] = 0;

            // tx_overflows
            data[20] = 0;
            data[21] = 0;
            data[22] = 0;
            data[23] = 0;

            // rx_overflows
            data[24] = 0;
            data[25] = 0;
            data[26] = 0;
            data[27] = 0;

            // faults
            data[28] = 0;
            data[29] = 0;
            data[30] = 0;
            data[31] = 0;

            // ignition line
            data[32] = this->ignited;

            // ignition can
            data[33] = this->ignited;

            // controls allowed
            data[34] = 1;

            // car harness status
            data[35] = 1; // normal

            // safety mode
            data[36] = 0;

            // fault_status
            data[37] = 0;

            // safety param
            data[38] = 0;
            data[39] = 0;

            //fault status
            data[40] = 0;

            // power save enabled
            data[41] = this->power_saving;

            // heartbeat lost
            data[42] = 0;

            // alternative experience
            data[43] = (this->alternative_experience & 0xFF00) >> 8;
            data[44] = this->alternative_experience & 0xFF;

            // interrupt load
            data[45] = 0;
            data[46] = 0;
            data[47] = 0;
            data[48] = 0;

            // fan power
            data[49] = 0;

            // safety rx checks invalid
            data[50] = 0;

            // spi checksum error count
            data[51] = 0;
            data[52] = 0;

            // fan stall count
            data[53] = 0;

            // sub1 voltage mv
            uint16_t volt = 12000;
            data[54] = (volt & 0xFF00) >> 8;
            data[55] = volt & 0xFF;

            // sub2 voltage mv
            data[56] = (volt & 0xFF00) >> 8;
            data[57] = volt & 0xFF;

            // som reset tirggered
            data[58] = 0;
            this->ignited = 1;
            break;
        }
    }
    return wLength;
}

int PandaFakeHandle::bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
    return length;
}

int PandaFakeHandle::bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
    return length;
}
