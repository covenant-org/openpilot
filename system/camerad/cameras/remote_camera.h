#pragma once
#include "cereal/gen/cpp/log.capnp.h"
#include "common/clutil.h"
#include "msgq/visionipc/visionipc_server.h"
#include <cstdint>
#ifdef QCOM2
#include "CL/cl_ext_qcom.h"
#endif
#define EGL_NO_X11
#ifdef Success
#undef Success
#endif
#include <capnp/serialize-packed.h>
#include "common/util.h"
#include "cereal/messaging/messaging.h"
#include <chrono>

class RemoteCamera {
private:
  std::thread fetch_thread;
  std::string last_frame;
  bool recv_frame;
  uint16_t frame_counter;
  std::chrono::time_point<std::chrono::system_clock> start;
  std::string ip;
  uint16_t port;
  uint16_t height;
  uint16_t width;
  unsigned int frame_id = 0;
  cl_command_queue queue;
  VisionIpcServer *vipc_server;
  PubMaster *pm;

public:
  explicit RemoteCamera(std::string addr, uint16_t port, uint16_t height,
                        uint16_t width, cl_context ctx, cl_device_id device_id,
                        VisionIpcServer *vipc_server);
  ~RemoteCamera();
  void fetch_frame_thread();
  void fetch_frame();
  void run();
  void init();
};

void remote_camerad_thread(std::string);
