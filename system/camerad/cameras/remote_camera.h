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

class RemoteCamera {
private:
  std::thread fetch_thread;
  std::string last_frame;
  bool recv_frame;
  std::string ip;
  uint16_t port;
  uint16_t height;
  uint16_t width;
  unsigned int frame_id = 0;
  cl_context ctx;
  cl_command_queue queue;
  cl_program program;
  cl_kernel kernel;
  VisionIpcServer *vipc_server;
  cl_mem output_cl;
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

void remote_camerad_thread();
