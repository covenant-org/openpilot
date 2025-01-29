#include "remote_camera.h"
#include "common/swaglog.h"
#include <CL/cl.h>
#include <arpa/inet.h> // inet_addr()
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

extern ExitHandler do_exit;

RemoteCamera::RemoteCamera(std::string addr, uint16_t port, uint16_t height,
                           uint16_t width, cl_context ctx,
                           cl_device_id device_id, VisionIpcServer *vipc_server)
    : ip(addr), port(port), height(height), width(width), ctx(ctx),
      vipc_server(vipc_server), last_frame(height * width * 3 / 2, ' ') {
  (void)this->vipc_server;
  cl_int err = 0;
  this->queue = clCreateCommandQueueWithProperties(ctx, device_id, 0, &err);
  assert(err == CL_SUCCESS);
  char cl_arg[1024];
  sprintf(cl_arg,
          " -DHEIGHT=%d -DWIDTH=%d -DRGB_STRIDE=%d -DUV_WIDTH=%d "
          "-DUV_HEIGHT=%d -DRGB_SIZE=%d -DCL_DEBUG ",
          height, width, width * 3, width / 2, height / 2, height * width);
  this->program = cl_program_from_file(
      ctx, device_id, "/data/openpilot/tools/rgb_to_nv12.cl", cl_arg);
  this->kernel = clCreateKernel(program, "rgb_to_nv12", &err);
  assert(err == CL_SUCCESS);
  this->output_cl = clCreateBuffer(ctx, CL_MEM_WRITE_ONLY,
                                   width * height * 3 / 2, NULL, &err);
  assert(err == CL_SUCCESS);
  LOGW("Created remote camera\n");
}

RemoteCamera::~RemoteCamera() {
  clReleaseKernel(kernel);
  clReleaseProgram(program);
  clReleaseMemObject(output_cl);
  clReleaseCommandQueue(queue);
  if (this->pm != nullptr) {
    delete this->pm;
  }
}

void RemoteCamera::fetch_frame() {
  LOGW("Attempting to fetch frame");
  int sfd = socket(AF_INET, SOCK_STREAM, 0);
  assert(sfd >= 0);
  sockaddr_in server_addr = {
      .sin_family = AF_INET,
      .sin_port = htons(this->port),
      .sin_addr = {.s_addr = inet_addr(this->ip.c_str())}};
  int ret = connect(sfd, (sockaddr *)&server_addr, sizeof(server_addr));
  char sendbuf[1] = {0};
  send(sfd, sendbuf, 1, 0);
  LOGW("Connected to remote");
  assert(ret == 0);
  capnp::PackedFdMessageReader message(sfd);
  cereal::Thumbnail::Reader frame = message.getRoot<cereal::Thumbnail>();
  LOGW("Got frame");
  cl_int err = 0;
  cl_mem cam_buf_cl = clCreateBuffer(
      ctx, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, width * height * 3,
      (void *)frame.getThumbnail().begin(), &err);
  assert(err == CL_SUCCESS);
  clSetKernelArg(kernel, 0, sizeof(cl_mem), &cam_buf_cl);
  clSetKernelArg(kernel, 1, sizeof(cl_mem), &output_cl);
  size_t global_size[] = {static_cast<size_t>(width / 4),
                          static_cast<size_t>(height / 4)};
  cl_event event;
  clFinish(queue);
  err = clEnqueueNDRangeKernel(queue, kernel, 2, NULL, global_size, NULL, 0,
                               NULL, &event);
  assert(err == CL_SUCCESS);
  clWaitForEvents(1, &event);
  assert(clReleaseEvent(event) == CL_SUCCESS);
  LOGW("Converted");
  clFinish(queue);
  clEnqueueReadBuffer(queue, output_cl, CL_TRUE, 0, width * height * 3 / 2,
                      this->last_frame.data(), 0, NULL, NULL);
  clFinish(queue);
  clReleaseMemObject(cam_buf_cl);
  close(sfd);
  this->recv_frame = true;
  LOGW("Saved frame");
};

void RemoteCamera::fetch_frame_thread() {
  while (!do_exit) {
    this->fetch_frame();
  }
};

void RemoteCamera::init() {
  assert(this->vipc_server != nullptr);
  this->vipc_server->create_buffers(VisionStreamType::VISION_STREAM_ROAD, 5,
                                    false, this->width, this->height);
  this->vipc_server->start_listener();
  this->pm = new PubMaster({"roadCameraState"});
}

void RemoteCamera::run() {
  this->fetch_thread = std::thread(&RemoteCamera::fetch_frame_thread, this);
  this->init();
  while (!do_exit) {
    if (!this->recv_frame) {
      sleep(1);
      continue;
    }
    auto buf =
        this->vipc_server->get_buffer(VisionStreamType::VISION_STREAM_ROAD);
    clFinish(queue);
    clEnqueueWriteBuffer(queue, buf->buf_cl, CL_TRUE, 0,
                         this->last_frame.size(),
                         (void *)this->last_frame.data(), 0, NULL, NULL);
    uint64_t eof = static_cast<uint64_t>(this->frame_id * 0.05 * 1e9);
    VisionIpcBufExtra extra = {
        this->frame_id,
        eof,
        eof,
    };
    buf->set_frame_id(this->frame_id);
    this->vipc_server->send(buf, &extra);

    MessageBuilder msg;
    auto framed = msg.initEvent().initRoadCameraState();
    framed.setFrameId(this->frame_id);
    framed.setTimestampEof(eof);
    framed.setSensor(2);

    this->pm->send("roadCameraState", msg);
    this->frame_id++;
    usleep(100000);
  }
  this->fetch_thread.join();
};

void remote_camerad_thread() {
  LOGW("Running remote camera thread\n");
  cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
#ifdef QCOM2
  const cl_context_properties props[] = {CL_CONTEXT_PRIORITY_HINT_QCOM,
                                         CL_PRIORITY_HINT_HIGH_QCOM, 0};
  cl_context context =
      CL_CHECK_ERR(clCreateContext(props, 1, &device_id, NULL, NULL, &err));
#else
  cl_context context =
      CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));
#endif
  VisionIpcServer vipc_server("camerad", device_id, context);
  RemoteCamera cam("192.168.100.100", 4069, 1080, 1920, context, device_id,
                   &vipc_server);
  cam.run();
};
