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
#include <string>
#include "zlib.h"

#define ZLCHUNK 16384

extern ExitHandler do_exit;

RemoteCamera::RemoteCamera(std::string addr, uint16_t port, uint16_t height,
                           uint16_t width, cl_context ctx,
                           cl_device_id device_id, VisionIpcServer *vipc_server)
    : ip(addr), port(port), height(height), width(width),
      vipc_server(vipc_server), last_frame(height * width * 3 / 2, ' ') {
  cl_int err = 0;
  this->queue = clCreateCommandQueueWithProperties(ctx, device_id, 0, &err);
  assert(err == CL_SUCCESS);
}

RemoteCamera::~RemoteCamera() {
  clReleaseCommandQueue(queue);
  if (this->pm != nullptr) {
    delete this->pm;
  }
}

int inf(int input_fd, unsigned char *output){
  int ret;
  unsigned output_offset = 0;
  z_stream strm;
  unsigned char in[ZLCHUNK];

  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  strm.avail_in = 0;
  strm.next_in = Z_NULL;
  ret = inflateInit(&strm);
  if(ret != Z_OK){
    return -1;
  }
  do {
    size_t bytes_read = read(input_fd, in, ZLCHUNK);
    if(bytes_read<0){
      (void)inflateEnd(&strm);
      return -1;
    }
    if(bytes_read == 0){
      break;
    }
    strm.next_in = in;
    strm.avail_in = bytes_read;
    do{
      strm.avail_out = ZLCHUNK;
      strm.next_out = output + output_offset;
      ret = inflate(&strm, Z_NO_FLUSH);
      assert(ret!=Z_STREAM_ERROR);
      switch(ret){
        case Z_NEED_DICT:
          ret = Z_DATA_ERROR;
        case Z_DATA_ERROR:
        case Z_MEM_ERROR:
          (void)inflateEnd(&strm);
          return -1;
      }
      output_offset += ZLCHUNK - strm.avail_out;
    }while(strm.avail_out==0);
  }while(ret!=Z_STREAM_END);

  (void)inflateEnd(&strm);
  return ret == Z_STREAM_END ? output_offset : -1;
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
  std::string package(10*1024*1024, 0);
  int read = inf(sfd, (unsigned char *)package.data());
  assert(read >= 0);
  LOGW("Read and inflated");
  kj::ArrayInputStream input(kj::ArrayPtr<kj::byte>((unsigned char*) package.data(), read));
  capnp::PackedMessageReader message(input);
  cereal::Thumbnail::Reader frame = message.getRoot<cereal::Thumbnail>();
  LOGW("Deserialized");
  auto size = frame.getThumbnail().size();
  LOGW("Got frame %lu bytes", size);
  memcpy(this->last_frame.data(), frame.getThumbnail().begin(), size);
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
    framed.setSensor(cereal::FrameData::ImageSensor::OX03C10);

    this->pm->send("roadCameraState", msg);
    this->frame_id++;
    usleep(50000);
  }
  this->fetch_thread.join();
};

void remote_camerad_thread(std::string ip) {
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
  size_t split_port_pos = ip.find(":");
  assert(split_port_pos != std::string::npos);
  VisionIpcServer vipc_server("camerad", device_id, context);
  RemoteCamera cam(ip.substr(0, split_port_pos), atoi(ip.substr(split_port_pos + 1).c_str()), 720, 1280, context, device_id,
                   &vipc_server);
  cam.run();
};
