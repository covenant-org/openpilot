import numpy as np
import os
import pyopencl as cl
import pyopencl.array as cl_array
import socket

from msgq.visionipc import VisionIpcServer, VisionStreamType
from cereal import messaging, log
from openpilot.common.realtime import Ratekeeper
from threading import Thread
from time import sleep
from PIL import Image

from openpilot.common.basedir import BASEDIR
W, H = 1928, 1208
MAX_BUFFER_SIZE=10*1024*1024

def nv12_to_rgb(nv12: bytes | bytearray, size: tuple[int, int]) -> Image:
  w, h = size
  n = w * h
  y, u, v = nv12[:n], nv12[n + 0::2], nv12[n + 1::2]
  yuv = bytearray(3 * n)
  yuv[0::3] = y
  yuv[1::3] = Image.frombytes('L', (w // 2, h // 2), u).resize(size).tobytes()
  yuv[2::3] = Image.frombytes('L', (w // 2, h // 2), v).resize(size).tobytes()
  return Image.frombuffer('YCbCr', size, yuv).convert('RGB')

class GZCamerad:
  """Simulates the camerad daemon"""
  def __init__(self, dual_camera):
    self.pm = messaging.PubMaster(['roadCameraState', 'wideRoadCameraState'])

    self.frame_road_id = 0
    self.frame_wide_id = 0
    self.last_frame=None
    self.vipc_server = VisionIpcServer("camerad")

    self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 5, False, W, H)
    if dual_camera:
      self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, False, W, H)

    self.vipc_server.start_listener()

    # set up for pyopencl rgb to yuv conversion
    self.ctx = cl.create_some_context()
    self.queue = cl.CommandQueue(self.ctx)
    cl_arg = f" -DHEIGHT={H} -DWIDTH={W} -DRGB_STRIDE={W * 3} -DUV_WIDTH={W // 2} -DUV_HEIGHT={H // 2} -DRGB_SIZE={W * H} -DCL_DEBUG "

    kernel_fn = os.path.join(BASEDIR, "tools/rgb_to_nv12.cl")
    with open(kernel_fn) as f:
      prg = cl.Program(self.ctx, f.read()).build(cl_arg)
      self.krnl = prg.rgb_to_nv12
    self.Wdiv4 = W // 4 if (W % 4 == 0) else (W + (4 - W % 4)) // 4
    self.Hdiv4 = H // 4 if (H % 4 == 0) else (H + (4 - H % 4)) // 4

  def cam_send_yuv_road(self, yuv):
    self._send_yuv(yuv, self.frame_road_id, 'roadCameraState', VisionStreamType.VISION_STREAM_ROAD)
    self.frame_road_id += 1

  def cam_send_yuv_wide_road(self, yuv):
    self._send_yuv(yuv, self.frame_wide_id, 'wideRoadCameraState', VisionStreamType.VISION_STREAM_WIDE_ROAD)
    self.frame_wide_id += 1

  # Returns: yuv bytes
  def rgb_to_yuv(self, rgb):
    assert rgb.shape == (H, W, 3), f"{rgb.shape}"
    assert rgb.dtype == np.uint8

    rgb_cl = cl_array.to_device(self.queue, rgb)
    yuv_cl = cl_array.empty_like(rgb_cl)
    self.krnl(self.queue, (self.Wdiv4, self.Hdiv4), None, rgb_cl.data, yuv_cl.data).wait()
    yuv = np.resize(yuv_cl.get(), rgb.size // 2)
    return yuv.data.tobytes()

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    eof = int(frame_id * 0.05 * 1e9)
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)

    dat = messaging.new_message(pub_type, valid=True)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0],
      "sensor": 2
    }
    setattr(dat, pub_type, msg)
    self.pm.send(pub_type, dat)

  def get_frame(self):
    data = bytearray(MAX_BUFFER_SIZE)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(("192.168.100.100", 4069))
        print("Connected to remote")
        s.send(bytes([0]))
        offset = 0
        while offset < len(data):
            recv = s.recv(len(data) - offset)
            data[offset:offset+len(recv)] = recv
            offset += len(recv)
            if len(recv) == 0:
                break
        print("Received frame")

    thumbnail = log.Thumbnail.from_bytes_packed(data)
    original = np.frombuffer(thumbnail.thumbnail, dtype=np.uint8).reshape((1080, 1920, 3))
    frame = np.pad(original, [(0, H - 1080), (0, W - 1920), (0, 0)], mode='constant', constant_values=0)
    self.last_frame = self.rgb_to_yuv(frame)
    return frame

  def get_frame_thread(self):
    while True:
      self.get_frame()

  def run(self):
      rk = Ratekeeper(100)
      while True:
        if self.last_frame is None:
          sleep(3)
          continue
        self.cam_send_yuv_road(self.last_frame)
        rk.monitor_time()

def main():
    camerad = GZCamerad(dual_camera=False)
    thread = Thread(target=camerad.get_frame_thread)
    thread.start()
    camerad.run()

if __name__ == "__main__":
    main()

