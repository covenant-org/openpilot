#!/usr/bin/env python3
import os
import time
import pickle
import numpy as np
import cereal.messaging as messaging
import socket
import pickle
from cereal import car, log
from pathlib import Path
from setproctitle import setproctitle
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process
from openpilot.common.transformations.camera import DEVICE_CAMERAS, CameraConfig
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.system import sentry
from openpilot.selfdrive.car.car_helpers import get_demo_car_params
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.modeld.runners import ModelRunner, Runtime
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.models.commonmodel_pyx import ModelFrame, CLContext
import zlib
from collections import namedtuple
from threading import Thread
from time import sleep

FrameInfo = namedtuple("FrameInfo", ["width", "height", "fps", "pixel_format"])


PROCESS_NAME = "selfdrive.stream.remote"
class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof

def send_frame(conn, frame):
    frame_raw_nv12 = frame.astype(np.uint8)
    compressed_frame = zlib.compress(frame_raw_nv12.tobytes())
    conn.sendall(bytes([1]) + int(len(compressed_frame)).to_bytes(length=4, byteorder="big") + compressed_frame)


def main(demo=False):
  setproctitle(PROCESS_NAME)
  config_realtime_process(7, 54)

  cloudlog.warning("setting up CL context")
  cl_context = CLContext()
  cloudlog.warning("CL context ready; loading model")

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      main_wide_camera = VisionStreamType.VISION_STREAM_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True, cl_context)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()

  sm = SubMaster(["roadCameraState"])
  params = Params()

  with car.CarParams.from_bytes(params.get("CarParams", block=True)) as msg:
    CP = msg

  width, height = vipc_client_main.width, vipc_client_main.height
  fps = 20
  info = FrameInfo(width=width, height=height, fps=fps, pixel_format="nv12")
  conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  conn.connect(("192.168.100.100", 4020))
  serialized_info = pickle.dumps(info)
#  print(bytes([0])+int(len(serialized_info)).to_bytes(length=4, byteorder="big") + serialized_info)
  conn.sendall(bytes([0])+int(len(serialized_info)).to_bytes(length=4, byteorder="big") + serialized_info)
  sock = messaging.sub_sock("livestreamRoadEncodeData", conflate=True)


  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame

    msg = None
    while True:
      msg = messaging.recv_one_or_none(sock)
      if msg is not None:
        break
      sleep(0.005)

    evta = getattr(msg, msg.which())
    frame = evta.header + evta.data
    conn.sendall(bytes([1]) + int(len(frame)).to_bytes(length=4, byteorder="big") + frame)

    #send_frame(conn, buf_main.data)



if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    cloudlog.warning(f"child {PROCESS_NAME} got SIGINT")
  except Exception:
    sentry.capture_exception()
    raise
