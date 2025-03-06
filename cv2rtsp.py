import numpy as np
import cv2
import time
import subprocess
from datetime import datetime
from PIL import Image


def nv12_to_I420(nv12_frame, width, height):
    """
    Convert NV12 frame to BGR format using OpenCV.
    Assumes the frame is flattened in a numpy array.
    """
    y_size = width * height
    uv_size = y_size // 2

    y_plane = nv12_frame[:y_size].reshape((height, width))
    uv_plane = nv12_frame[y_size:y_size +
                          uv_size].reshape((height // 2, width // 2, 2))

    u_plane = uv_plane[:, :, 0]
    v_plane = uv_plane[:, :, 1]

    # Merge Y, U, V into a BGR image
    return cv2.merge([y_plane, u_plane, v_plane])


def stream_nv12_to_rtsp(nv12_frames, rtsp_url, width, height, fps=30):
    """
    Stream NV12 frames to an RTSP server using OpenCV.
    nv12_frames: Flattened NV12 frames array.
    rtsp_url: URL of the RTSP server.
    width, height: Dimensions of the video.
    fps: Frames per second of the video.
    """
    # Initialize RTSP stream using OpenCV
    fourcc = cv2.VideoWriter_fourcc(*'H264')  # H264 encoding
    out = cv2.VideoWriter(rtsp_url, fourcc, fps, (width, height))

    for frame in nv12_frames:
        bgr_frame = nv12_to_I420(frame, width, height)
        out.write(bgr_frame)
        time.sleep(1 / fps)  # Control the frame rate

    out.release()


def open_ffmpeg_stream_process(width, height, fps):
    args = (
        "ffmpeg -re -stream_loop -1 -f rawvideo -pix_fmt "
        f"rgb24 -s {width}x{height} -r {fps} -i pipe:0 -pix_fmt yuv420p "
        "-f rtsp rtsp://159.54.131.60:8554/stream"
    ).split()
    return subprocess.Popen(args, stdin=subprocess.PIPE)


# input is a RGB numpy array with shape (height,width,3), can be uint,int, float or double, values expected in the range 0..255
# output is a double YUV numpy array with shape (height,width,3), values in the range 0..255
def RGB2YUV(rgb):

    m = np.array([
        [0.29900, -0.147108,  0.614777],
        [0.58700, -0.288804, -0.514799],
        [0.11400,  0.435912, -0.099978]
    ])

    yuv = np.dot(rgb, m)
    yuv[:, :, 1:] += 0.5
    return yuv


if __name__ == "__main__":
    # Example parameters
    image = Image.open("/home/kevinm/Downloads/cat.jpg")
    image2 = Image.open("/home/kevinm/Downloads/cat2.webp")
#    cv2.imshow("image", frame_raw)
#    cv2.waitKey(0)
#    exit()
    width = 1280
    height = 720
    image = image.resize((width, height))
    image2 = image2.resize((width, height))
    frame_raw = np.asarray(image)
    frame_raw2 = np.asarray(image2)
    converted = cv2.cvtColor(frame_raw, cv2.COLOR_BGR2RGB)
    converted = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2YUV_I420)
    converted2 = cv2.cvtColor(frame_raw2, cv2.COLOR_BGR2RGB)
    converted2 = cv2.cvtColor(frame_raw2, cv2.COLOR_RGB2YUV_I420)
    fps = 20
    ffmpeg_process = open_ffmpeg_stream_process(width, height, fps)
    frameidx = 0

    start = time.time()
    while True:
        # Example: Create a dummy NV12 frame for testing
        # This should come from your actual flattened NV12 frame data
        # Create random NV12 data (flattened array)
        #      nv12_frame = np.random.randint(0, 255, width * height * 3 // 2, dtype=np.uint8)

        #      frame = nv12_to_I420(nv12_frame, width, height)
        if frameidx // 100 % 2 == 0:
          ffmpeg_process.stdin.write(frame_raw.tobytes())
        else:
          ffmpeg_process.stdin.write(frame_raw2.tobytes())

        now = time.time()
        diff = (1 / fps) - now - start
        if diff > 0:
            time.sleep(diff)
        start = now
        frameidx += 1
