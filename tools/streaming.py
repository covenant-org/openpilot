import m3u8
import requests
import cv2


def play_mpeg_ts(file_path):
    """
    Play an MPEG-TS video file using OpenCV.

    Args:
        file_path (str): Path to the MPEG-TS file.
    """
    # Open the video file
    video_capture = cv2.VideoCapture(file_path)

    if not video_capture.isOpened():
        print(f"Error: Unable to open the file {file_path}")
        return

    print(f"Playing video: {file_path}")

    while True:
        # Read frame-by-frame
        ret, frame = video_capture.read()

        # If no frame is read, end of video is reached
        if not ret:
            print("End of video or error reading the file.")
            break

        # Display the frame
        cv2.imshow("MPEG-TS Player", frame)

        # Break the loop on pressing 'q'
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break


class RequestsClient():
    def __init__(self):
      self.token = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJleHAiOjE3NDEyMDIyNTgsIm5iZiI6MTczMzQyNjI1OCwiaWF0IjoxNzMzNDI2MjU4LCJpZGVudGl0eSI6IjkxMDI5MDYwMTdkZmQ5MTIifQ.5bSBOb1OTyqKVmdrRJBr-YOn_i84Y-K_VzFsCWMhlFo'

    def download(self, uri, timeout=None, headers={}, verify_ssl=True):
        headers['Authorization'] = 'JWT ' + self.token
        o = requests.get(uri, timeout=timeout, headers=headers)
        return o.text, o.url

last_uri = None
while True:
  playlist = m3u8.load('https://api.commadotai.com/v1/route/1b163de4ada100b0|00000074--eb9b1577dd/qcamera.m3u8', http_client=RequestsClient())
  if len(playlist.segments) == 0:
    sleep(10)
    continue
  uri = playlist.segments[-1].uri
  if last_uri == uri:
    sleep(10)
    continue
  play_mpeg_ts(uri)
  last_uri = uri


