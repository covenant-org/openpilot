#!/usr/bin/bash

export MAVLINK_CONNECTION_URI=udp://:14540
export REMOTE_CAMERA=${1:-"192.168.100.100:4069"}
export BLOCK=encoderd,stream_encoderd,dmonitoringd,dmonitoringmodeld
export DRONE_HEIGHT=${2:-"1.0"}
export USE_LIVE_CALIBRATION=1
#export USE_AUTOMATIC_CALIBRATION=1
exec ./launch_chffrplus.sh
