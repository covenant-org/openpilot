#!/usr/bin/bash

export MAVLINK_CONNECTION_URI=udp://:14540
export REMOTE_CAMERA=${1:-"192.168.100.100:4069"}
export BLOCK=encoderd,stream_encoderd
export DRONE_HEIGHT=${2:-"0.8"}
exec ./launch_chffrplus.sh
