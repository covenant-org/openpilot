#!/usr/bin/env bash

export DRONE_HEIGHT=1.0
export MANUAL_CONTROL=1
export DISABLE_DRIVER=1
export BLOCK=dmonitoringd
exec ./launch_chffrplus.sh
