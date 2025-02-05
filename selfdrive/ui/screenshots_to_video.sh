#!/usr/bin/bash

ffmpeg -framerate 3 -pattern_type glob -i ${1:-"./*.png"} -c:v libx264 ${2:-"output.mp4"}

if [ $? -ne 0 ]; then
    echo "Failed to generate video using libx264, trying mpeg4"
    ffmpeg -framerate 3 -pattern_type glob -i ${1:-"./*.png"} -c:v mpeg4 ${2:-"output.mp4"}
fi
