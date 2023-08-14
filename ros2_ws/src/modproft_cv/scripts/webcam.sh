#!/bin/bash
gphoto2 --auto-detect &
gphoto2 --stdout --capture-movie | ffmpeg -i - -vcodec rawvideo -pix_fmt yuv420p -s 1280x720 -threads 5 -f v4l2 /dev/video2 &



