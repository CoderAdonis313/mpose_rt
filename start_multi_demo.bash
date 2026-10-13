#!/usr/bin/env bash
################################################################ Slightly optimized ##############################################################
echo "INFO: Running multi megapose and multi contour with threads"
python -m src.multi.rt_multi \
  --cam_id 2 \
  --cam_file configs/see_cam_1080p_raw_calib.json \
  # --stereo
  # set res for single image you want even in stereo mode