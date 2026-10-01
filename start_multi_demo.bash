#!/bin/usr/env bash
################################################################ Optimized ##############################################################
# This is single marker demo

echo "INFO: Running pose tracking live and with camera thread"
echo "INFO: Check with utils/check_udp.py"

python -m src.utils.test_capture_cam \
  --cam_source 0 \
  --mesh models/PointerActual.obj \
  --cam_file configs/zed_1080p_raw_calib.json \


################################################################ Slightly optimized ##############################################################
# echo "INFO: Running pose tracking live on zed cam"

# python -m src.single.rt_zed_cam \
#   --cam_source 2 \
#   --mesh models/PointerActual.obj \
#   --cam_file configs/zed_1080p_raw_calib.json \
