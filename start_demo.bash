#!/bin/usr/env bash
################################################################ Optimized ##############################################################
# This is single marker demo

echo "INFO: Running pose tracking live and sending pose on port 5005"
echo "INFO: Check with utils/check_udp.py"

python -m src.single.rt_live \
  --cam_source 0 \
  --mesh models/PointerActual.obj \
  --cam_file configs/zed_1080p_raw_calib.json \
  --udp-port 5005


################################################################ Slightly optimized ##############################################################
# echo "INFO: Running pose tracking live on zed cam"

# python -m src.single.rt_zed_cam \
#   --cam_source 2 \
#   --mesh models/PointerActual.obj \
#   --cam_file configs/zed_1080p_raw_calib.json \
