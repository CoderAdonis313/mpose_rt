#!/bin/usr/env bash
################################################################ Slightly optimized ##############################################################
echo "INFO: Running multi megapose and multi contour with threads"
python -m src.utils.test_multi_mpose \
  --cam_source 2 \
  --cam_file configs/zed_1080p_raw_calib.json \
