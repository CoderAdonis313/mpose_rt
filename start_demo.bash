#! /bin/usr/env bash
TAG="Pointer on real video"

echo "INFO: Running pose tracking on Video"
python rt_live.py \
  --cam_source 2 \
  --mesh PointerActual.obj \
  --cam_file zed_1080p_raw_calib.json \
  --udp-port 5005
  # --project-root /home/abhi/Dev/mpose_rt \