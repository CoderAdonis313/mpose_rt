#! /bin/usr/env bash
TAG="Pointer on real video"

echo "INFO: Running pose tracking on Video"
python rt_live.py \
  --project-root /home/abhi/dev/mpose_rt \
  --cam_source 0 \
  --mesh PointerActual.obj \
  --cam_file zed_1080p_raw_calib.json \
  --udp-port 5005