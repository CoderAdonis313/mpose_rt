#! /bin/usr/env bash
TAG="Pointer on simulation video"


echo "INFO: Running pose tracking megapose"
python rt_video.py \
--video_file output_1789260512.mp4 \
--mesh BaselinePointer.obj \
--cam_file zed_1080p_calib.json


echo "INFO: Calculating inference metrics"
gt_file=(inputs/*.txt)
est_file=(outputs/*.txt)
python metrics_postprocess.py \
  --gt_file "$gt_file" \
  --est_file "$est_file" \
  --output-dir error_outputs \
  --beta 0.1 \
  --angle-unit "deg" \
  --normal-axis z \
  --tag "$TAG"