#! /bin/usr/env bash
TAG="Pointer on simulation video"


# echo "INFO: Running pose tracking on Video"
# python rt_video.py \
#   --mesh BaselinePointer.obj \
#   --cam_file zed_1080p_calib.json \
#   --video_file output_1789260512.mp4 \


echo "INFO: Running pose tracking on camera"
python rt_zed_cam.py \
  --mesh BaselinePointer.obj \
  --cam_file zed_1080p_calib.json \


echo "INFO: Calculating inference metrics"
GT_FILE="inputs/gt_poses_19_07_04.txt"
EST_FILE="outputs/pose_track_1789260645.txt"
python metrics_postprocess.py \
  --gt_file "$GT_FILE" \
  --est_file "$EST_FILE" \
  --output-dir error_outputs \
  --beta 0.1 \
  --angle-unit "deg" \
  --normal-axis z \
  --tag "$TAG"