#! /bin/usr/env bash
TAG="Pointer on real video"

echo "INFO: Running pose tracking on Video"
python rt_zed_cam.py \
  --cam_source 0 \
  --mesh BaselinePointer.obj \
  --cam_file zed_1080p_raw_calib.json \
  --fps 15


echo "INFO: Calculating inference metrics"
GT_FILE=$(ls -t inputs/experiment_*/gt_poses.txt | head -1)
EST_FILE=$(ls -t outputs/pose_track_*.txt | head -1)
echo "GT & EST files used : $GT_FILE $EST_FILE"
python metrics_postprocess.py \
  --gt_file "$GT_FILE" \
  --est_file "$EST_FILE" \
  --output-dir error_outputs \
  --beta 0.1 \
  --angle-unit "deg" \
  --normal-axis z \
  --tag "$TAG_$(date +%s)"