#! /bin/usr/env bash
################################################################ Not optimized ##############################################################
TAG="Tracking on real video"

echo "INFO: Running pose tracking on Video"
python -m src.single.rt_video \
  --video_file inputs/pose_track_1790801207.mp4 \
  --mesh models/BaselinePointer.obj \
  --cam_file configs/zed_1080p_raw_calib.json \


echo "INFO: Calculating inference metrics"
GT_FILE=$(ls -t inputs/gt_poses_*.txt | head -1)
EST_FILE=$(ls -t outputs/pose_track_*.txt | head -1)
echo "GT & EST files used : $GT_FILE $EST_FILE"
python -m src.single.metrics_postprocess \
  --gt_file "$GT_FILE" \
  --est_file "$EST_FILE" \
  --output-dir error_outputs \
  --beta 0.1 \
  --angle-unit "deg" \
  --normal-axis z \
  --tag "$TAG_$(date +%s)"


################################################################ Not optimized ##############################################################
# TAG="Tracking on real camera"

# echo "INFO: Running pose tracking on Video"
# python -m src.single.rt_cam \
#   --cam_source 2 \
#   --mesh models/BaselinePointer.obj \
#   --cam_file configs/zed_1080p_raw_calib.json \


# echo "INFO: Calculating inference metrics"
# GT_FILE=$(ls -t inputs/gt_poses_*.txt | head -1)
# EST_FILE=$(ls -t outputs/pose_track_*.txt | head -1)
# echo "GT & EST files used : $GT_FILE $EST_FILE"
# python -m src.single.metrics_postprocess \
#   --gt_file "$GT_FILE" \
#   --est_file "$EST_FILE" \
#   --output-dir error_outputs \
#   --beta 0.1 \
#   --angle-unit "deg" \
#   --normal-axis z \
#   --tag "$TAG_$(date +%s)"