#! /bin/usr/env bash
TAG="Pointer on real video"

VID_FILE=$(ls -t inputs/experiment_*/video.mp4 | head -1)
echo "INFO: Running pose tracking on Video"
echo "Video file used : $VID_FILE"
python rt_video.py \
  --mesh PointerActual.obj \
  --cam_file zed_1080p_calib.json \
  --video_file $VID_FILE \


# echo "INFO: Running pose tracking on camera"
# python rt_cam.py \
#   --mesh BaselinePointer.obj \
#   --cam_source "http://192.168.1.107:8080/video" \
#   --cam_file samsung1920_calib.json \


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