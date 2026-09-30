# Real-time inference scripts

The `src/single/rt_*` programs run single-object MegaPose inference from either a camera or recorded video.

Run them from the repository root using module syntax:

```bash
python -m src.single.rt_video --help
```

## Which script should I use?

| Script | Input | Purpose | Outputs |
|---|---|---|---|
| `rt_cam.py` | Generic OpenCV camera | Basic live-camera inference | Annotated MP4 and pose TXT |
| `rt_video.py` | Recorded video | Repeatable offline inference | Annotated MP4 and pose TXT |
| `rt_zed_cam.py` | ZED UVC stereo stream | Standalone ZED live inference | Annotated MP4 and pose CSV |
| `rt_live.py` | ZED UVC stereo stream | Live inference with UDP publishing | MP4, CSV, and UDP packets |

## `rt_cam.py`

Opens a numeric camera ID with OpenCV, detects one colored marker, runs MegaPose, and records pose estimates.

Use it for a simple live-camera test that does not need ZED-specific stereo extraction or UDP.

## `rt_video.py`

Runs the single-marker inference pipeline on a recorded video.

This is the best script for repeatable debugging and comparison because every run processes the same frames. Lower `--megapose-batch-size` if CUDA runs out of memory.

## `rt_zed_cam.py`

Opens the ZED through its OpenCV/V4L2 stereo interface, validates the side-by-side frame size, extracts the left image, and runs MegaPose.

It does not require the ZED Python SDK. Results are written under `outputs/` as:

- `zed_pose_*.mp4`
- `zed_pose_*.csv`

## `rt_live.py`

Extends the ZED live pipeline with non-blocking JSON/UDP publishing to `127.0.0.1`.

It sends the full object-to-camera transform, frame timestamps, and tracking status to the ROS-side bridge. Use this script for the live robot demo.

UDP packets can be inspected without ROS using:

```bash
python -m src.utils.check_udp
```

## Shared arguments

- `--mesh`: object mesh, normally under `models/`
- `--cam_file`: camera calibration, normally under `configs/`
- `--model`: MegaPose model; default is `megapose-1.0-RGB`
- `--out_dir`: output directory where supported

Run all scripts inside the `mpose` Conda environment.

The ZED OpenCV scripts consume raw UVC images and do not currently undistort frames before inference. The chosen calibration must remain consistent with this image pipeline.
