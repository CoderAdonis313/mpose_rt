#!/usr/bin/env python3
"""
Run with the mpose Python environment. No ROS or ZED SDK imports.
Images are raw left-camera frames; no undistortion is performed.
"""

import csv
import json
import sys
from argparse import ArgumentParser
from datetime import datetime
from pathlib import Path
from time import perf_counter, time_ns
from uuid import uuid4
# import cv2
import numpy as np
from configs.config import *
# Load imports
import torch
from configs.config import COLOR_RANGES, OUT_RES
from src.single.contour_runner import ContourRunner
from src.multi.mpose_runner_multi import MegaPoseRunnerMulti
from src.multi.capture_cam import LatestFrameCamera


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def parse_args():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--cam_source", default="0", type=int)
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--cam_file", required=True)
    parser.add_argument("--model", default="megapose-1.0-RGB")
    parser.add_argument("--frame-id", default="zed_left_camera_optical_frame")
    parser.add_argument("--out_dir", default='outputs')

    args = parser.parse_args()

    if FPS <= 0 or MPOSE_BATCH_SIZE <= 0:
        parser.error("FPS and batch size must be positive.")
    if not 0 <= IOU <= 1:
        parser.error("IoU threshold must be between 0 and 1.")
    return args


def calculate_iou(a, b):
    if a is None or b is None:
        return 0.0

    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    if a.shape != (4,) or b.shape != (4,):
        return 0.0
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        return 0.0

    intersection = np.maximum(
        0.0,
        np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2]),
    )
    overlap = float(np.prod(intersection))
    area_a = float(np.prod(np.maximum(0.0, a[2:] - a[:2])))
    area_b = float(np.prod(np.maximum(0.0, b[2:] - b[:2])))
    union = area_a + area_b - overlap
    return overlap / union if union > 0 else 0.0


def extract_left(frame, size):
    width, height = size
    expected = (height, width * 2, 3)

    if frame is None or frame.shape != expected or frame.dtype != np.uint8:
        actual = None if frame is None else frame.shape
        raise RuntimeError(
            f"Expected ZED stereo frame {expected}; got {actual}."
        )
    return frame[:, :width].copy()


def process_frame(mpose, detector, left_bgr, threshold):
    rgb = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2RGB)
    _, boxes = detector.estimate(rgb)

    if boxes is None or len(boxes) == 0:
        mpose.reset_tracking()
        return left_bgr.copy(), None, "NO_TARGET"

    bbox = boxes[0]

    if mpose.tracking_active:
        try:
            predicted = mpose.mpose_bboxes()
        except ValueError:
            predicted = []

        if calculate_iou(bbox, predicted) < threshold:
            mpose.reset_tracking()

    if not mpose.tracking_active:
        mpose.load_detection(bbox)

    pose = mpose.estimate(rgb)

    if pose is None:
        mpose.reset_tracking()
        display, status = left_bgr.copy(), "NO_POSE"
    else:
        display, status = mpose.draw_triaxis(), "TRACKING"

    x1, y1, x2, y2 = map(int, bbox)
    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return display, pose, status


def main():
    args = parse_args()

    # This script can live in ros2_ws while reusing the inference project.
    sys.path.insert(0, str(PROJECT_ROOT))


    mesh_path = PROJECT_ROOT / args.mesh
    camera_path = PROJECT_ROOT / args.cam_file
    size = tuple(json.loads(camera_path.read_text())["img_size"])

    if size != tuple(OUT_RES):
        raise ValueError(
            f"Calibration {size} must match OUT_RES={OUT_RES}."
        )

    width, height = size

    cap = video = pose_file = None
    window_created = False
    frame_index = 0
    processed = 0
    started = perf_counter()

    try:
        mpose = MegaPoseRunnerMulti(
            mesh_path,
            "fiducial",
            args.model,
            camera_path,
            batch_size=MPOSE_BATCH_SIZE,
            n_workers=N_WORKERS
        )
        detector = ContourRunner(COLOR_RANGES)

        cap = LatestFrameCamera(
            source=args.cam_source,
            frame_size=(width * 2, height),
            backend=cv2.CAP_V4L2,
            fps=FPS
        )

        cap.start()

        cv2.namedWindow("ZED MegaPose", cv2.WINDOW_NORMAL)
        window_created = True
        cv2.resizeWindow("ZED MegaPose", 1280, 720)

        print("q/Escape: quit; r: reset tracking.")
        print("Timestamps are host read times, not hardware exposure times.")
        print(f"Recorded MP4 playback uses fixed {FPS} FPS.")

        previous_time = perf_counter()

        while True:
            frame_data = cap.read_latest()
            image_time_ns = frame_data.image_time_ns
            frame = frame_data.image

            left_bgr = extract_left(frame, size)
            inference_start = perf_counter()

            try:
                display, pose, status = process_frame(
                    mpose,
                    detector,
                    left_bgr,
                    IOU,
                )
            except torch.cuda.OutOfMemoryError:
                mpose.reset_tracking()
                torch.cuda.empty_cache()
                raise RuntimeError(
                    "CUDA out of memory; retry --megapose-batch-size 4."
                ) from None

            inference_seconds = perf_counter() - inference_start
            result_time_ns = time_ns()

            # Never reuse a previous pose when this frame has no estimate.
            transform = mpose.poses if pose is not None else None
            print('POSE', transform)

            now = perf_counter()
            loop_fps = 1.0 / max(now - previous_time, 1e-6)
            previous_time = now

            lines = [f"{status} | processed FPS: {loop_fps:.1f}"]
            if pose is not None:
                lines.append(
                    f"x={pose[0]:.3f} y={pose[1]:.3f} "
                    f"z={pose[2]:.3f} m"
                )

            for index, line in enumerate(lines):
                cv2.putText(
                    display,
                    line,
                    (20, 35 + 30 * index),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

            cv2.imshow("ZED MegaPose", display)
            processed += 1
            frame_index += 1

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                mpose.reset_tracking()
            if cv2.getWindowProperty(
                "ZED MegaPose", cv2.WND_PROP_VISIBLE
            ) < 1:
                break

    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print('ERROR: ', e)
        raise
    finally:
        if cap is not None:
            cap.close()
    
        if window_created:
            cv2.destroyAllWindows()

        print(
            f"Processed {processed} frames "
            f"in {perf_counter() - started:.1f}s."
        )


if __name__ == "__main__":
    main()