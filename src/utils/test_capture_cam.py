#!/usr/bin/env python3
"""
Run with the pose_detector Python environment. No ROS or ZED SDK imports.
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
import numpy as np
from configs.config import *
# Load imports
import torch
from configs.config import COLOR_RANGES, OUT_RES
from src.single.contour_runner import ContourRunner
from src.single.mpose_runner import MegaPoseRunner
from src.single.pose_worker import LatestPoseInference
from src.single.cam_worker import LatestFrameCamera

import cv2
cv2.ocl.setUseOpenCL(False)
cv2.setNumThreads(1)


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


def extract_left(frame, size):
    width, height = size
    expected = (height, width * 2, 3)

    if frame is None or frame.shape != expected or frame.dtype != np.uint8:
        actual = None if frame is None else frame.shape
        raise RuntimeError(
            f"Expected ZED stereo frame {expected}; got {actual}."
        )
    return frame[:, :width].copy()


def draw_triaxis(image_bgr, camera_matrix, pose_matrix, axis_length=0.1,):
    rotation = pose_matrix[:3, :3].astype(float)
    translation = pose_matrix[:3, 3].astype(float)

    rotation_vector, _ = cv2.Rodrigues(rotation)
    translation_vector = translation.reshape(3, 1)
    distortion = np.zeros((5, 1), dtype=float)

    cv2.drawFrameAxes(
        image_bgr,
        camera_matrix,
        distortion,
        rotation_vector,
        translation_vector,
        axis_length,
    )


def main():
    args = parse_args()
    # This script can live in ros2_ws while reusing the inference project.
    sys.path.insert(0, str(PROJECT_ROOT))

    mesh_path = PROJECT_ROOT / args.mesh
    camera_path = PROJECT_ROOT / args.cam_file
    size = tuple(json.loads(camera_path.read_text())["img_size"])

    if size != tuple(OUT_RES):
        raise ValueError(f"Calibration {size} must match OUT_RES={OUT_RES}.")

    width, height = size
    cam_worker = pose_worker = None
    window_created = False
    frame_index = 0
    processed = 0
    POSE_HOLD_NS = 300_000_000
    last_frame_idx = -1
    started = perf_counter()

    try:
        pose_detector = MegaPoseRunner(
            mesh_path,
            "fiducial",
            args.model,
            camera_path,
            batch_size=MPOSE_BATCH_SIZE,
            n_workers=N_WORKERS
        )
        bbox_detector = ContourRunner(COLOR_RANGES)
        camera_matrix = np.asarray(pose_detector.K, dtype=float).copy()

        pose_worker = LatestPoseInference(pose_detector, iou_threshold=IOU)
        cam_worker = LatestFrameCamera(
            source=args.cam_source,
            frame_size=(width * 2, height),
            backend=cv2.CAP_V4L2,
            fps=FPS
        )

        cam_worker.start()
        pose_worker.start()

        cv2.namedWindow("ZED MegaPose", cv2.WINDOW_NORMAL)
        window_created = True
        cv2.resizeWindow("ZED MegaPose", 1280, 720)

        print("q/Escape: quit; r: reset tracking.")
        print("Timestamps are host read times, not hardware exposure times.")
        print(f"Recorded MP4 playback uses fixed {FPS} FPS.")

        previous_time = perf_counter()

        last_frame_idx = -1
        latest_left_bgr = None
        latest_bbox = None

        while True:
            # This never waits for the camera thread.
            frame_data = cam_worker.latest()

            if (
                frame_data is not None
                and frame_data.index != last_frame_idx
            ):
                last_frame_idx = frame_data.index

                latest_left_bgr = extract_left(
                    frame_data.image,
                    size,
                )

                img_rgb = cv2.cvtColor(
                    latest_left_bgr,
                    cv2.COLOR_BGR2RGB,
                )

                _, bboxs = bbox_detector.estimate(
                    img_rgb,
                )

                latest_bbox = None

                if bboxs is not None and len(bboxs) > 0:
                    latest_bbox = np.asarray(
                        bboxs[0],
                        dtype=float,
                    )

                # This only replaces the pending job; it does not wait for MegaPose.
                pose_worker.submit(
                    frame_index=frame_data.index,
                    image_time_ns=frame_data.image_time_ns,
                    image_rgb=img_rgb,
                    bbox=latest_bbox,
                )

                processed += 1

            if latest_left_bgr is not None:
                display = latest_left_bgr.copy()

                if latest_bbox is not None:
                    x1, y1, x2, y2 = map(
                        int,
                        latest_bbox,
                    )

                    cv2.rectangle(
                        display,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2,
                    )

                # This also returns immediately.
                pose_result = pose_worker.latest()

                pose_is_fresh = (
                    pose_result is not None
                    and pose_result.pose_matrix is not None
                    and time_ns() - pose_result.result_time_ns
                    <= POSE_HOLD_NS
                )

                if pose_is_fresh:
                    draw_triaxis(
                        display,
                        camera_matrix,
                        pose_result.pose_matrix,    #type: ignore
                    )

                status = (
                    pose_result.status
                    if pose_result is not None
                    else "POSE_PENDING"
                )

                cv2.putText(
                    display,
                    status,
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                cv2.imshow(
                    "ZED MegaPose",
                    display,
                )

            # GUI events are now processed even when no new camera frame arrives.
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord("r"):
                pose_worker.request_reset()

            if cv2.getWindowProperty(
                "ZED MegaPose",
                cv2.WND_PROP_VISIBLE,
            ) < 1:
                break

    except KeyboardInterrupt:
        print("\nStopped by keyboard")

    except Exception as e:
        print('ERROR: ', e)
        raise
    
    finally:
        if pose_worker is not None:
            pose_worker.close()
        if cam_worker is not None:
            cam_worker.close()
        if window_created:
            cv2.destroyAllWindows()
        print(f"Processed {processed} frames in {perf_counter() - started:.1f}s.")


if __name__ == "__main__":
    main()