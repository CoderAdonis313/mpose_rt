#!/usr/bin/env python3
"""
Non-blocking multi-marker MegaPose camera test.

Camera capture and MegaPose run in worker threads. The main thread performs
contour detection, draws the latest available results, and handles the GUI.
"""

import json
from argparse import ArgumentParser
from pathlib import Path
from time import perf_counter, time_ns
import numpy as np
from configs.config_multi import *
from src.multi.cam_worker import LatestFrameCamera
from src.multi.contour_runner_multi import ContourRunnerMulti
from src.multi.mpose_runner_multi import MegaPoseRunnerMulti
from src.multi.pose_worker import LatestPoseInference

import cv2
cv2.ocl.setUseOpenCL(False)
cv2.setNumThreads(1)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WINDOW_NAME = "ZED Multi-Marker MegaPose"
POSE_HOLD_NS = 300_000_000


def parse_args():
    parser = ArgumentParser(
        description=__doc__,
    )

    parser.add_argument(
        "--cam_source",
        default=0,
        type=int,
    )

    parser.add_argument(
        "--cam_file",
        required=True,
    )

    parser.add_argument(
        "--model",
        default="megapose-1.0-RGB",
    )

    args = parser.parse_args()

    if FPS <= 0:
        parser.error("FPS must be positive")

    if MPOSE_BATCH_SIZE <= 0:
        parser.error("MegaPose batch size must be positive")

    if not 0.0 <= IOU <= 1.0:
        parser.error(
            "IoU threshold must be between zero and one"
        )

    if set(COLOR_RANGES) != set(MESH_PATHS):
        parser.error(
            "COLOR_RANGES and MESH_PATHS must contain "
            "the same marker labels"
        )

    return args


def resolve_project_path(path_value):
    path = Path(path_value)

    if path.is_absolute():
        return path

    return PROJECT_ROOT / path


def extract_left(stereo_frame, size):
    width, height = size

    expected_shape = (
        height,
        width * 2,
        3,
    )

    if (
        stereo_frame is None
        or stereo_frame.shape != expected_shape
        or stereo_frame.dtype != np.uint8
    ):
        actual_shape = (
            None
            if stereo_frame is None
            else stereo_frame.shape
        )

        raise RuntimeError(
            f"Expected ZED stereo frame {expected_shape}; "
            f"received {actual_shape}"
        )

    return stereo_frame[:, :width].copy()


def draw_triaxis(
    image_bgr,
    camera_matrix,
    pose_matrix,
    axis_length=0.1,
):
    rotation = np.asarray(
        pose_matrix[:3, :3],
        dtype=float,
    )

    translation = np.asarray(
        pose_matrix[:3, 3],
        dtype=float,
    ).reshape(3, 1)

    rotation_vector, _ = cv2.Rodrigues(
        rotation
    )

    distortion = np.zeros(
        (5, 1),
        dtype=float,
    )

    cv2.drawFrameAxes(
        image_bgr,
        camera_matrix,
        distortion,
        rotation_vector,
        translation,
        axis_length,
        2,
    )


def draw_detections_bgr(
    image_bgr,
    detections,
    detector,
):
    """Draw the current contour detections on a BGR image."""

    for item in detections:
        label = item["label"]

        x1, y1, x2, y2 = map(
            int,
            item["detection"],
        )

        # ContourRunnerMulti stores display colors for an RGB image.
        color_rgb = detector.label_colors.get(
            label,
            (255, 0, 255),
        )

        color_bgr = (
            int(color_rgb[2]),
            int(color_rgb[1]),
            int(color_rgb[0]),
        )

        cv2.rectangle(
            image_bgr,
            (x1, y1),
            (x2, y2),
            color_bgr,
            2,
        )

        text_x = max(0, x1)
        text_y = max(25, y1 - 10)

        cv2.putText(
            image_bgr,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color_bgr,
            2,
            cv2.LINE_AA,
        )


def main():
    args = parse_args()

    camera_path = resolve_project_path(
        args.cam_file
    )

    if not camera_path.is_file():
        raise FileNotFoundError(
            f"Camera calibration not found: {camera_path}"
        )

    calibration = json.loads(
        camera_path.read_text(
            encoding="utf-8"
        )
    )

    size = tuple(
        calibration["img_size"]
    )

    if size != tuple(OUT_RES):
        raise ValueError(
            f"Calibration size {size} must match "
            f"OUT_RES={OUT_RES}"
        )

    mesh_paths = {
        label: resolve_project_path(mesh_path)
        for label, mesh_path in MESH_PATHS.items()
    }

    width, height = size

    pose_detector = None
    bbox_detector = None
    pose_worker = None
    cam_worker = None
    window_created = False

    processed_frames = 0
    started = perf_counter()

    try:
        pose_detector = MegaPoseRunnerMulti(
            mesh_paths=mesh_paths,
            model_name=args.model,
            K_path=camera_path,
            batch_size=MPOSE_BATCH_SIZE,
            n_workers=N_WORKERS,
            iou_threshold=IOU,
            min_mesh_points=N_POINTS,
        )

        bbox_detector = ContourRunnerMulti(
            COLOR_RANGES,
        )

        camera_matrix = np.asarray(
            pose_detector.K,
            dtype=float,
        ).copy()

        pose_worker = LatestPoseInference(
            pose_detector
        )

        cam_worker = LatestFrameCamera(
            source=args.cam_source,
            frame_size=(
                width * 2,
                height,
            ),
            backend=cv2.CAP_V4L2,
            fps=FPS,
        )

        pose_worker.start()
        cam_worker.start()

        cv2.namedWindow(
            WINDOW_NAME,
            cv2.WINDOW_NORMAL,
        )

        window_created = True

        cv2.resizeWindow(
            WINDOW_NAME,
            1280,
            720,
        )

        print(
            "q/Escape: quit; r: reset all tracking"
        )

        last_frame_index = -1
        latest_left_bgr = None
        latest_detections = []

        while True:
            # Returns immediately with the newest camera frame.
            frame_data = cam_worker.latest()

            if (
                frame_data is not None
                and frame_data.index
                != last_frame_index
            ):
                last_frame_index = (
                    frame_data.index
                )

                latest_left_bgr = extract_left(
                    frame_data.image,
                    size,
                )

                image_rgb = cv2.cvtColor(
                    latest_left_bgr,
                    cv2.COLOR_BGR2RGB,
                )

                # Contour detection is currently synchronous,
                # but it does not wait for MegaPose.
                _, latest_detections = (
                    bbox_detector.estimate(
                        image_rgb,
                        draw=True,
                    )
                )

                # This returns immediately. If MegaPose is busy,
                # the newest pending job replaces the older one.
                pose_worker.submit(
                    frame_index=frame_data.index,
                    image_time_ns=(
                        frame_data.image_time_ns
                    ),
                    image_rgb=image_rgb,
                    detections=latest_detections,
                )

                processed_frames += 1

            if latest_left_bgr is not None:
                display = (
                    latest_left_bgr.copy()
                )

                draw_detections_bgr(
                    display,
                    latest_detections,
                    bbox_detector,
                )

                # Returns immediately with the last completed
                # MegaPose result.
                pose_packet = (
                    pose_worker.latest()
                )

                pose_is_fresh = (
                    pose_packet is not None
                    and (
                        time_ns()
                        - pose_packet.result_time_ns
                    )
                    <= POSE_HOLD_NS
                )

                if pose_is_fresh:
                    for (
                        label,
                        marker_pose,
                    ) in pose_packet.poses.items(): #type: ignore
                        draw_triaxis(
                            display,
                            camera_matrix,
                            marker_pose.pose_matrix,
                        )

                if pose_packet is None:
                    status = "POSE_PENDING"
                    pose_count = 0
                    inference_ms = 0.0
                else:
                    status = pose_packet.status
                    pose_count = len(
                        pose_packet.poses
                    )
                    inference_ms = (
                        pose_packet.inference_seconds
                        * 1000.0
                    )

                status_text = (
                    f"{status} | "
                    f"detections={len(latest_detections)} | "
                    f"poses={pose_count} | "
                    f"inference={inference_ms:.1f} ms | "
                    f"dropped={pose_worker.dropped_inputs}"
                )

                cv2.putText(
                    display,
                    status_text,
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                cv2.imshow(
                    WINDOW_NAME,
                    display,
                )

            # GUI processing continues even while MegaPose is busy.
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord("r"):
                pose_worker.request_reset()

            if (
                window_created
                and cv2.getWindowProperty(
                    WINDOW_NAME,
                    cv2.WND_PROP_VISIBLE,
                )
                < 1
            ):
                break

    except KeyboardInterrupt:
        print("\nStopped by keyboard")

    except Exception as error:
        print(f"ERROR: {error}")
        raise

    finally:
        # Stop all MegaPose calls before stopping its renderer.
        if pose_worker is not None:
            pose_worker.close()

        if cam_worker is not None:
            cam_worker.close()

        if pose_detector is not None:
            pose_detector.close()

        if window_created:
            cv2.destroyAllWindows()

        elapsed = perf_counter() - started

        print(
            f"Submitted {processed_frames} frames "
            f"in {elapsed:.1f} seconds"
        )

        if pose_worker is not None:
            print(
                f"MegaPose processed "
                f"{pose_worker.processed_inputs} jobs; "
                f"dropped "
                f"{pose_worker.dropped_inputs} pending jobs"
            )


if __name__ == "__main__":
    main()