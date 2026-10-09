#!/usr/bin/env python3
"""
Non-blocking multi-marker MegaPose camera test.

Camera capture and MegaPose run in worker threads. The main thread performs
contour detection, draws the latest available results, and handles the GUI.
"""

import json
from argparse import ArgumentParser
from pathlib import Path
from time import perf_counter, time_ns, time
import numpy as np
from configs.config_multi import *
from src.multi.cam_worker import LatestFrameCamera
from src.multi.contour_runner_multi import ContourRunnerMulti
from src.multi.mpose_runner_multi import MegaPoseRunnerMulti
from src.multi.pose_worker import LatestPoseInference
from src.multi.multi_utils import PoseSender
from src.multi.writer_worker import VideoWriterWorker


import cv2
cv2.ocl.setUseOpenCL(False)
cv2.setNumThreads(1)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WINDOW_NAME = "ZED Multi-Marker MegaPose"
POSE_HOLD_NS = 1_000_000_000


def parse_args():
    parser = ArgumentParser(description=__doc__,)
    parser.add_argument("--cam_id",default=0, type=int)
    parser.add_argument("--cam_file", required=True, type=str)
    parser.add_argument("--model", default="megapose-1.0-RGB",)
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument('--frame_id', default="zed_left_cam")
    parser.add_argument('--out_dir', default='outputs/live')
    args = parser.parse_args()
    return args


def extract_left(stereo_frame, size):
    width, height = size
    expected_shape = (height, width * 2, 3)

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


def draw_triaxis(image_bgr, camera_matrix, pose_matrix, axis_length=0.1):
    rotation = np.asarray(pose_matrix[:3, :3],dtype=float,)
    translation = np.asarray(pose_matrix[:3, 3],dtype=float,).reshape(3, 1)

    rotation_vector, _ = cv2.Rodrigues(rotation)
    distortion = np.zeros((5, 1), dtype=float)

    cv2.drawFrameAxes(
        image_bgr,
        camera_matrix,
        distortion,
        rotation_vector,
        translation,
        axis_length,
        2,
    )


def draw_detections(
    image_bgr,
    detections,
    label_colors,
):
    """Draw the current contour detections on a BGR image."""

    for item in detections:
        label = item["label"]
        x1, y1, x2, y2 = map(int, item["detection"],)

        # ContourRunnerMulti stores display colors for an RGB image.
        color_bgr = label_colors.get(label, (255, 0, 255),)
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
    cam_path = PROJECT_ROOT / args.cam_file
    out_path = PROJECT_ROOT / args.out_dir / f'rt_live_{time()}.mp4'

    calibration = json.loads(cam_path.read_text(encoding="utf-8"))
    size = tuple(calibration["img_size"])

    if size != tuple(OUT_RES):
        raise ValueError(f"Calibration size {size} must match OUT_RES={OUT_RES}")

    if not 1 <= args.udp_port <= 65535:
        raise ValueError("UDP port must be between 1 and 65535")

    mesh_paths = {
        label: PROJECT_ROOT / mesh_path
        for label, mesh_path in MESH_PATHS.items()
    }

    width, height = size
    pose_detector = None
    bbox_detector = None
    pose_worker = None
    cam_worker = None
    writer_worker = None
    window_created = False

    processed_frames = 0
    started = perf_counter()
    sender = PoseSender(
        port=args.udp_port,
        frame_id=args.frame_id,
        send_hz=SEND_RATE,
        max_pose_age_ns=POSE_HOLD_NS,
    )
    terminal_status = "STOPPED"

    try:
        sender.send('STARTING', force=True)

        pose_detector = MegaPoseRunnerMulti(
            mesh_paths=mesh_paths,
            model_name=args.model,
            K_path=cam_path,
            batch_size=MPOSE_BATCH_SIZE,
            n_workers=N_WORKERS,
            iou_threshold=IOU,
            min_mesh_points=N_POINTS,
        )

        bbox_detector = ContourRunnerMulti(
            COLOR_RANGES,
            bbox_pad=BBOX_PAD
        )

        camera_matrix = np.asarray(
            pose_detector.K,
            dtype=float,
        ).copy()

        pose_worker = LatestPoseInference(
            pose_detector
        )

        cam_worker = LatestFrameCamera(
            source=args.cam_id,
            frame_size=(width * 2, height,),
            backend=cv2.CAP_V4L2,
            fps=FPS,
        )

        writer_worker = VideoWriterWorker(
            out_path=str(out_path),
            frame_size=OUT_RES,
            fps=FPS,
            buffer_size=BUFFER_SIZE
        )

        pose_worker.start()
        cam_worker.start()
        writer_worker.start()

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL,)
        cv2.resizeWindow(WINDOW_NAME, 1280, 720,)
        window_created = True

        print("q/Escape: quit; r: reset all tracking")

        last_frame_index = -1
        last_written_frame_index = -1
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
                last_frame_index = (frame_data.index)
                latest_left_bgr = extract_left(frame_data.image, size,)

                image_rgb = cv2.cvtColor(latest_left_bgr, cv2.COLOR_BGR2RGB,)
                # Contour detection is currently synchronous,
                # but it does not wait for MegaPose.
                _, latest_detections = (
                    bbox_detector.estimate(
                        image_rgb,
                        draw=False,
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
                display = (latest_left_bgr.copy())
                draw_detections(display, latest_detections, bbox_detector.label_colors,)

                # Returns immediately with the last completed
                # MegaPose result.
                pose_packet = (
                    pose_worker.latest()
                )

                pose_is_fresh = (
                    pose_packet is not None
                    and (time_ns() - pose_packet.image_time_ns) <= POSE_HOLD_NS 
                    # Tells is the original image captured time and now time is too old, then this packet is stale
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
                    pose_count = len(pose_packet.poses)
                    inference_ms = (pose_packet.inference_seconds* 1000.0)

                status_text = (
                    f"{status} | "
                    f"detections={len(latest_detections)} | "
                    f"poses={pose_count} | "
                    f"inference={inference_ms:.1f} ms | "
                    f"dropped={pose_worker.dropped_inputs}"
                )

                sender.send(
                    status,
                    pose_packet
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

                if last_frame_index != last_written_frame_index:
                    writer_worker.submit(frame_index=last_frame_index, frame=display)
                    last_written_frame_index = last_frame_index

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
                sender.send('RESET', force=True)

            if (window_created and cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE,) < 1):
                break

    except KeyboardInterrupt:
        print("\nStopped by keyboard")

    except Exception as error:
        print(f"ERROR: {error}")
        terminal_status = 'ERROR'
        raise

    finally:
        # Stop all MegaPose calls before stopping its renderer.
        try:
            sender.send(terminal_status, force=True)
        finally:
            sender.close()

        if pose_worker is not None:
            pose_worker.close()

        if cam_worker is not None:
            cam_worker.close()

        if pose_detector is not None:
            pose_detector.close()

        if writer_worker is not None:
            writer_worker.close()

        if window_created:
            cv2.destroyAllWindows()

        elapsed = perf_counter() - started
        print(
            f"Submitted {processed_frames} frames "
            f"in {elapsed:.1f} seconds"
        )
        print(f'Wrote output file to: {out_path}')
        print(f"Video writer wrote {writer_worker.written_frames} frames and dropped {writer_worker.dropped_frames} frames") #type: ignore

        if pose_worker is not None:
            print(
                f"MegaPose processed "
                f"{pose_worker.processed_inputs} jobs; "
                f"dropped "
                f"{pose_worker.dropped_inputs} pending jobs"
            )


if __name__ == "__main__":
    main()