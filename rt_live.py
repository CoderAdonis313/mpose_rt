#!/usr/bin/env python3
"""OpenCV ZED inference with JSON pose packets sent over localhost UDP.

Run with the mpose Python environment. No ROS or ZED SDK imports.
Images are raw left-camera frames; no undistortion is performed.
"""

import csv
import json
import socket
import sys
from argparse import ArgumentParser
from datetime import datetime
from pathlib import Path
from time import perf_counter, time_ns
from uuid import uuid4

import cv2
import numpy as np


def parse_args():
    parser = ArgumentParser(description=__doc__)
    # parser.add_argument(
    #     "--project-root",
    #     type=Path,
    #     default=Path.home() / "dev/mpose_rt",
    # )
    parser.add_argument("--cam_source", default="0")
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--cam_file", default="zed_1080p_raw_calib.json")
    parser.add_argument("--model", default="megapose-1.0-RGB")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--megapose-batch-size", type=int, default=16)
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--frame-id", default="zed_left_camera_optical_frame")

    args = parser.parse_args()

    if args.fps <= 0 or args.megapose_batch_size <= 0:
        parser.error("FPS and batch size must be positive.")
    if not 0 <= args.iou_threshold <= 1:
        parser.error("IoU threshold must be between 0 and 1.")
    if not 1 <= args.udp_port <= 65535:
        parser.error("UDP port must be between 1 and 65535.")

    # args.project_root = args.project_root.expanduser().resolve()
    # if not (args.project_root / "mpose_runner.py").is_file():
    #     parser.error("--project-root must contain mpose_runner.py.")

    return args


def resolve_input(value, root, folder):
    path = Path(value).expanduser()
    for candidate in (path, root / path, root / folder / path):
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f"Cannot locate {value!r} in {root / folder}")


class PoseSender:
    def __init__(self, port, frame_id):
        self.destination = ("127.0.0.1", port)
        self.frame_id = frame_id
        self.session_id = uuid4().hex
        self.packet_seq = 0
        self.failed_sends = 0
        self.last_warning = -float("inf")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(
        self,
        status,
        frame_index=None,
        image_time_ns=None,
        result_time_ns=None,
        transform=None,
        inference_seconds=None,
    ):
        valid = status == "TRACKING"
        matrix = None

        if valid:
            matrix = np.asarray(transform, dtype=float)
            if (
                matrix.shape != (4, 4)
                or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0, 0, 0, 1])
                or matrix[2, 3] <= 0
            ):
                raise ValueError("Invalid current-frame object-to-camera pose.")

            if frame_index is None or image_time_ns is None:
                raise ValueError(
                    "A valid pose requires its source frame timestamp."
                )

            matrix = matrix.tolist()

        packet = {
            "schema_version": 1,
            "session_id": self.session_id,
            "packet_seq": self.packet_seq,
            "frame_index": frame_index,
            "image_time_ns": image_time_ns,
            "result_time_ns": (
                result_time_ns if result_time_ns is not None else time_ns()
            ),
            "timestamp_source": "host_read",
            "frame_id": self.frame_id,
            "object_id": "fiducial",
            "status": status,
            "valid": valid,
            "T_camera_object": matrix,
            "inference_seconds": inference_seconds,
        }

        payload = json.dumps(
            packet,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")

        self.packet_seq += 1

        try:
            self.sock.sendto(payload, self.destination)
            return True
        except OSError as error:
            self.failed_sends += 1
            now = perf_counter()
            if now - self.last_warning >= 2.0:
                print(f"UDP send failed: {error}", file=sys.stderr)
                self.last_warning = now
            return False

    def close(self):
        self.sock.close()


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
    root = Path.cwd()

    # This script can live in ros2_ws while reusing the inference project.
    sys.path.insert(0, str(root))

    import torch
    from configs.config import COLOR_RANGES, OUT_RES
    from contour_runner import ContourRunner
    from mpose_runner import MegaPoseRunner

    mesh_path = resolve_input(args.mesh, root, "models")
    camera_path = resolve_input(args.cam_file, root, "configs")
    size = tuple(json.loads(camera_path.read_text())["img_size"])

    if size != tuple(OUT_RES):
        raise ValueError(
            f"Calibration {size} must match OUT_RES={OUT_RES}."
        )

    width, height = size

    sender = PoseSender(args.udp_port, args.frame_id)
    cap = video = pose_file = None
    window_created = False
    terminal_status = "STOPPED"
    frame_index = 0
    processed = 0
    started = perf_counter()

    try:
        sender.send("STARTING")

        mpose = MegaPoseRunner(
            mesh_path,
            "fiducial",
            args.model,
            camera_path,
            batch_size=args.megapose_batch_size,
        )
        detector = ContourRunner(COLOR_RANGES)

        source = (
            int(args.cam_source)
            if args.cam_source.isdecimal()
            else args.cam_source
        )
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)

        if not cap.isOpened():
            raise RuntimeError(
                f"Cannot open ZED device {args.cam_source}."
            )

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width * 2)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Backend-dependent, best effort.

        output_dir = root / "outputs"
        output_dir.mkdir(parents=True, exist_ok=True)
        stem = "zed_pose_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        video_path = output_dir / f"{stem}.mp4"
        csv_path = output_dir / f"{stem}.csv"

        video = cv2.VideoWriter(
            str(video_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(args.fps),
            size,
        )
        if not video.isOpened():
            raise RuntimeError(f"Cannot create {video_path}.")

        pose_file = csv_path.open("w", newline="", encoding="utf-8")
        rows = csv.writer(pose_file)
        rows.writerow([
            "session_id",
            "frame_index",
            "image_time_ns",
            "result_time_ns",
            "status",
            "x_camera_m",
            "y_camera_m",
            "z_camera_m",
            "roll_camera_deg",
            "pitch_camera_deg",
            "yaw_camera_deg",
            "inference_seconds",
            "udp_send_ok",
        ])

        cv2.namedWindow("ZED MegaPose", cv2.WINDOW_NORMAL)
        window_created = True
        cv2.resizeWindow("ZED MegaPose", 1280, 720)

        print(f"UDP destination: 127.0.0.1:{args.udp_port}")
        print(f"Session: {sender.session_id}")
        print(f"Video: {video_path}\nPoses: {csv_path}")
        print("q/Escape: quit; r: reset tracking.")
        print("Timestamps are host read times, not hardware exposure times.")
        print(f"Recorded MP4 playback uses fixed {args.fps} FPS.")

        previous_time = perf_counter()

        while True:
            ok, frame = cap.read()
            image_time_ns = time_ns()

            if not ok:
                raise RuntimeError("ZED stopped returning frames.")

            left_bgr = extract_left(frame, size)
            inference_start = perf_counter()

            try:
                display, pose, status = process_frame(
                    mpose,
                    detector,
                    left_bgr,
                    args.iou_threshold,
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
            sent = sender.send(
                status,
                frame_index,
                image_time_ns,
                result_time_ns,
                transform,
                inference_seconds,
            )

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

            values = pose if pose is not None else [float("nan")] * 6
            rows.writerow([
                sender.session_id,
                frame_index,
                image_time_ns,
                result_time_ns,
                status,
                *values,
                f"{inference_seconds:.6f}",
                int(sent),
            ])
            pose_file.flush()

            video.write(display)
            cv2.imshow("ZED MegaPose", display)
            processed += 1
            frame_index += 1

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                mpose.reset_tracking()
                sender.send("RESET")
            if cv2.getWindowProperty(
                "ZED MegaPose", cv2.WND_PROP_VISIBLE
            ) < 1:
                break

    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception:
        terminal_status = "ERROR"
        raise
    finally:
        # Best effort: the receiver must also enforce its own timeout.
        try:
            sender.send(terminal_status)
        finally:
            sender.close()
            if cap is not None:
                cap.release()
            if video is not None:
                video.release()
            if pose_file is not None:
                pose_file.close()
            if window_created:
                cv2.destroyAllWindows()

        print(
            f"Processed {processed} frames "
            f"in {perf_counter() - started:.1f}s."
        )
        print(f"Local UDP send failures: {sender.failed_sends}")


if __name__ == "__main__":
    main()