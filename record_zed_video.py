#!/usr/bin/env python3

import argparse
from pathlib import Path
from time import time, strftime

#################### use conda env zed_ros for this stuff #################################
import cv2
import pyzed.sl as sl


RESOLUTION_MAP = {
    "HD2K": sl.RESOLUTION.HD2K,
    "FHD": sl.RESOLUTION.HD1080,
    "HD1080": sl.RESOLUTION.HD1080,
    "HD720": sl.RESOLUTION.HD720,
    "VGA": sl.RESOLUTION.VGA,
}

VIEW_MAP = {
    "left": sl.VIEW.LEFT,
    "right": sl.VIEW.RIGHT,
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record a fixed-duration video from a ZED camera."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Recording duration in seconds.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Camera FPS and output video FPS.",
    )
    parser.add_argument(
        "--view",
        default="left",
        choices=sorted(VIEW_MAP.keys()),
        help="Which ZED camera view to record.",
    )
    parser.add_argument(
        "--resolution",
        default="HD1080",
        choices=sorted(RESOLUTION_MAP.keys()),
        help="ZED camera resolution.",
    )
    return parser.parse_args()


def default_output_path():
    output_dir = Path.cwd() / 'captures'
    output_dir.mkdir(exist_ok=True, parents=True)
    return output_dir / f"zed_capture_{strftime('%Y%m%d_%H%M%S')}.mp4"


def zed_frame_to_bgr(image):
    frame = image.get_data()
    if frame.ndim == 3 and frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def main():
    args = parse_args()
    output_path = default_output_path()
    output_path.parent.mkdir(exist_ok=True, parents=True)

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = RESOLUTION_MAP[args.resolution]
    init_params.camera_fps = args.fps

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED: {repr(err)}")

    runtime_params = sl.RuntimeParameters()
    image = sl.Mat()
    writer = None

    cv2.namedWindow("ZED recording", cv2.WINDOW_NORMAL)
    cv2.resizeWindow('ZED recording', 640, 480)

    print(f"Recording {args.view} ZED view for {args.duration:.1f}s")
    print(f"Resolution: {args.resolution}, FPS: {args.fps}")
    print(f"Output: {output_path}")
    print("Press 'q' to stop early.")

    try:
        print('Keeping camera on for 10 seconds to finish loading')
        cam_load_start = time()

        while time() - cam_load_start < 10:
            err = zed.grab(runtime_params)
            if err != sl.ERROR_CODE.SUCCESS:
                print(f"Grab failed: {err}")
                continue

            zed.retrieve_image(image, VIEW_MAP[args.view])
            frame_bgr = zed_frame_to_bgr(image)

            if writer is None:
                height, width = frame_bgr.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(
                    str(output_path),
                    fourcc,
                    float(args.fps),
                    (width, height),
                )
                if not writer.isOpened():
                    raise RuntimeError(f"Failed to open video writer: {output_path}")

            cv2.imshow("ZED recording", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Stopped early.")
                break

        start_time = time()
        frame_count = 0
        print('Starting recording now !')

        while time() - start_time < args.duration:
            err = zed.grab(runtime_params)
            if err != sl.ERROR_CODE.SUCCESS:
                print(f"Grab failed: {err}")
                continue

            zed.retrieve_image(image, VIEW_MAP[args.view])
            frame_bgr = zed_frame_to_bgr(image)

            writer.write(frame_bgr)
            frame_count += 1

            cv2.imshow("ZED recording", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Stopped early.")
                break

    finally:
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        zed.close()

    elapsed = time() - start_time
    print(f"Saved {frame_count} frames in {elapsed:.2f}s: {output_path}")


if __name__ == "__main__":
    main()
