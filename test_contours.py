import argparse
from collections import Counter
from time import perf_counter

import cv2

from configs.config import COLOR_RANGES, OUT_RES
from contour_runner import ContourRunnerMulti


WINDOW_NAME = "ContourRunnerMulti Test"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Test multi-marker detection using "
            "the right ZED camera"
        )
    )

    parser.add_argument(
        "--cam-id",
        default="2",
        help="Camera index or video path",
    )
    parser.add_argument(
        "--res",
        nargs=2,
        type=int,
        metavar=("WIDTH", "HEIGHT"),
        default=list(OUT_RES),
        help=(
            "Resolution of one ZED camera image, "
            "for example: --res 1920 1080"
        ),
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=15,
        help="Requested camera frame rate",
    )
    parser.add_argument(
        "--min-area",
        type=int,
        default=400,
        help="Minimum connected-component area",
    )

    args = parser.parse_args()

    width, height = args.res

    if width <= 0 or height <= 0:
        parser.error(
            "Resolution values must be positive"
        )

    if args.fps <= 0:
        parser.error(
            "FPS must be positive"
        )

    if args.min_area < 0:
        parser.error(
            "Minimum area must be non-negative"
        )

    return args


def open_capture(args):
    source = (
        int(args.cam_id)
        if args.cam_id.isdecimal()
        else args.cam_id
    )

    if isinstance(source, int):
        capture = cv2.VideoCapture(
            source,
            cv2.CAP_V4L2,
        )
    else:
        capture = cv2.VideoCapture(source)

    if not capture.isOpened():
        raise RuntimeError(
            f"Could not open camera or video "
            f"{args.cam_id!r}"
        )

    image_width, image_height = args.res

    # ZED UVC output contains left and right images
    # next to each other:
    #
    # [left image | right image]
    stereo_width = image_width * 2

    capture.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        stereo_width,
    )
    capture.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        image_height,
    )
    capture.set(
        cv2.CAP_PROP_FPS,
        args.fps,
    )
    capture.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1,
    )

    actual_width = int(
        capture.get(
            cv2.CAP_PROP_FRAME_WIDTH
        )
    )
    actual_height = int(
        capture.get(
            cv2.CAP_PROP_FRAME_HEIGHT
        )
    )
    actual_fps = capture.get(
        cv2.CAP_PROP_FPS
    )

    print(
        f"Requested stereo capture: "
        f"{stereo_width}x{image_height} "
        f"at {args.fps} FPS"
    )
    print(
        f"Reported capture: "
        f"{actual_width}x{actual_height} "
        f"at {actual_fps:.1f} FPS"
    )

    return capture


def extract_right_image(
    stereo_frame,
    resolution,
):
    image_width, image_height = resolution

    expected_shape = (
        image_height,
        image_width * 2,
        3,
    )

    if stereo_frame.shape != expected_shape:
        raise RuntimeError(
            f"Expected ZED stereo frame "
            f"{expected_shape}; received "
            f"{stereo_frame.shape}. "
            "The camera may not support the "
            "requested resolution."
        )

    # Select only the right camera.
    right_frame = stereo_frame[
        :,
        image_width:image_width * 2,
    ]

    return right_frame.copy()


def draw_information(
    image_bgr,
    bot_count,
    arena_count,
    inference_ms,
):
    valid_scene = (
        bot_count >= 1
        and arena_count >= 4
    )

    status = (
        "PASS"
        if valid_scene
        else "WAITING"
    )

    status_color = (
        (0, 255, 0)
        if valid_scene
        else (0, 0, 255)
    )

    detection_fps = (
        1000.0 / max(inference_ms, 0.001)
    )

    lines = [
        f"bot markers: {bot_count}",
        f"arena markers: {arena_count}",
        (
            f"detection time: "
            f"{inference_ms:.2f} ms"
        ),
        (
            f"detection FPS: "
            f"{detection_fps:.1f}"
        ),
        f"scene status: {status}",
        "q/Escape: quit",
    ]

    for index, line in enumerate(lines):
        position = (
            20,
            35 + index * 30,
        )

        # Dark outline for readability.
        cv2.putText(
            image_bgr,
            line,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )

        cv2.putText(
            image_bgr,
            line,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_color,
            2,
            cv2.LINE_AA,
        )


def main():
    args = parse_args()
    resolution = tuple(args.res)

    detector = ContourRunnerMulti(
        COLOR_RANGES,
        min_area=args.min_area,
        bbox_pad=0.10,
        kernel_size=5,
    )

    capture = open_capture(args)

    cv2.namedWindow(
        WINDOW_NAME,
        cv2.WINDOW_NORMAL,
    )
    cv2.resizeWindow(
        WINDOW_NAME,
        1280,
        720,
    )

    frame_count = 0
    total_inference_seconds = 0.0

    try:
        while True:
            success, stereo_frame = capture.read()

            if not success:
                raise RuntimeError(
                    "Camera stopped returning frames"
                )

            frame_bgr = extract_right_image(
                stereo_frame,
                resolution,
            )

            # ContourRunnerMulti expects RGB input.
            frame_rgb = cv2.cvtColor(
                frame_bgr,
                cv2.COLOR_BGR2RGB,
            )

            inference_start = perf_counter()

            vis_rgb, detections = detector.estimate(
                frame_rgb,
                draw=True,
            )

            inference_seconds = (
                perf_counter() - inference_start
            )
            inference_ms = (
                inference_seconds * 1000.0
            )

            total_inference_seconds += (
                inference_seconds
            )
            frame_count += 1

            counts = Counter(
                detection["label"]
                for detection in detections
            )

            bot_count = counts[
                "bot_marker"
            ]
            arena_count = counts[
                "arena_marker"
            ]

            # Detector visualization is RGB.
            vis_bgr = cv2.cvtColor(vis_rgb, cv2.COLOR_RGB2BGR,)

            draw_information(
                vis_bgr,
                bot_count,
                arena_count,
                inference_ms,
            )

            cv2.imshow(
                WINDOW_NAME,
                vis_bgr,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if (
                cv2.getWindowProperty(
                    WINDOW_NAME,
                    cv2.WND_PROP_VISIBLE,
                )
                < 1
            ):
                break

    finally:
        capture.release()
        cv2.destroyAllWindows()

        if frame_count > 0:
            average_ms = (
                total_inference_seconds
                / frame_count
                * 1000.0
            )

            print(
                f"Processed {frame_count} frames"
            )
            print(
                f"Average detection time: "
                f"{average_ms:.2f} ms"
            )


if __name__ == "__main__":
    main()