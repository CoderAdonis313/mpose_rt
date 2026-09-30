"""
Live HSV range tuner for the right ZED camera.

Controls:
    1-9         Select marker label
    Left click  Sample marker color
    Space       Freeze/resume frame
    p           Print COLOR_RANGES
    s           Save ranges to JSON
    r           Reset selected label
    q / Escape  Quit
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from configs.config_multi import COLOR_RANGES, OUT_RES


PREVIEW_WINDOW = "HSV Tuner - Right Camera"
MASK_WINDOW = "HSV Mask"

TRACKBARS = (
    ("H min", 179),
    ("H max", 179),
    ("S min", 255),
    ("S max", 255),
    ("V min", 255),
    ("V max", 255),
)


class HSVTuner:
    def __init__(
        self,
        initial_ranges,
        output_path,
        min_area=400,
        display_scale=0.5,
    ):
        if not initial_ranges:
            raise ValueError(
                "COLOR_RANGES must contain at least one label"
            )

        self.labels = list(initial_ranges)
        self.selected_index = 0
        self.output_path = Path(output_path)
        self.min_area = int(min_area)
        self.display_scale = float(display_scale)

        self.current_frame = None
        self.syncing_trackbars = False
        self.trackbars_ready = False

        self.ranges = {}
        self.initial_ranges = {}

        for label, (lower, upper) in initial_ranges.items():
            values = {
                "H min": int(lower[0]),
                "H max": int(upper[0]),
                "S min": int(lower[1]),
                "S max": int(upper[1]),
                "V min": int(lower[2]),
                "V max": int(upper[2]),
            }

            self.ranges[label] = values.copy()
            self.initial_ranges[label] = values.copy()

        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (5, 5),
        )

        cv2.namedWindow(
            PREVIEW_WINDOW,
            cv2.WINDOW_AUTOSIZE,
        )
        cv2.namedWindow(
            MASK_WINDOW,
            cv2.WINDOW_AUTOSIZE,
        )

        for name, maximum in TRACKBARS:
            cv2.createTrackbar(
                name,
                PREVIEW_WINDOW,
                0,
                maximum,
                self._on_trackbar,
            )

        self.trackbars_ready = True

        cv2.setMouseCallback(
            PREVIEW_WINDOW,
            self._on_mouse,
        )

        self._load_selected_range()

    @property
    def selected_label(self):
        return self.labels[self.selected_index]

    def _on_trackbar(self, unused_value):
        if (
            not self.trackbars_ready
            or self.syncing_trackbars
        ):
            return

        values = self.ranges[
            self.selected_label
        ]

        for name, _ in TRACKBARS:
            values[name] = cv2.getTrackbarPos(
                name,
                PREVIEW_WINDOW,
            )

    def _load_selected_range(self):
        self.syncing_trackbars = True

        values = self.ranges[
            self.selected_label
        ]

        for name, _ in TRACKBARS:
            cv2.setTrackbarPos(
                name,
                PREVIEW_WINDOW,
                values[name],
            )

        self.syncing_trackbars = False

        print(
            f"Selected: {self.selected_label}"
        )

    def _on_mouse(
        self,
        event,
        display_x,
        display_y,
        flags,
        parameter,
    ):
        if (
            event != cv2.EVENT_LBUTTONDOWN
            or self.current_frame is None
        ):
            return

        frame_height, frame_width = (
            self.current_frame.shape[:2]
        )

        image_x = int(
            display_x / self.display_scale
        )
        image_y = int(
            display_y / self.display_scale
        )

        image_x = int(
            np.clip(
                image_x,
                0,
                frame_width - 1,
            )
        )
        image_y = int(
            np.clip(
                image_y,
                0,
                frame_height - 1,
            )
        )

        # Sample an 11x11 patch instead of one noisy pixel.
        radius = 5

        x1 = max(0, image_x - radius)
        y1 = max(0, image_y - radius)
        x2 = min(
            frame_width,
            image_x + radius + 1,
        )
        y2 = min(
            frame_height,
            image_y + radius + 1,
        )

        patch_bgr = self.current_frame[
            y1:y2,
            x1:x2,
        ]

        patch_hsv = cv2.cvtColor(
            patch_bgr,
            cv2.COLOR_BGR2HSV,
        )

        median_hsv = np.median(
            patch_hsv.reshape(-1, 3),
            axis=0,
        ).astype(int)

        hue = int(median_hsv[0])
        saturation = int(median_hsv[1])
        value = int(median_hsv[2])

        values = self.ranges[
            self.selected_label
        ]

        # Hue is kept narrow. Saturation and value receive wider
        # margins to accommodate reflections and shadows.
        values.update({
            "H min": max(0, hue - 6),
            "H max": min(179, hue + 6),
            "S min": max(0, saturation - 120),
            "S max": 255,
            "V min": max(0, value - 190),
            "V max": 255,
        })

        self._load_selected_range()

        print(
            f"Clicked {self.selected_label}: "
            f"HSV=({hue}, {saturation}, {value})"
        )

        self.print_selected_range()

    def get_bounds(self, label):
        values = self.ranges[label]

        lower = np.asarray(
            [
                values["H min"],
                values["S min"],
                values["V min"],
            ],
            dtype=np.uint8,
        )

        upper = np.asarray(
            [
                values["H max"],
                values["S max"],
                values["V max"],
            ],
            dtype=np.uint8,
        )

        return lower, upper

    def make_mask(self, frame_bgr):
        hsv = cv2.cvtColor(
            frame_bgr,
            cv2.COLOR_BGR2HSV,
        )

        lower, upper = self.get_bounds(
            self.selected_label
        )

        mask = cv2.inRange(
            hsv,
            lower,
            upper,
        )

        # Remove isolated sensor noise without temporal flicker.
        mask = cv2.medianBlur(
            mask,
            5,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.kernel,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self.kernel,
        )

        return mask

    def draw_components(self, preview, mask):
        component_count, _, stats, _ = (
            cv2.connectedComponentsWithStats(
                mask,
                connectivity=8,
            )
        )

        detection_count = 0

        for index in range(1, component_count):
            area = int(
                stats[index, cv2.CC_STAT_AREA]
            )

            if area < self.min_area:
                continue

            x = int(
                stats[index, cv2.CC_STAT_LEFT]
            )
            y = int(
                stats[index, cv2.CC_STAT_TOP]
            )
            width = int(
                stats[index, cv2.CC_STAT_WIDTH]
            )
            height = int(
                stats[index, cv2.CC_STAT_HEIGHT]
            )

            detection_count += 1

            cv2.rectangle(
                preview,
                (x, y),
                (x + width, y + height),
                (0, 255, 0),
                3,
            )

        return detection_count

    def show(self, frame_bgr, paused):
        # Keep an independent stable frame for mouse sampling.
        self.current_frame = frame_bgr.copy()

        mask = self.make_mask(
            frame_bgr,
        )

        # Show the original image instead of flashing accepted pixels.
        preview = frame_bgr.copy()

        detection_count = self.draw_components(
            preview,
            mask,
        )

        lower, upper = self.get_bounds(
            self.selected_label
        )

        state = (
            "PAUSED"
            if paused
            else "LIVE"
        )

        lines = [
            f"{self.selected_label} | {state}",
            (
                f"Lower: "
                f"({lower[0]}, {lower[1]}, {lower[2]})"
            ),
            (
                f"Upper: "
                f"({upper[0]}, {upper[1]}, {upper[2]})"
            ),
            f"Components: {detection_count}",
            "1-9 label | click sample | Space freeze",
            "p print | s save | r reset | q quit",
        ]

        for index, line in enumerate(lines):
            position = (
                20,
                35 + index * 30,
            )

            cv2.putText(
                preview,
                line,
                position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )

            cv2.putText(
                preview,
                line,
                position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        display_width = max(
            1,
            int(
                preview.shape[1]
                * self.display_scale
            ),
        )
        display_height = max(
            1,
            int(
                preview.shape[0]
                * self.display_scale
            ),
        )

        display_size = (
            display_width,
            display_height,
        )

        preview_small = cv2.resize(
            preview,
            display_size,
            interpolation=cv2.INTER_AREA,
        )

        mask_small = cv2.resize(
            mask,
            display_size,
            interpolation=cv2.INTER_NEAREST,
        )

        cv2.imshow(
            PREVIEW_WINDOW,
            preview_small,
        )
        cv2.imshow(
            MASK_WINDOW,
            mask_small,
        )

    def select_label(self, index):
        if 0 <= index < len(self.labels):
            self.selected_index = index
            self._load_selected_range()

    def reset_selected(self):
        label = self.selected_label

        self.ranges[label] = (
            self.initial_ranges[label].copy()
        )

        self._load_selected_range()

        print(f"Reset {label}")

    def print_selected_range(self):
        lower, upper = self.get_bounds(
            self.selected_label
        )

        print(
            f'"{self.selected_label}": '
            f"(({int(lower[0])}, "
            f"{int(lower[1])}, "
            f"{int(lower[2])}), "
            f"({int(upper[0])}, "
            f"{int(upper[1])}, "
            f"{int(upper[2])})),"
        )

    def print_ranges(self):
        print("\nCOLOR_RANGES = {")

        for label in self.labels:
            lower, upper = self.get_bounds(label)

            print(
                f'    "{label}": '
                f"(({int(lower[0])}, "
                f"{int(lower[1])}, "
                f"{int(lower[2])}), "
                f"({int(upper[0])}, "
                f"{int(upper[1])}, "
                f"{int(upper[2])})),"
            )

        print("}\n")

    def save_ranges(self):
        output = {}

        for label in self.labels:
            lower, upper = self.get_bounds(label)

            output[label] = [
                [int(value) for value in lower],
                [int(value) for value in upper],
            ]

        self.output_path.write_text(
            json.dumps(
                output,
                indent=2,
            ),
            encoding="utf-8",
        )

        print(
            f"Saved ranges to {self.output_path}"
        )

        self.print_ranges()


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Tune HSV ranges using the right ZED camera"
        )
    )

    parser.add_argument(
        "--cam-id",
        default="2",
        help="Camera index",
    )
    parser.add_argument(
        "--res",
        nargs=2,
        type=int,
        metavar=("WIDTH", "HEIGHT"),
        default=list(OUT_RES),
        help="Resolution of one ZED camera image",
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
        help="Minimum displayed component area",
    )
    parser.add_argument(
        "--display-scale",
        type=float,
        default=0.5,
        help="Preview scaling factor",
    )
    parser.add_argument(
        "--output",
        default="hsv_ranges.json",
        help="JSON output path",
    )

    args = parser.parse_args()

    if args.res[0] <= 0 or args.res[1] <= 0:
        parser.error("Resolution must be positive")

    if args.fps <= 0:
        parser.error("FPS must be positive")

    if args.display_scale <= 0:
        parser.error(
            "Display scale must be positive"
        )

    return args


def open_camera(args):
    camera_id = int(args.cam_id)
    width, height = args.res

    capture = cv2.VideoCapture(
        camera_id,
        cv2.CAP_V4L2,
    )

    if not capture.isOpened():
        raise RuntimeError(
            f"Could not open camera {camera_id}"
        )

    # ZED UVC output contains left and right images.
    capture.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        width * 2,
    )
    capture.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        height,
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
        capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    )
    actual_height = int(
        capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )
    actual_fps = capture.get(
        cv2.CAP_PROP_FPS
    )

    print(
        f"Requested stereo capture: "
        f"{width * 2}x{height} at {args.fps} FPS"
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
    width, height = resolution

    expected_shape = (
        height,
        width * 2,
        3,
    )

    if stereo_frame.shape != expected_shape:
        raise RuntimeError(
            f"Expected stereo frame {expected_shape}; "
            f"received {stereo_frame.shape}. "
            "The camera may not support the requested mode."
        )

    return stereo_frame[
        :,
        width:width * 2,
    ].copy()


def main():
    args = parse_args()
    resolution = tuple(args.res)

    capture = open_camera(args)

    tuner = HSVTuner(
        initial_ranges=COLOR_RANGES,
        output_path=args.output,
        min_area=args.min_area,
        display_scale=args.display_scale,
    )

    paused = False
    right_frame = None

    print("\nControls:")

    for index, label in enumerate(tuner.labels):
        print(f"  {index + 1}: {label}")

    print("  Left click: sample color")
    print("  Space: freeze/resume")
    print("  p: print ranges")
    print("  s: save ranges")
    print("  r: reset selected label")
    print("  q/Escape: quit")

    try:
        while True:
            if not paused or right_frame is None:
                success, stereo_frame = capture.read()

                if not success:
                    raise RuntimeError(
                        "Camera stopped returning frames"
                    )

                right_frame = extract_right_image(
                    stereo_frame,
                    resolution,
                )

            tuner.show(
                right_frame,
                paused,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if ord("1") <= key <= ord("9"):
                tuner.select_label(
                    key - ord("1")
                )

            elif key == ord(" "):
                paused = not paused

                print(
                    "Frame frozen"
                    if paused
                    else "Live camera resumed"
                )

            elif key == ord("p"):
                tuner.print_ranges()

            elif key == ord("s"):
                tuner.save_ranges()

            elif key == ord("r"):
                tuner.reset_selected()

            if (
                cv2.getWindowProperty(
                    PREVIEW_WINDOW,
                    cv2.WND_PROP_VISIBLE,
                )
                < 1
            ):
                break

    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()