import json
import time
from pathlib import Path
import cv2
import numpy as np


class ContourRunnerMulti:
    def __init__(self, hsv_ranges_dict, min_area=400, bbox_pad=0.10, kernel_size=5, debug_mask=False):
        if not hsv_ranges_dict:
            raise ValueError("At least one HSV range is required")

        if min_area < 0:
            raise ValueError("min_area must be non-negative")

        if bbox_pad < 0:
            raise ValueError("bbox_pad must be non-negative")

        if kernel_size < 1:
            raise ValueError("kernel_size must be positive")

        self.min_area = int(min_area)
        self.bbox_pad = float(bbox_pad)
        self.debug_mask = bool(debug_mask)

        # Convert bounds once instead of doing it for every frame.
        self.hsv_ranges = {}

        for label, hsv_range in hsv_ranges_dict.items():
            lower, upper = hsv_range
            lower_array = np.asarray(lower, dtype=np.uint8)
            upper_array = np.asarray(upper, dtype=np.uint8)

            self.hsv_ranges[label] = (
                lower_array,
                upper_array,
            )

        # Build the morphology kernel once.
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )

        self.detections = []


    def _pad_bbox(self, x, y, width, height, image_width, image_height):
        pad_x = int(width * self.bbox_pad)
        pad_y = int(height * self.bbox_pad)

        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(image_width - 1, x + width + pad_x)
        y2 = min(image_height - 1, y + height + pad_y)

        return [int(x1), int(y1), int(x2), int(y2),]


    def _detect_components(self, label, mask, image_width, image_height, ):
        """
        Find every connected component for one label.

        Connected components directly provides bounding boxes and areas,
        avoiding the additional contour-to-bounding-box conversion.
        """
        component_count, _, stats, _ = (
            cv2.connectedComponentsWithStats(
                mask,
                connectivity=8,
            )
        )

        detections = []

        # Component zero is the background.
        for component_index in range(1, component_count):
            x = int(stats[component_index, cv2.CC_STAT_LEFT,])
            y = int(stats[component_index, cv2.CC_STAT_TOP,])
            width = int(stats[component_index, cv2.CC_STAT_WIDTH,])
            height = int(stats[component_index, cv2.CC_STAT_HEIGHT,])
            area = int(stats[component_index, cv2.CC_STAT_AREA,])

            if area < self.min_area:
                continue

            if width < 8 or height < 8:
                continue

            bbox = self._pad_bbox(x, y, width, height, image_width, image_height,)

            detections.append({
                "label": label,
                "detection": bbox,
            })
        return detections


    @staticmethod
    def draw_detections(image, detections):
        """
        Draw labeled detections on an RGB image.
        """
        colors = {
            "bot_marker": (0, 255, 0),
            "arena_marker": (255, 255, 0),
        }

        for item in detections:
            label = item["label"]
            x1, y1, x2, y2 = item["detection"]

            color = colors.get(label, (255, 0, 255),)

            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2,)


    def estimate(self, img, draw=False):
        """
        Detect all configured marker colors.

        Args:
            img:
                uint8 RGB image.

            draw:
                If True, return a copy with labeled bounding boxes.
                Keep False for fastest inference.

        Returns:
            vis:
                Annotated RGB image when draw=True.
                None when draw=False and debug_mask=False.

            detections:
                List of labeled bounding-box dictionaries.
        """
        image_height, image_width = img.shape[:2]

        # Perform RGB-to-HSV conversion once for all labels.
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV,)
        detections = []
        combined_mask = None

        if self.debug_mask:
            combined_mask = np.zeros(
                (image_height, image_width),
                dtype=np.uint8,
            )

        for label, (lower, upper) in self.hsv_ranges.items():
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel,)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel,)

            label_detections = self._detect_components(label, mask, image_width, image_height,)
            detections.extend(label_detections)

            if combined_mask is not None:
                cv2.bitwise_or(combined_mask, mask, dst=combined_mask,)

        self.detections = detections

        # Avoid allocating a display image during inference.
        if not draw and not self.debug_mask:
            return None, detections

        vis = img.copy()

        if draw:
            self.draw_detections(
                vis,
                detections,
            )

        if combined_mask is not None:
            mask_rgb = cv2.cvtColor(
                combined_mask,
                cv2.COLOR_GRAY2RGB,
            )

            vis = np.hstack([
                vis,
                mask_rgb,
            ])

        return vis, detections


    def write_json(self):
        data = [
            {
                "label": item["label"],
                "bbox_modal": item["detection"],
                "detection": True,
            }
            for item in self.detections
        ]

        timestamp = int(time.time() * 1000)

        output_folder = Path("contour_poses")
        output_folder.mkdir(
            parents=True,
            exist_ok=True,
        )

        output_path = (
            output_folder
            / f"object_data_{timestamp}.json"
        )

        with output_path.open(
            "w",
            encoding="utf-8",
        ) as json_file:
            json.dump(
                data,
                json_file,
                indent=2,
            )

        return str(output_path)