import cv2
import json
import time
import numpy as np
from pathlib import Path
from configs.config import OUT_RES


class ContourRunner:
    def __init__(
        self,
        hsv_ranges,
        min_area=400,
        bbox_pad=0.10,
        kernel_size=5,
        debug_mask=False,
    ):
        """
        hsv_ranges:
            list of tuples like [((h1, s1, v1), (h2, s2, v2)), ...]
            Use multiple ranges when hue wraps around, e.g. red/pink.

        min_area:
            reject tiny blobs

        bbox_pad:
            fractional padding around contour bbox

        kernel_size:
            morphology cleanup kernel

        debug_mask:
            if True, saves/visualizes mask-like output
        """
        self.hsv_ranges = hsv_ranges
        self.min_area = min_area
        self.bbox_pad = bbox_pad
        self.kernel_size = kernel_size
        self.debug_mask = debug_mask

        self.img = None
        self.detections = []


    def write_json(self):
        if len(self.detections) == 0:
            data = [{
                "label": "fiducial",
                "bbox_modal": [],
                "detection": False
            }]
        else:
            data = [{
                "label": "fiducial",
                "bbox_modal": self.detections[0],
                "detection": True
            }]

        timestamp = int(time.time() * 1000)
        pose_folder_path = Path("contour_poses")
        pose_folder_path.mkdir(parents=True, exist_ok=True)
        file_name = str(pose_folder_path / f"object_data_{timestamp}.json")

        with open(file_name, "w", encoding="utf-8") as jfile:
            json.dump(data, jfile)

        return file_name


    def _make_mask(self, img_rgb):
        hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

        mask = None
        for lower, upper in self.hsv_ranges:
            lower_np = np.array(lower, dtype=np.uint8)
            upper_np = np.array(upper, dtype=np.uint8)
            part = cv2.inRange(hsv, lower_np, upper_np)
            mask = part if mask is None else cv2.bitwise_or(mask, part)

        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # type: ignore
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask


    def _best_bbox_from_mask(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = -1

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # optional extra filters
            if w < 8 or h < 8:
                continue

            if area > best_area:
                best_area = area
                best = (x, y, w, h)

        return best


    def _pad_bbox(self, x, y, w, h, W, H):
        pad_x = int(w * self.bbox_pad)
        pad_y = int(h * self.bbox_pad)

        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(W - 1, x + w + pad_x)
        y2 = min(H - 1, y + h + pad_y)

        return [int(x1), int(y1), int(x2), int(y2)]


    def estimate(self, img: np.ndarray):
        self.img = img.copy()
        H, W = img.shape[:2]

        mask = self._make_mask(img)
        bbox = self._best_bbox_from_mask(mask)

        vis = img.copy()
        detections = []

        if bbox is not None:
            x, y, w, h = bbox
            bbox_xyxy = self._pad_bbox(x, y, w, h, W, H)
            detections.append(bbox_xyxy)

            x1, y1, x2, y2 = bbox_xyxy
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

        self.detections = detections

        if self.debug_mask:
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
            vis = np.hstack([vis, mask_bgr])

        return vis, detections


    # def estimate_and_save(self, i_path: str, save_vis_path: str, resize_shape=OUT_RES):
    #     vis_img, detections = self.estimate(i_path, resize_shape=resize_shape)
    #     file_path = self.write_json()

    #     if save_vis_path is not None:
    #         cv2.imwrite(save_vis_path, vis_img)
    #         print(f"Wrote image: {save_vis_path}")

    #     return file_path