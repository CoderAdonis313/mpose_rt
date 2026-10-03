"""Multi-marker MegaPose inference with label-based tracking."""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import pandas as pd
import torch
import trimesh

from megapose.datasets.object_dataset import (
    RigidObject,
    RigidObjectDataset,
)
from megapose.datasets.scene_dataset import (
    CameraData,
    ObjectData,
)
from megapose.inference.types import (
    ObservationTensor,
    PoseEstimatesType,
)
from megapose.inference.utils import (
    make_detections_from_object_data,
)
from megapose.utils.tensor_collection import (
    PandasTensorCollection,
    concatenate,
)

from src.multi.custom_load_model import (
    NAMED_MODELS,
    load_named_model,
)


@dataclass(frozen=True)
class MarkerPose:
    """Accepted pose for one uniquely labeled marker."""

    label: str
    pose_matrix: np.ndarray
    # translation_m: Tuple[float, float, float]
    # euler_xyz_deg: Tuple[float, float, float]
    detection_bbox: np.ndarray
    projected_bbox: Optional[np.ndarray]

    # @property
    # def pose_values(self):
    #     """Return the legacy x, y, z, roll, pitch, yaw tuple."""

    #     return (
    #         *self.translation_m,
    #         *self.euler_xyz_deg,
    #     )


class MegaPoseRunnerMulti:
    """Estimate and track any number of uniquely labeled markers.

    Each marker label must appear exactly once in ``mesh_paths``. Contour
    detections must use those same labels.

    A marker's previous pose is refined while its projected bbox agrees with
    the current contour bbox. Otherwise that marker is independently
    reinitialized from its contour bbox.
    """

    def __init__(
        self,
        mesh_paths: dict,
        model_name: str,
        K_path: Path,
        n_workers: int = 6,
        batch_size: int = 16,
        iou_threshold: float = 0.25,
        mesh_units: str = "m",
        min_mesh_points: int = 1000,

    ):
        if not mesh_paths:
            raise ValueError("At least one marker mesh must be configured")

        if not 0.0 <= iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be between zero and one")

        self.model_name = model_name
        self.model_info = NAMED_MODELS[model_name]
        self.megapose_batch_size = int(batch_size)
        self.iou_threshold = float(iou_threshold)
        self._closed = False

        self.mesh_paths = {}
        self.mesh_points = {}
        rigid_objects = []

        for label, mesh_path in mesh_paths.items():
            mesh_path = Path(mesh_path)
            mesh = trimesh.load(
                str(mesh_path),
                force="mesh",
                process=False,
            )

            if not isinstance(mesh, trimesh.Trimesh):
                raise TypeError(
                    f"Expected a triangle mesh for {label!r}; "
                    f"got {type(mesh).__name__}"
                )

            vertices = np.asarray(
                mesh.vertices,
                dtype=np.float32,
            )

            if vertices is not None and len(vertices) < min_mesh_points:
                raise ValueError(
                    f"Mesh for {label!r} must contain {min_mesh_points} finite vertices"
                )

            self.mesh_paths[label] = mesh_path
            self.mesh_points[label] = vertices

            rigid_objects.append(
                RigidObject(
                    label=label,
                    mesh_path=mesh_path,
                    mesh_units=mesh_units,
                )
            )

        self.labels = tuple(self.mesh_paths)
        self.object_dataset = RigidObjectDataset(
            rigid_objects
        )

        K_path = Path(K_path)

        if not K_path.is_file():
            raise FileNotFoundError(
                f"Camera calibration does not exist: {K_path}"
            )

        calibration = json.loads(
            K_path.read_text(encoding="utf-8")
        )

        image_size = calibration.get("img_size")
        camera_matrix = calibration.get(
            "camera_matrix"
        )

        if (
            not isinstance(image_size, list)
            or len(image_size) != 2
        ):
            raise ValueError(
                "Calibration img_size must be [width, height]"
            )

        if (
            int(image_size[0]) <= 0
            or int(image_size[1]) <= 0
        ):
            raise ValueError(
                "Calibration image dimensions must be positive"
            )

        mpose_camera = {
            "K": camera_matrix,
            "resolution": list(
                reversed(image_size)
            ),
        }

        camera_data = CameraData.from_json(
            json.dumps(mpose_camera)
        )

        self.K = np.asarray(
            camera_data.K,
            dtype=float,
        )
        self.resolution = tuple(
            int(value)
            for value in camera_data.resolution
        )
        self.camera_data = camera_data

        # Load one model and one mesh database containing every marker.
        self.pose_estimator = load_named_model(
            model_name,
            self.object_dataset,
            n_workers=n_workers,
            bsz_images=self.megapose_batch_size,
        ).cuda()
        self.pose_estimator.eval()

        # One normalized GPU pose collection per marker.
        self.previous_pose_estimates: Dict[
            str,
            PoseEstimatesType,
        ] = {}

        # One current CPU 4x4 pose matrix per marker.
        self.poses: Dict[str, np.ndarray] = {}

        self.last_detections: Dict[
            str,
            np.ndarray,
        ] = {}


    @property
    def tracking_labels(self):
        return frozenset(
            self.previous_pose_estimates
        )


    @property
    def tracking_active(self):
        """True when at least one marker is being tracked."""

        return bool(self.previous_pose_estimates)


    def is_tracking(self, label):
        return label in self.previous_pose_estimates


    def _validate_image(self, image_rgb):
        if (
            not isinstance(image_rgb, np.ndarray)
            or image_rgb.ndim != 3
            or image_rgb.shape[2] != 3
            or image_rgb.dtype != np.uint8
        ):
            raise ValueError(
                "Expected a uint8 RGB image with three channels"
            )

        if image_rgb.shape[:2] != self.resolution:
            raise ValueError(
                f"Image dimensions {image_rgb.shape[:2]} "
                f"must match calibration {self.resolution}"
            )


    def _parse_detections(
        self,
        detections,
    ):
        """Convert ContourRunnerMulti output into label -> bbox."""

        if detections is None:
            return {}

        if not isinstance(detections, Sequence):
            raise TypeError(
                "detections must be a sequence of dictionaries"
            )

        image_height, image_width = self.resolution
        parsed = {}

        for index, item in enumerate(detections):
            if not isinstance(item, Mapping):
                raise TypeError(
                    f"Detection {index} must be a dictionary"
                )

            label = item.get("label")

            if label not in self.mesh_paths:
                raise KeyError(
                    f"Detection label {label!r} has no configured mesh"
                )

            # if label in parsed:
            #     raise ValueError(
            #         f"Multiple detections use label {label!r}. "
            #         "The unique-marker design requires one detection "
            #         "per label."
            #     )

            bbox_value = item.get("detection")

            if bbox_value is None:
                bbox_value = item.get("bbox_modal")

            bbox = np.asarray(
                bbox_value,
                dtype=float,
            )

            # if (
            #     bbox.shape != (4,)
            #     or not np.isfinite(bbox).all()
            #     or bbox[2] <= bbox[0]
            #     or bbox[3] <= bbox[1]
            # ):
            #     raise ValueError(
            #         f"Detection {index} for {label!r} has "
            #         "an invalid xyxy bbox"
            #     )


            # if (
            #     bbox[0] < 0
            #     or bbox[1] < 0
            #     or bbox[2] > image_width
            #     or bbox[3] > image_height
            # ):
            #     raise ValueError(
            #         f"Detection bbox for {label!r} is outside "
            #         f"the image: {bbox.tolist()}"
            #     )

            parsed[label] = bbox.copy()
        return parsed


    def _make_observation(self, image_rgb):
        return ObservationTensor.from_numpy(
            rgb=image_rgb,
            depth=None,
            K=self.K,
        ).cuda()


    def _make_detections(self, labels, bbox_by_label):
        object_data = [
            ObjectData(
                label=label,
                bbox_modal=bbox_by_label[
                    label
                ],
            )
            for label in labels
        ]

        return (make_detections_from_object_data(object_data).cuda())


    @staticmethod
    def calculate_iou(a, b):
        if a is None or b is None:
            return 0.0

        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)

        if a.shape != (4,) or b.shape != (4,):
            return 0.0

        if (
            not np.isfinite(a).all()
            or not np.isfinite(b).all()
        ):
            return 0.0

        intersection = np.maximum(
            0.0,
            np.minimum(a[2:], b[2:])
            - np.maximum(a[:2], b[:2]),
        )

        overlap = float(
            np.prod(intersection)
        )
        area_a = float(
            np.prod(
                np.maximum(
                    0.0,
                    a[2:] - a[:2],
                )
            )
        )
        area_b = float(
            np.prod(
                np.maximum(
                    0.0,
                    b[2:] - b[:2],
                )
            )
        )
        union = area_a + area_b - overlap

        return (
            overlap / union
            if union > 0
            else 0.0
        )


    def _project_bbox(
        self,
        label,
        pose_matrix,
    ):
        points = self.mesh_points[label]
        pose = np.asarray(
            pose_matrix,
            dtype=float,
        )

        if pose.shape != (4, 4):
            raise ValueError(
                "pose_matrix must have shape (4, 4)"
            )

        if not np.isfinite(pose).all():
            raise ValueError(
                "pose_matrix contains non-finite values"
            )

        rotation = pose[:3, :3]
        translation = pose[:3, 3]

        camera_points = (
            points @ rotation.T
            + translation
        )

        if np.any(
            camera_points[:, 2] <= 1e-6
        ):
            raise ValueError(
                f"Mesh {label!r} reaches or crosses "
                "the camera plane"
            )

        homogeneous_pixels = (
            camera_points @ self.K.T
        )

        pixels = (
            homogeneous_pixels[:, :2]
            / homogeneous_pixels[:, 2:3]
        )

        if not np.isfinite(pixels).all():
            raise ValueError(
                f"Projection for {label!r} is non-finite"
            )

        return np.concatenate(
            (
                pixels.min(axis=0),
                pixels.max(axis=0),
            )
        )


    @staticmethod
    def _pose_values(
        pose_matrix,
    ):
        rotation = pose_matrix[:3, :3].astype(
            float
        )
        translation = pose_matrix[:3, 3].astype(
            float
        )

        sy = np.sqrt(
            rotation[0, 0] ** 2
            + rotation[1, 0] ** 2
        )
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(
                rotation[2, 1],
                rotation[2, 2],
            )
            pitch = np.arctan2(
                -rotation[2, 0],
                sy,
            )
            yaw = np.arctan2(
                rotation[1, 0],
                rotation[0, 0],
            )
        else:
            roll = np.arctan2(
                -rotation[1, 2],
                rotation[1, 1],
            )
            pitch = np.arctan2(
                -rotation[2, 0],
                sy,
            )
            yaw = 0.0

        translation_m = tuple(
            float(value)
            for value in translation
        )

        euler_xyz_deg = tuple(
            float(value)
            for value in np.degrees(
                [roll, pitch, yaw]
            )
        )

        return (
            translation_m,
            euler_xyz_deg,
        )


    @staticmethod
    def _normalized_pose_state(label, pose_tensor):
        """Keep only data needed for the next refinement pass."""
        infos = pd.DataFrame([
            {
                "label": label,
                "batch_im_id": 0,
                "instance_id": 0,
            }
        ])
        return PandasTensorCollection(infos=infos, poses=pose_tensor.detach().clone())


    def _drop_tracking(
        self,
        label,
    ):
        self.previous_pose_estimates.pop(
            label,
            None,
        )
        self.poses.pop(
            label,
            None,
        )


    def _consume_output(
        self,
        output,
        expected_labels,
        bbox_by_label,
    ):
        """Validate and store one output per expected label."""

        results = {}
        accepted_labels = set()

        if output is None or len(output) == 0:
            for label in expected_labels:
                self._drop_tracking(label)

            return results, accepted_labels

        output_labels = (
            output.infos["label"]
            .astype(str)
            .to_numpy()
        )

        for label in expected_labels:
            indices = np.flatnonzero(
                output_labels == label
            )

            if len(indices) != 1:
                self._drop_tracking(label)
                continue

            output_index = int(indices[0])

            pose_tensor = output.poses[
                output_index:output_index + 1
            ]

            pose_is_valid = (
                torch.isfinite(
                    pose_tensor
                ).all().item()
                and float(
                    pose_tensor[0, 2, 3].item()
                ) > 0.0
            )

            if not pose_is_valid:
                self._drop_tracking(label)
                continue

            pose_matrix = (
                pose_tensor[0]
                .detach()
                .cpu()
                .numpy()
                .copy()
            )

            self.previous_pose_estimates[label] = (
                self._normalized_pose_state(
                    label,
                    pose_tensor,
                )
            )
            self.poses[label] = pose_matrix
            accepted_labels.add(label)

            try:
                projected_bbox = (
                    self._project_bbox(
                        label,
                        pose_matrix,
                    )
                )
            except ValueError:
                projected_bbox = None

            # (
            #     translation_m,
            #     euler_xyz_deg,
            # ) = self._pose_values(
            #     pose_matrix
            # )

            results[label] = MarkerPose(
                label=label,
                pose_matrix=pose_matrix,
                # translation_m=translation_m,
                # euler_xyz_deg=euler_xyz_deg,
                detection_bbox=(
                    bbox_by_label[label].copy()
                ),
                projected_bbox=projected_bbox,
            )
        return results, accepted_labels


    def reset_tracking(
        self,
        label=None,
    ):
        """Reset one marker, or all markers when label is None."""

        if label is None:
            self.previous_pose_estimates.clear()
            self.poses.clear()
            self.last_detections.clear()
            return

        if label not in self.mesh_paths:
            raise KeyError(
                f"Unknown marker label {label!r}"
            )

        self._drop_tracking(label)
        self.last_detections.pop(
            label,
            None,
        )


    def mpose_bboxes(self):
        """Return projected bboxes for all current poses."""
        projected = {}

        for label, pose_matrix in self.poses.items():
            try:
                projected[label] = (
                    self._project_bbox(
                        label,
                        pose_matrix,
                    )
                )
            except ValueError:
                continue
        return projected


    @torch.no_grad()
    def estimate(
        self,
        image_rgb,
        detections,
    ):
        """Estimate every detected configured marker.

        Args:
            image_rgb:
                Current calibrated uint8 RGB image.

            detections:
                Current ContourRunnerMulti output, for example::

                    [
                        {
                            "label": "bot_marker",
                            "detection": [x1, y1, x2, y2],
                        },
                        {
                            "label": "arena_marker",
                            "detection": [x1, y1, x2, y2],
                        },
                    ]

        Returns:
            Dictionary mapping marker label to MarkerPose.

            Only markers with a valid pose in this frame are returned.
        """

        if self._closed:
            raise RuntimeError(
                "MegaPoseRunnerMulti is closed"
            )

        self._validate_image(image_rgb)

        bbox_by_label = self._parse_detections(
            detections
        )

        self.last_detections = {
            label: bbox.copy()
            for label, bbox in bbox_by_label.items()
        }

        # Under the unique-color design, losing a labeled contour means
        # that marker's tracking state is no longer trusted.
        for label in list(
            self.previous_pose_estimates
        ):
            if label not in bbox_by_label:
                self._drop_tracking(label)

        if not bbox_by_label:
            return {}

        tracked_labels = []
        initialization_labels = []

        for label, bbox in bbox_by_label.items():
            previous_pose = self.poses.get(
                label
            )

            if previous_pose is None:
                initialization_labels.append(
                    label
                )
                continue

            try:
                predicted_bbox = (
                    self._project_bbox(
                        label,
                        previous_pose,
                    )
                )
            except ValueError:
                predicted_bbox = None

            if (
                self.calculate_iou(
                    bbox,
                    predicted_bbox,
                )
                >= self.iou_threshold
            ):
                tracked_labels.append(label)
            else:
                self._drop_tracking(label)
                initialization_labels.append(
                    label
                )

        observation = self._make_observation(
            image_rgb
        )
        results = {}

        # Refine all still-valid tracks in one MegaPose batch.
        if tracked_labels:
            tracked_input = concatenate([
                self.previous_pose_estimates[
                    label
                ]
                for label in tracked_labels
            ])

            predictions, _ = (
                self.pose_estimator.forward_refiner(
                    observation,
                    data_TCO_input=tracked_input,
                    n_iterations=1,
                    keep_all_outputs=False,
                )
            )

            tracked_output = predictions[
                "iteration=1"
            ]

            (
                tracked_results,
                accepted_tracked,
            ) = self._consume_output(
                tracked_output,
                tracked_labels,
                bbox_by_label,
            )

            results.update(
                tracked_results
            )

            # If refinement failed for a marker, attempt current-frame
            # reinitialization from its contour bbox.
            for label in tracked_labels:
                if label not in accepted_tracked:
                    initialization_labels.append(
                        label
                    )

        # Remove duplicates while preserving detection order.
        initialization_labels = list(
            dict.fromkeys(
                initialization_labels
            )
        )

        # Initialize all new/lost markers together in one coarse pass.
        if initialization_labels:
            megapose_detections = (
                self._make_detections(
                    initialization_labels,
                    bbox_by_label,
                )
            )

            parameters = dict(
                self.model_info[
                    "inference_parameters"
                ]
            )
            parameters["run_depth_refiner"] = False
            parameters["bsz_images"] = (
                self.megapose_batch_size
            )

            initialized_output, _ = (
                self.pose_estimator.run_inference_pipeline(
                    observation,
                    detections=megapose_detections,
                    **parameters,
                )
            )

            (
                initialized_results,
                _,
            ) = self._consume_output(
                initialized_output,
                initialization_labels,
                bbox_by_label,
            )

            results.update(
                initialized_results
            )

        return results


    def close(self):
        """Stop MegaPose's renderer worker processes."""

        if self._closed:
            return

        self._closed = True
        self.reset_tracking()

        seen = set()

        for model_name in (
            "coarse_model",
            "refiner_model",
        ):
            model = getattr(
                self.pose_estimator,
                model_name,
                None,
            )
            renderer = getattr(
                model,
                "renderer",
                None,
            )

            if (
                renderer is None
                or id(renderer) in seen
            ):
                continue

            seen.add(id(renderer))
            renderer.stop()