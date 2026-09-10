from pathlib import Path
import numpy as np
import cv2
import json
import torch

from megapose.datasets.object_dataset import RigidObject, RigidObjectDataset
from megapose.datasets.scene_dataset import ObjectData, CameraData
from megapose.inference.types import ObservationTensor
from megapose.inference.utils import make_detections_from_object_data
from megapose.panda3d_renderer import Panda3dLightData
from megapose.panda3d_renderer.panda3d_scene_renderer import Panda3dSceneRenderer
from custom_load_model import NAMED_MODELS, load_named_model
from megapose.lib3d.transform import Transform
from megapose.utils.conversion import convert_scene_observation_to_panda3d
from configs.config import OUT_RES


class MegaPoseRunner:
    def __init__(self, mesh_path: Path, label: str, model_name: str, K_path: Path):
        self.label = label
        self.model_name = model_name
        self.model_info = NAMED_MODELS[model_name]

        if not mesh_path.exists():
            raise Exception('Input mesh file missing')
        
        obj = RigidObject(label=label, mesh_path=mesh_path, mesh_units="m")
        self.object_dataset = RigidObjectDataset([obj])

        if not K_path.exists():
            raise Exception('Camera calibration file missing')

        K_json = json.loads(K_path.read_text(encoding='utf-8'))
        img_res = K_json['img_size']
        assert img_res == list(OUT_RES)

        mpose_format = {
            "K": K_json["camera_matrix"],
            "resolution": list(reversed(img_res))
        }
        K_data = json.dumps(mpose_format, indent=2)

        cam_data = CameraData.from_json(K_data)
        self.K = cam_data.K
        self.resolution = cam_data.resolution
        self.camera_data = cam_data

        # Heavy model loaded once.
        self.pose_estimator = load_named_model(
            model_name, self.object_dataset, n_workers=12
        ).cuda()
        self.pose_estimator.eval()
        self.scene_renderer = Panda3dSceneRenderer(self.object_dataset)

        # Tracking variables
        self.poses = None
        self.detection = None

        # Stage II: keep the full GPU collection separately from display poses.
        self.reset_tracking()

    ######################### PRIVATE FUNCTIONS ##########################
    def _make_observation(self):
        return ObservationTensor.from_numpy(
            rgb=self.img,
            depth=None,
            K=self.K
        ).cuda()


    def _make_full_frame_detection(self):
        height, width = self.img.shape[:2]
        object_data = [
            ObjectData(
                label=self.label,
                bbox_modal=np.array([0, 0, width, height], dtype=float),
            )
        ]
        return make_detections_from_object_data(object_data).cuda()

    def _current_pose_transform(self):
        if not self.poses:
            raise RuntimeError("No pose prediction available. Run estimate() first.")

        quat, trans = self.poses[0]["TWO"] #type: ignore
        quat = np.array(quat, dtype=float)
        trans = np.array(trans, dtype=float)
        return Transform(quat, trans)


    def _render_mesh_rgb(self):
        pose_transform = self._current_pose_transform()

        camera_data = CameraData(
            K=self.K,
            resolution=self.resolution,
            TWC=Transform(np.eye(4)),
        )
        object_datas = [ObjectData(label=self.label, TWO=pose_transform)]
        panda_camera_data, panda_object_datas = convert_scene_observation_to_panda3d(
            camera_data,
            object_datas,
        )

        light_datas = [
            Panda3dLightData(
                light_type="ambient",
                color=(1.0, 1.0, 1.0, 1.0),
            ),
        ]

        renderings = self.scene_renderer.render_scene(
            panda_object_datas,
            [panda_camera_data],
            light_datas,
            render_depth=False,
            render_binary_mask=False,
            render_normals=False,
            copy_arrays=True,
        )[0]
        return renderings.rgb


    ############################# PUBLIC METHODS #########################
    def reset_tracking(self):
        """Call before loading a fresh detection to restart initialization."""
        self.previous_pose_estimates = None
        self.tracking_active = False
        self.detection = None
        self.poses = []


    def load_detection(self, detection):
        """Load one current-frame box: left, top, right, bottom."""
        if detection is None or len(detection) == 0:
            return

        bbox = np.asarray(detection, dtype=float)
        if (
            bbox.shape != (4,)
            or not np.isfinite(bbox).all()
            or bbox[2] <= bbox[0]
            or bbox[3] <= bbox[1]
        ):
            raise ValueError('Expected a valid [left, top, right, bottom] bbox')

        object_data = [ObjectData(label=self.label, bbox_modal=bbox)]
        self.detection = make_detections_from_object_data(object_data).cuda()

    def save_predictions(self, pose_estimates):
        labels = pose_estimates.infos["label"]
        poses = pose_estimates.poses.detach().cpu().numpy()

        # These are TCO transforms. TWO is equivalent here because the
        # visualization camera's TWC is identity.
        object_data = [
            ObjectData(label=label, TWO=Transform(pose))
            for label, pose in zip(labels, poses)
        ]
        self.poses = [x.to_json() for x in object_data]

        T = object_data[0].TWO
        H = T.toHomogeneousMatrix()
        R = H[:3, :3].astype(float)
        t = H[:3, 3].astype(float)

        x = float(t[0])
        y = float(t[1])
        z = float(t[2])

        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0.0

        roll = float(np.degrees(roll))
        pitch = float(np.degrees(pitch))
        yaw = float(np.degrees(yaw))
        return x, y, z, roll, pitch, yaw

    def draw_mesh_overlay(self):
        if not hasattr(self, "img"):
            raise RuntimeError("No image loaded. Run estimate() first.")

        rgb_rendered = self._render_mesh_rgb()
        mask = np.any(rgb_rendered > 0, axis=-1)

        rgb_overlay = np.zeros_like(self.img, dtype=np.float32)
        rgb_overlay[~mask] = self.img[~mask] * 0.6 + 255 * 0.4
        rgb_overlay[mask] = rgb_rendered[mask] * 0.8 + 255 * 0.2
        rgb_overlay = rgb_overlay.astype(np.uint8)
        return cv2.cvtColor(rgb_overlay, cv2.COLOR_RGB2BGR)

    @torch.no_grad()
    def estimate(self, img: np.ndarray):
        """Accept calibrated-size uint8 RGB; return pose tuple or None.

        Initialization needs a detection from this frame. Tracking uses the
        previous GPU pose and one refinement iteration, without fresh scoring.
        Basic validity checks do not establish that tracking is accurate.
        """
        # Consume once so a box cannot accidentally initialize a later frame.
        detections = self.detection
        self.detection = None
        self.poses = []

        if (
            not isinstance(img, np.ndarray)
            or img.ndim != 3
            or img.shape[2] != 3
            or img.dtype != np.uint8
        ):
            raise ValueError('Expected a uint8 RGB image with three channels')
        if img.shape[:2] != tuple(self.resolution):
            raise ValueError(
                f'Image dimensions {img.shape[:2]} must match '
                f'calibration {tuple(self.resolution)} (height, width)'
            )

        self.img = img
        if not self.tracking_active and detections is None:
            return None

        observation = self._make_observation()
        if self.tracking_active:
            predictions, _ = self.pose_estimator.forward_refiner(
                observation,
                data_TCO_input=self.previous_pose_estimates,
                n_iterations=1,
                keep_all_outputs=False,
            )
            output = predictions["iteration=1"]
        else:
            parameters = dict(self.model_info["inference_parameters"])
            parameters["run_depth_refiner"] = False  # Monocular RGB only.
            output, _ = self.pose_estimator.run_inference_pipeline(
                observation, detections=detections, **parameters
            )

        # This runner supports exactly one object and one retained pose.
        if output is None or output.poses.shape != (1, 4, 4):
            self.reset_tracking()
            return None
        valid = torch.isfinite(output.poses).all() & (output.poses[:, 2, 3] > 0).all()
        if not valid.item():
            self.reset_tracking()
            return None

        self.previous_pose_estimates = output
        self.tracking_active = True
        return self.save_predictions(output)

    def draw_triaxis(self):
        T = self._current_pose_transform()
        H = T.toHomogeneousMatrix()
        R = H[:3, :3].astype(float)
        t = H[:3, 3].astype(float)

        rvec, _ = cv2.Rodrigues(R)
        tvec = t.reshape(3, 1)
        dist = np.zeros((5, 1), dtype=float)
        axis_length = 0.1

        img_bgr = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        cv2.drawFrameAxes(img_bgr, self.K, dist, rvec, tvec, axis_length) #type: ignore
        return img_bgr
