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
import trimesh


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
        mesh = trimesh.load(
            str(mesh_path),
            force="mesh",
            process=False,
        )

        # force="mesh" requests scene conversion; narrow the type explicitly
        # before accessing vertices (also resolves static type-checker errors).
        if not isinstance(mesh, trimesh.Trimesh):
            raise TypeError(f"Expected a triangle mesh, got {type(mesh).__name__}")
        self.mesh_points = np.asarray(mesh.vertices, dtype=np.float32)
        if self.mesh_points.size == 0 or not np.isfinite(self.mesh_points).all():
            raise ValueError("Mesh must contain finite, nonempty vertices")

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
        if self.poses is None:
            raise RuntimeError("No pose prediction available. Run estimate() first.")
        # self.poses is a single (4, 4) TCO matrix, not a JSON list.
        return Transform(self.poses)


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


    def _transform_mesh_points(self, mesh_points, pose):
        """mesh_points: (N, 3), pose: (4, 4) TCO; output: (N, 3)."""
        points = np.asarray(mesh_points, dtype=float)
        pose = np.asarray(pose, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("mesh_points must be a nonempty (N, 3) array")
        if pose.shape != (4, 4):
            raise ValueError("pose must be a (4, 4) object-to-camera matrix")
        if not np.isfinite(points).all() or not np.isfinite(pose).all():
            raise ValueError("Mesh and pose must contain finite values")
        R = pose[:3, :3]
        t = pose[:3, 3]
        return points @ R.T + t


    def _project_points(self, mesh_points_camera, K):
        """Project (N, 3) camera points to (N, 2) undistorted pixels."""
        points = np.asarray(mesh_points_camera, dtype=float)
        K = np.asarray(K, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("Camera points must be a nonempty (N, 3) array")
        if K.shape != (3, 3):
            raise ValueError("K must have shape (3, 3)")
        if not np.isfinite(points).all() or not np.isfinite(K).all():
            raise ValueError("Camera points and K must contain finite values")
        if np.any(points[:, 2] <= 1e-6):
            raise ValueError("Mesh reaches or crosses the camera plane")
        pixels_h = points @ K.T
        if np.any(np.abs(pixels_h[:, 2]) <= 1e-12):
            raise ValueError("Invalid homogeneous projection denominator")
        pixels = pixels_h[:, :2] / pixels_h[:, 2:3]
        if not np.isfinite(pixels).all():
            raise ValueError("Projection produced nonfinite pixels")
        return pixels


    def _bounding_rectangle(self, projected_pixels):
        """Return unclipped float [left, top, right, bottom]."""
        pixels = np.asarray(projected_pixels, dtype=float)
        if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) == 0:
            raise ValueError("Pixels must be a nonempty (N, 2) array")
        if not np.isfinite(pixels).all():
            raise ValueError("Pixels must contain finite values")
        return np.concatenate((pixels.min(axis=0), pixels.max(axis=0)))


    def _process_estimates(self, pose_estimates):
        """Store one CPU TCO matrix; return meters and Euler angles in degrees."""
        self.poses = pose_estimates.poses[0].detach().cpu().numpy().copy()
        H = self.poses
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


    ############################# PUBLIC METHODS #########################
    def mpose_bboxes(self):
        """Return [left, top, right, bottom] pixels, or [] if unavailable.

        Before estimate(): projects the last accepted pose.
        After estimate(): projects the newly estimated pose.
        The box is not clipped to image bounds. Assumes undistorted RGB.
        """
        if self.poses is None:
            return []

        # Projection is undefined if the mesh crosses the camera plane.
        # Report no usable box rather than interrupting the video loop.
        mesh_points_camera = self._transform_mesh_points(self.mesh_points, self.poses)
        projected_pixels = self._project_points(mesh_points_camera, self.K)
        bboxs = self._bounding_rectangle(projected_pixels)
        return bboxs


    def reset_tracking(self):
        """Call before loading a fresh detection to restart initialization."""
        self.previous_pose_estimates = None
        self.tracking_active = False
        self.detection = None
        self.poses = None  # Single CPU (4, 4) TCO matrix, when available.


    def load_detection(self, detection):
        """Load one current-frame box: left, top, right, bottom."""
        if detection is None or len(detection) == 0:
            self.detection = None  # Missing/invalid input must not retain an old box.
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
        self.poses = None

        if (
            not isinstance(img, np.ndarray)
            or img.ndim != 3
            or img.shape[2] != 3
            or img.dtype != np.uint8
        ):
            raise ValueError('Expected a uint8 RGB image with three channels')
        if img.shape[:2] != tuple(self.resolution):             #type: ignore
            raise ValueError(
                f'Image dimensions {img.shape[:2]} must match '
                f'calibration {tuple(self.resolution)} (height, width)' #type: ignore
            )

        self.img = img
        if not self.tracking_active and detections is None:
            return None

        observation = self._make_observation()
        if self.tracking_active:
            predictions, _ = self.pose_estimator.forward_refiner(
                observation,
                data_TCO_input=self.previous_pose_estimates,    #type: ignore
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
        poses_list = self._process_estimates(output)
        return poses_list


    # def draw_triaxis(self):
    #     T = self._current_pose_transform()
    #     H = T.toHomogeneousMatrix()
    #     R = H[:3, :3].astype(float)
    #     t = H[:3, 3].astype(float)

    #     rvec, _ = cv2.Rodrigues(R)
    #     tvec = t.reshape(3, 1)
    #     dist = np.zeros((5, 1), dtype=float)
    #     axis_length = 0.1

    #     img_bgr = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
    #     cv2.drawFrameAxes(img_bgr, self.K, dist, rvec, tvec, axis_length) #type: ignore
    #     return img_bgr


    def draw_triaxis(self):
        if self.poses is None:
            raise RuntimeError("No pose prediction available. Run estimate() first.")

        R = self.poses[:3, :3].astype(float)
        t = self.poses[:3, 3].astype(float)

        rvec, _ = cv2.Rodrigues(R)
        tvec = t.reshape(3, 1)
        dist = np.zeros((5, 1), dtype=float)
        axis_length = 0.1

        img_bgr = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        cv2.drawFrameAxes(img_bgr, self.K, dist, rvec, tvec, axis_length) #type: ignore
        return img_bgr