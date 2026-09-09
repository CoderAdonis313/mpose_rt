from pathlib import Path
import numpy as np
import cv2
import json

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

        # K_data = open(K_path, 'r', encoding='utf-8').read()
        # Read calibdb format
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

        # heavy model loaded once
        self.pose_estimator = load_named_model(model_name, self.object_dataset, n_workers=12).cuda()
        self.scene_renderer = Panda3dSceneRenderer(self.object_dataset)


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
                bbox_modal=np.ndarray([0, 0, width, height], dtype=float),
            )
        ]

        return make_detections_from_object_data(object_data).cuda()


    def _current_pose_transform(self):
        if not hasattr(self, "poses") or not self.poses:
            raise RuntimeError("No pose prediction available. Run estimate() first.")

        quat, trans = self.poses[0]["TWO"]  # type: ignore[index]
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
    def load_detection(self, detection):
        if detection is None or len(detection) == 0:
            raise RuntimeError('No bbox detection here')

        object_data = [
            ObjectData(
                label=self.label,
                bbox_modal=np.array(detection, dtype=float),
            )
        ]

        self.detection = make_detections_from_object_data(object_data).cuda()


    def save_predictions(self, pose_estimates):
        labels = pose_estimates.infos["label"]
        poses = pose_estimates.poses.cpu().numpy()

        object_data = [
            ObjectData(label=label, TWO=Transform(pose)) for label, pose in zip(labels, poses)
        ]

        json_data = [x.to_json() for x in object_data]
        self.poses = json_data

        T = object_data[0].TWO
        H = T.toHomogeneousMatrix() #type: ignore

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


    def estimate(self, img: np.ndarray):
        self.img = img

        observation = self._make_observation()
        detections = self.detection

        output, _ = self.pose_estimator.run_inference_pipeline(
            observation, detections=detections, **self.model_info["inference_parameters"]
        )
        return self.save_predictions(output)


    def draw_triaxis(self):
        pose = self.poses[0]["TWO"]
        quat, trans = pose #type: ignore
        quat = np.array(quat, dtype=float)
        trans = np.array(trans, dtype=float)

        T = Transform(quat, trans)
        H = T.toHomogeneousMatrix()
        R = H[:3, :3].astype(float)
        t = H[:3, 3].astype(float)
        # print('INFO: RHS OR LHS', np.linalg.det(R))

        rvec, _ = cv2.Rodrigues(R)
        tvec = t.reshape(3, 1)

        dist = np.zeros((5, 1), dtype=float)
        axis_length = 0.1

        img_bgr = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        cv2.drawFrameAxes(img_bgr, self.K, dist, rvec, tvec, axis_length) #type: ignore
        return img_bgr
