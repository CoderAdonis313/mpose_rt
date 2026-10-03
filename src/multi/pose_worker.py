"""Latest-only multi-marker MegaPose inference worker."""

from collections.abc import Mapping
from dataclasses import dataclass
import threading
from time import perf_counter, time_ns
from typing import Dict, Optional, Tuple
import numpy as np
from src.multi.mpose_runner_multi import MarkerPose


@dataclass(frozen=True)
class PoseInput:
    frame_index: int
    image_time_ns: int
    image_rgb: np.ndarray
    detections: Tuple[dict, ...]


@dataclass(frozen=True)
class CapturedPoses:
    frame_index: int
    image_time_ns: int
    result_time_ns: int
    status: str
    poses: Dict[str, MarkerPose]
    inference_seconds: float


class LatestPoseInference:
    """Run multi-marker MegaPose on only the newest submitted frame.

    While MegaPose is busy, one pending input is retained. A newer input
    replaces the pending input so an inference backlog cannot develop.
    """

    def __init__(self, mpose):
        self.mpose = mpose

        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._reset_event = threading.Event()

        self._pending: Optional[PoseInput] = None
        self._latest: Optional[CapturedPoses] = None
        self._error: Optional[Exception] = None

        self._last_submitted_index = -1
        self._started = False
        self._closed = False

        self.dropped_inputs = 0
        self.processed_inputs = 0

        self._thread = threading.Thread(
            target=self._inference_loop,
            name="multi-pose-inference",
            daemon=True,
        )


    def start(self):
        if self._closed:
            raise RuntimeError(
                "Cannot restart a closed pose worker"
            )

        if not self._started:
            self._started = True
            self._thread.start()

        return self


    @staticmethod
    def _copy_detections(detections):
        """Make a worker-owned snapshot of the detection list."""

        if detections is None:
            return tuple()

        copied = []

        for index, item in enumerate(detections):
            if not isinstance(item, Mapping):
                raise TypeError(
                    f"Detection {index} must be a dictionary"
                )

            copied_item = dict(item)

            # Support either name accepted by MegaPoseRunnerMulti.
            for bbox_key in ("detection", "bbox_modal"):
                bbox = copied_item.get(bbox_key)

                if bbox is not None:
                    copied_item[bbox_key] = np.asarray(
                        bbox,
                        dtype=float,
                    ).copy()

            copied.append(copied_item)

        return tuple(copied)


    def submit(
        self,
        frame_index,
        image_time_ns,
        image_rgb,
        detections,
    ):
        """Submit a frame without waiting for MegaPose."""

        if not self._started:
            raise RuntimeError(
                "Pose worker must be started before submitting"
            )

        if self._closed:
            raise RuntimeError("Pose worker is closed")

        job = PoseInput(
            frame_index=int(frame_index),
            image_time_ns=int(image_time_ns),

            # cvtColor creates a new image in the caller. The caller must
            # not modify this array after submitting it.
            image_rgb=image_rgb,

            detections=self._copy_detections(
                detections
            ),
        )

        with self._condition:
            self._raise_if_failed_locked()

            if frame_index <= self._last_submitted_index:
                return False

            self._last_submitted_index = frame_index

            if self._pending is not None:
                self.dropped_inputs += 1

            # Replace the older waiting job.
            self._pending = job
            self._condition.notify_all()

        return True


    def latest(self):
        """Immediately return the latest completed result."""

        with self._condition:
            self._raise_if_failed_locked()
            return self._latest


    def request_reset(self):
        """Request that all tracking state be reset by the worker."""

        with self._condition:
            self._latest = None
            self._pending = None
            self._reset_event.set()
            self._condition.notify_all()


    @staticmethod
    def _copy_pose(pose):
        """Detach a MarkerPose result from mutable runner state."""

        pose_matrix = np.asarray(
            pose.pose_matrix,
            dtype=float,
        ).copy()

        detection_bbox = np.asarray(
            pose.detection_bbox,
            dtype=float,
        ).copy()

        projected_bbox = None

        if pose.projected_bbox is not None:
            projected_bbox = np.asarray(
                pose.projected_bbox,
                dtype=float,
            ).copy()

        return MarkerPose(
            label=pose.label,
            pose_matrix=pose_matrix,
            detection_bbox=detection_bbox,
            projected_bbox=projected_bbox,
        )


    def _publish(self, result):
        with self._condition:
            # A reset requested during inference invalidates this result.
            if self._reset_event.is_set():
                return

            self._latest = result
            self.processed_inputs += 1
            self._condition.notify_all()


    def _estimate(self, job):
        started = perf_counter()

        # MegaPoseRunnerMulti performs:
        #   - detection parsing
        #   - per-label IoU checking
        #   - refinement
        #   - initialization
        #   - per-label tracking reset
        pose_results = self.mpose.estimate(
            job.image_rgb,
            job.detections,
        )

        poses = {
            label: self._copy_pose(marker_pose)
            for label, marker_pose
            in pose_results.items()
        }

        detected_labels = {
            item.get("label")
            for item in job.detections
        }

        if not job.detections:
            status = "NO_DETECTIONS"
        elif not poses:
            status = "NO_POSES"
        elif set(poses) == detected_labels:
            status = "TRACKING"
        else:
            status = "PARTIAL"

        self._publish(
            CapturedPoses(
                frame_index=job.frame_index,
                image_time_ns=job.image_time_ns,
                result_time_ns=time_ns(),
                status=status,
                poses=poses,
                inference_seconds=(
                    perf_counter() - started
                ),
            )
        )


    def _inference_loop(self):
        try:
            while not self._stop_event.is_set():
                with self._condition:
                    while (
                        self._pending is None
                        and not self._reset_event.is_set()
                        and not self._stop_event.is_set()
                    ):
                        self._condition.wait()

                    if self._stop_event.is_set():
                        break

                    reset_requested = (
                        self._reset_event.is_set()
                    )
                    self._reset_event.clear()

                    job = self._pending
                    self._pending = None

                if reset_requested:
                    self.mpose.reset_tracking()

                if job is not None:
                    self._estimate(job)


        except Exception as error:
            with self._condition:
                self._error = error
                self._condition.notify_all()


    def _raise_if_failed_locked(self):
        if self._error is not None:
            raise RuntimeError(
                "MegaPose inference worker failed"
            ) from self._error


    def close(self, join_timeout=None):
        if self._closed:
            return

        self._closed = True
        self._stop_event.set()

        with self._condition:
            self._pending = None
            self._condition.notify_all()

        if self._started:
            self._thread.join(join_timeout)

            if self._thread.is_alive():
                raise RuntimeError("MegaPose worker did not stop")


    def __enter__(self):
        return self.start()


    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback,
    ):
        self.close()