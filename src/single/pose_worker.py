"""Latest-only MegaPose inference worker."""

from dataclasses import dataclass
import threading
from time import perf_counter, time_ns
from typing import Optional, Tuple
import numpy as np


@dataclass(frozen=True)
class PoseInput:
    frame_index: int
    image_time_ns: int
    image_rgb: np.ndarray
    bbox: Optional[np.ndarray]


@dataclass(frozen=True)
class CapturedPose:
    frame_index: int
    image_time_ns: int
    result_time_ns: int
    status: str
    bbox: Optional[np.ndarray]
    pose_matrix: Optional[np.ndarray]
    pose_values: Optional[Tuple[float, ...]]
    inference_seconds: float


def calculate_iou(a, b):
    if a is None or b is None:
        return 0.0

    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    if a.shape != (4,) or b.shape != (4,):
        return 0.0
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        return 0.0

    intersection = np.maximum(
        0.0,
        np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2]),
    )
    overlap = float(np.prod(intersection))
    area_a = float(np.prod(np.maximum(0.0, a[2:] - a[:2])))
    area_b = float(np.prod(np.maximum(0.0, b[2:] - b[:2])))
    union = area_a + area_b - overlap
    return overlap / union if union > 0 else 0.0


class LatestPoseInference:
    """Run MegaPose on the newest submitted frame.

    Only one input waits while inference is running. Submitting a newer input
    replaces the waiting input, preventing an inference backlog.
    """

    def __init__(self, mpose, iou_threshold):
        self.mpose = mpose
        self.iou_threshold = float(iou_threshold)

        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._reset_event = threading.Event()

        self._pending: Optional[PoseInput] = None
        self._latest: Optional[CapturedPose] = None
        self._error: Optional[Exception] = None

        self._last_submitted_index = -1
        self._started = False
        self._closed = False

        self.dropped_inputs = 0
        self.processed_inputs = 0

        self._thread = threading.Thread(
            target=self._inference_loop,
            name="pose-capture",
            daemon=True,
        )


    def start(self):
        if self._closed:
            raise RuntimeError("Cannot restart a closed pose worker")

        if not self._started:
            self._started = True
            self._thread.start()
        return self


    def submit(self, frame_index, image_time_ns, image_rgb, bbox):
        """Submit work without waiting for MegaPose.
        If inference is busy, this replaces the previously waiting input.
        """

        if not self._started:
            raise RuntimeError("Pose worker must be started before submitting")

        if self._closed:
            raise RuntimeError("Pose worker is closed")

        bbox_array = None
        if bbox is not None:
            bbox_array = np.asarray(bbox, dtype=float,).copy()

        job = PoseInput(
            frame_index=int(frame_index),
            image_time_ns=int(image_time_ns),
            # cvtColor creates a new array in the caller. Do not mutate this
            # image after submitting it.
            image_rgb=image_rgb,
            bbox=bbox_array,
        )

        with self._condition:
            self._raise_if_failed_locked()
            if frame_index <= self._last_submitted_index:
                return False

            self._last_submitted_index = frame_index
            if self._pending is not None:
                self.dropped_inputs += 1

            self._pending = job
            self._condition.notify_all()
        return True
    

    def latest(self):
        """Return immediately with the latest pose result."""

        with self._condition:
            self._raise_if_failed_locked()
            return self._latest


    def request_reset(self):
        """Clear published state and reset tracking inside the worker."""

        with self._condition:
            self._latest = None
            self._pending = None
            self._reset_event.set()
            self._condition.notify_all()


    def _publish(self, result):
        with self._condition:
            # A reset requested during inference invalidates this result.
            if self._reset_event.is_set():
                return

            self._latest = result
            self.processed_inputs += 1
            self._condition.notify_all()


    def _make_no_pose_result(self, job, status, started):
        return CapturedPose(
            frame_index=job.frame_index,
            image_time_ns=job.image_time_ns,
            result_time_ns=time_ns(),
            status=status,
            bbox=(
                None
                if job.bbox is None
                else job.bbox.copy()
            ),
            pose_matrix=None,
            pose_values=None,
            inference_seconds=perf_counter() - started,
        )


    def _estimate(self, job: PoseInput):
        started = perf_counter()

        if job.bbox is None:
            self.mpose.reset_tracking()

            self._publish(
                self._make_no_pose_result(job, status="NO_TARGET", started=started)
            )
            return

        if self.mpose.tracking_active:
            try:
                predicted_bbox = self.mpose.mpose_bboxes()
            except ValueError:
                predicted_bbox = []

            if (
                calculate_iou(
                    job.bbox,
                    predicted_bbox,
                )
                < self.iou_threshold
            ):
                self.mpose.reset_tracking()

        if not self.mpose.tracking_active:
            self.mpose.load_detection(job.bbox)
        pose_values = self.mpose.estimate(job.image_rgb)

        if pose_values is None:
            self.mpose.reset_tracking()

            self._publish(
                self._make_no_pose_result(
                    job,
                    status="NO_POSE",
                    started=started,
                )
            )
            return

        pose_matrix = self.mpose.poses.copy()
        self._publish(
            CapturedPose(
                frame_index=job.frame_index,
                image_time_ns=job.image_time_ns,
                result_time_ns=time_ns(),
                status="TRACKING",
                bbox=job.bbox.copy(),
                pose_matrix=pose_matrix,
                pose_values=tuple(pose_values),
                inference_seconds=perf_counter() - started,
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

                if job is None:
                    continue

                self._estimate(job)

        except Exception as error:
            with self._condition:
                self._error = error
                self._condition.notify_all()


    def _raise_if_failed_locked(self):
        if self._error is not None:
            raise RuntimeError("MegaPose inference worker failed") from self._error


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