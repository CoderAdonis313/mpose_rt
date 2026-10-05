import json
import socket
from uuid import uuid4
import sys
from time import monotonic, perf_counter, time_ns
import numpy as np


class PoseSender:
    """Send the latest multi-marker pose over non-blocking localhost UDP.

    send() returns:
        True:
            Packet was sent.

        False:
            Packet construction or UDP transmission failed.

        None:
            Packet was skipped by the rate limiter.
    """

    UDP_MAX_PAYLOAD = 65_507

    def __init__(
        self,
        port,
        frame_id,
        send_hz=30.0,
        max_pose_age_ns=300_000_000,
    ):
        port = int(port)
        send_hz = float(send_hz)

        if not 1 <= port <= 65_535:
            raise ValueError(
                "UDP port must be between 1 and 65535"
            )

        if send_hz <= 0:
            raise ValueError(
                "UDP send frequency must be positive"
            )

        if max_pose_age_ns <= 0:
            raise ValueError(
                "Maximum pose age must be positive"
            )

        self.destination = (
            "127.0.0.1",
            port,
        )

        self.frame_id = str(frame_id)
        self.session_id = uuid4().hex

        self.send_period = 1.0 / send_hz
        self.max_pose_age_ns = int(
            max_pose_age_ns
        )

        self.packet_seq = 0
        self.failed_sends = 0
        self.skipped_sends = 0

        self.last_warning = -float("inf")
        self.next_send_time = 0.0
        self._closed = False

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        self.sock.setblocking(False)

    def _warn(self, message):
        now = perf_counter()

        if now - self.last_warning >= 2.0:
            print(
                f"UDP send failed: {message}",
                file=sys.stderr,
            )

            self.last_warning = now

    @staticmethod
    def _serialize_bbox(bbox, field_name):
        if bbox is None:
            return None

        array = np.asarray(
            bbox,
            dtype=float,
        )

        if (
            array.shape != (4,)
            or not np.isfinite(array).all()
        ):
            raise ValueError(
                f"{field_name} must be a finite "
                "four-value bbox"
            )

        return array.tolist()

    @staticmethod
    def _serialize_pose_matrix(matrix):
        matrix = np.asarray(
            matrix,
            dtype=float,
        )

        if matrix.shape != (4, 4):
            raise ValueError(
                "Pose matrix must have shape (4, 4)"
            )

        if not np.isfinite(matrix).all():
            raise ValueError(
                "Pose matrix contains non-finite values"
            )

        if not np.allclose(
            matrix[3],
            [0.0, 0.0, 0.0, 1.0],
            atol=1e-5,
        ):
            raise ValueError(
                "Pose matrix has an invalid final row"
            )

        if matrix[2, 3] <= 0:
            raise ValueError(
                "Pose translation is behind the camera"
            )

        return matrix.tolist()

    def _serialize_poses(
        self,
        pose_data,
        poses_are_fresh,
    ):
        if pose_data is None:
            return {}

        serialized = {}

        for label, marker_pose in (
            pose_data.poses.items()
        ):
            serialized[label] = {
                # This matrix transforms points from the
                # marker frame into the camera frame.
                "T_camera_object": (
                    self._serialize_pose_matrix(
                        marker_pose.pose_matrix
                    )
                ),

                "detection_bbox": (
                    self._serialize_bbox(
                        marker_pose.detection_bbox,
                        "detection_bbox",
                    )
                ),

                "projected_bbox": (
                    self._serialize_bbox(
                        marker_pose.projected_bbox,
                        "projected_bbox",
                    )
                ),

                # A stale matrix may still be transmitted for
                # diagnostics, but receivers must not treat it
                # as a current measurement.
                "valid": bool(poses_are_fresh),
            }

        return serialized

    def send(
        self,
        status,
        pose_data=None,
        force=False,
    ):
        """Send the newest pose packet.

        STARTING, RESET, STOPPED, and ERROR packets bypass the
        rate limiter. Normal pose packets are limited to send_hz.
        """

        if self._closed:
            return False

        status = str(status)

        force = force or status in {
            "STARTING",
            "RESET",
            "STOPPED",
            "ERROR",
        }

        monotonic_now = perf_counter()

        if (
            not force
            and monotonic_now < self.next_send_time
        ):
            self.skipped_sends += 1
            return None

        self.next_send_time = (
            monotonic_now + self.send_period
        )

        send_time_ns = time_ns()

        frame_index = None
        image_time_ns = None
        result_time_ns = None
        inference_seconds = None

        source_age_ns = None
        result_age_ns = None
        source_status = status

        has_poses = (
            pose_data is not None
            and bool(pose_data.poses)
        )

        if pose_data is not None:
            frame_index = pose_data.frame_index
            image_time_ns = pose_data.image_time_ns
            result_time_ns = pose_data.result_time_ns
            inference_seconds = (
                pose_data.inference_seconds
            )
            source_status = pose_data.status

            source_age_ns = max(
                0,
                send_time_ns - image_time_ns,
            )

            result_age_ns = max(
                0,
                send_time_ns - result_time_ns,
            )

        poses_are_fresh = (
            has_poses
            and source_age_ns is not None
            and source_age_ns
            <= self.max_pose_age_ns
        )

        effective_status = status

        if has_poses and not poses_are_fresh:
            effective_status = "STALE"

        # TRACKING means all requested markers succeeded.
        # PARTIAL can still contain valid poses for some markers.
        valid = (
            poses_are_fresh
            and source_status
            in {"TRACKING", "PARTIAL"}
        )

        sequence = self.packet_seq
        self.packet_seq += 1

        try:
            poses = self._serialize_poses(
                pose_data,
                poses_are_fresh=valid,
            )

            packet = {
                "schema_version": 1,
                "session_id": self.session_id,
                "packet_seq": sequence,

                "send_time_ns": send_time_ns,
                "frame_index": frame_index,
                "image_time_ns": image_time_ns,
                "result_time_ns": result_time_ns,
                "timestamp_source": "host_read",

                "frame_id": self.frame_id,

                "status": effective_status,
                "source_status": source_status,
                "valid": valid,
                "complete": (
                    valid
                    and source_status == "TRACKING"
                ),

                "source_age_ns": source_age_ns,
                "result_age_ns": result_age_ns,

                "poses": poses,
                "pose_count": len(poses),
                "inference_seconds": (
                    inference_seconds
                ),
            }

            payload = json.dumps(
                packet,
                allow_nan=False,
                separators=(",", ":"),
            ).encode("utf-8")

            if len(payload) > self.UDP_MAX_PAYLOAD:
                raise ValueError(
                    f"UDP payload is too large: "
                    f"{len(payload)} bytes"
                )

            self.sock.sendto(
                payload,
                self.destination,
            )

            return True

        except (
            OSError,
            TypeError,
            ValueError,
            OverflowError,
        ) as error:
            self.failed_sends += 1
            self._warn(error)
            return False

    def close(self):
        if self._closed:
            return

        self._closed = True
        self.sock.close()