"""Threaded OpenCV capture using latest-frame semantics."""

from dataclasses import dataclass
import threading
from time import monotonic, time_ns
from typing import Optional, Tuple, Union
import cv2
import numpy as np


@dataclass(frozen=True)
class CapturedFrame:
    """A frame with its capture sequence and host timestamp."""

    index: int
    image_time_ns: int
    image: np.ndarray


class LatestFrameCamera:
    """Capture continuously while retaining only the newest frame.

    Slow consumers skip stale frames instead of building a frame queue.
    """

    def __init__(
        self,
        source: Union[int, str],
        frame_size: Tuple[int, int],
        fps: float,
        backend: int = cv2.CAP_ANY,
        buffer_size: int = 1,
    ):
        width, height = frame_size

        if width <= 0 or height <= 0:
            raise ValueError("frame_size must contain positive dimensions")
        if fps <= 0:
            raise ValueError("fps must be positive")

        self._source = source
        self._condition = threading.Condition()
        self._stop_event = threading.Event()

        self._latest: Optional[CapturedFrame] = None
        self._error: Optional[Exception] = None
        self._started = False
        self._closed = False

        self._capture = cv2.VideoCapture(source, backend)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_FPS, fps)
        self._capture.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)

        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError(f"Cannot open camera source {source}.")

        self._thread = threading.Thread(
            target=self._capture_loop,
            name="camera-capture",
            daemon=True,
        )


    def start(self):
        if self._closed:
            raise RuntimeError("Cannot restart a closed camera")

        if not self._started:
            self._started = True
            self._thread.start()
        return self


    def _capture_loop(self):
        frame_index = 0

        try:
            while not self._stop_event.is_set():
                ok, frame = self._capture.read()
                captured_at_ns = time_ns()

                if not ok:
                    if self._stop_event.is_set():
                        break

                    raise RuntimeError(
                        f"Camera source {self._source} stopped returning frames."
                    )

                packet = CapturedFrame(
                    index=frame_index,
                    image_time_ns=captured_at_ns,
                    image=frame,
                )
                frame_index += 1

                with self._condition:
                    # Overwriting this packet intentionally drops stale frames.
                    self._latest = packet
                    self._condition.notify_all()

        except Exception as error:
            with self._condition:
                self._error = error
                self._condition.notify_all()

        finally:
            self._capture.release()
            with self._condition:
                self._condition.notify_all()


    def read_latest(
        self,
        after_index: int = -1,
        timeout: Optional[float] = None,
    ) -> CapturedFrame:
        """Return the newest frame newer than ``after_index``.

        Waiting only happens when inference has caught up with capture.
        """

        if not self._started:
            raise RuntimeError("Camera must be started before reading frames")

        if timeout is not None and timeout <= 0:
            raise ValueError("timeout must be positive or None")

        deadline = None if timeout is None else monotonic() + timeout

        with self._condition:
            while self._latest is None or self._latest.index <= after_index:
                if self._error is not None:
                    raise RuntimeError("Camera capture failed") from self._error

                if self._stop_event.is_set() or not self._thread.is_alive():
                    raise RuntimeError("Camera capture stopped")

                remaining = (
                    None
                    if deadline is None
                    else deadline - monotonic()
                )

                if remaining is not None and remaining <= 0:
                    raise TimeoutError("Timed out waiting for a camera frame")

                self._condition.wait(remaining)
            return self._latest


    def close(self, join_timeout: float = 2.0):
        """Stop capture and release the camera."""

        if self._closed:
            return

        self._closed = True
        self._stop_event.set()

        with self._condition:
            self._condition.notify_all()

        if not self._started:
            self._capture.release()
            return

        self._thread.join(join_timeout)

        if self._thread.is_alive():
            # Some camera backends can remain blocked inside read().
            self._capture.release()
            self._thread.join(join_timeout)


    def __enter__(self):
        return self.start()


    def __exit__(self, exc_type, exc_value, traceback):
        self.close()