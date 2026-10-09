"""Non-blocking, bounded-queue OpenCV video writer."""
from queue import Empty, Full, Queue
import threading
from typing import Optional, Tuple

import cv2
import numpy as np

cv2.ocl.setUseOpenCL(False)
cv2.setNumThreads(1)


class VideoWriterWorker:
    def __init__(
        self,
        out_path: str,
        frame_size: Tuple[int, int],
        fps: float,
        codec_string: str = "mp4v",
        buffer_size: int = 10,
    ):
        width, height = map(int, frame_size)
        fps = float(fps)
        buffer_size = int(buffer_size)

        if width <= 0 or height <= 0:
            raise ValueError("frame_size must contain positive dimensions")

        if fps <= 0:
            raise ValueError("fps must be positive")

        if not isinstance(out_path, str) or not out_path:
            raise ValueError("Video writer path is empty")

        if not isinstance(codec_string, str) or len(codec_string) != 4:
            raise ValueError("codec_string must contain exactly four characters")

        if buffer_size <= 0:
            raise ValueError("buffer_size must be positive")

        self._source = out_path
        self._expected_shape = (height, width, 3)
        self._stop_event = threading.Event()
        self._queue = Queue(maxsize=buffer_size)

        self._started = False
        self._closed = False
        self._error: Optional[Exception] = None

        self.dropped_frames = 0
        self.written_frames = 0

        codec = cv2.VideoWriter_fourcc(*codec_string)   #type: ignore
        self._writer = cv2.VideoWriter(out_path, codec, fps, (width, height))

        if not self._writer.isOpened():
            self._writer.release()
            raise RuntimeError(f"Cannot open video writer: {out_path}")

        self._thread = threading.Thread(
            target=self._writer_loop,
            name="video-writer",
            daemon=False,
        )


    def start(self):
        if self._closed:
            raise RuntimeError("Cannot restart a closed writer")

        if not self._started:
            self._thread.start()
            self._started = True
        return self


    def submit(self, frame_index, frame):
        """Submit a frame without waiting for encoding."""
        self._raise_if_failed()

        if self._closed:
            raise RuntimeError("Writer is closed")

        if not self._started:
            raise RuntimeError("Writer must be started before submitting")

        if not isinstance(frame, np.ndarray):
            raise TypeError("Frame must be a NumPy array")

        if frame.shape != self._expected_shape or frame.dtype != np.uint8:
            raise ValueError(
                f"Expected uint8 BGR frame {self._expected_shape}; "
                f"received shape={frame.shape}, dtype={frame.dtype}"
            )

        try:
            self._queue.put_nowait((int(frame_index), frame))
        except Full:
            self.dropped_frames += 1
            return False
        return True


    def _writer_loop(self):
        try:
            while not self._stop_event.is_set() or not self._queue.empty():
                try:
                    frame_index, frame = self._queue.get(timeout=0.1)
                except Empty:
                    continue

                try:
                    self._writer.write(frame)
                    self.written_frames += 1
                finally:
                    self._queue.task_done()

        except Exception as error:
            self._error = error
        finally:
            self._writer.release()


    def close(self, join_timeout=None):
        """Drain queued frames and finalize the video."""
        if self._closed:
            return

        self._closed = True
        self._stop_event.set()

        if not self._started:
            self._writer.release()
            return

        self._thread.join(join_timeout)

        if self._thread.is_alive():
            raise RuntimeError("Video writer worker did not stop")
        self._raise_if_failed()


    def _raise_if_failed(self):
        if self._error is not None:
            raise RuntimeError("Video writer worker failed") from self._error


    def __enter__(self):
        return self.start()


    def __exit__(self, exc_type, exc_value, traceback):
        self.close()