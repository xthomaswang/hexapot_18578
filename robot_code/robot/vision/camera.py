#!/usr/bin/env python3
import io
import os
from threading import Condition, Lock

from libcamera import controls
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = bytes(buf)
            self.condition.notify_all()


class PiCameraStream:
    def __init__(self, width=None, height=None, fps=None):
        self.width = int(width or os.environ.get("CAMERA_WIDTH", "1280"))
        self.height = int(height or os.environ.get("CAMERA_HEIGHT", "720"))
        self.fps = int(fps or os.environ.get("CAMERA_FPS", "30"))
        self.frame_duration_us = int(1_000_000 / max(self.fps, 1))
        self.picam2 = Picamera2()
        self.output = StreamingOutput()
        self._lock = Lock()
        self._started = False

    def start(self):
        if self._started:
            return

        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height)},
            controls={"FrameDurationLimits": (self.frame_duration_us, self.frame_duration_us)},
        )
        self.picam2.configure(config)

        try:
            self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        except Exception:
            pass

        self.picam2.start_recording(JpegEncoder(), FileOutput(self.output))
        self._started = True

    def stop(self):
        if not self._started:
            return

        with self._lock:
            try:
                self.picam2.stop_recording()
            finally:
                self._started = False

    def wait_for_frame(self):
        self.start()
        with self.output.condition:
            while self.output.frame is None:
                self.output.condition.wait()
            return self.output.frame

    def iter_mjpeg(self):
        self.start()
        while True:
            frame = self.wait_for_frame()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )

    def capture_array(self):
        self.start()
        with self._lock:
            return self.picam2.capture_array("main")
