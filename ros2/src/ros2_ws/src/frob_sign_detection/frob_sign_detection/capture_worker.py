"""USB reads are isolated so a stuck driver cannot block the ROS executor."""
import glob
import multiprocessing as mp
import os
import time

import cv2
import numpy as np


def capture_loop(config, pixels, sequence, heartbeat):
    cv2.setNumThreads(1)
    device, width, height, fps = config
    shared = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 3)
    capture = None
    while True:
        heartbeat.value = time.monotonic()
        if capture is None:
            # Persist identity across video0 -> video2 USB re-enumeration.
            cameras = sorted(glob.glob('/dev/v4l/by-id/*video-index0'))
            if not cameras:
                cameras = sorted(
                    path for path in glob.glob('/dev/v4l/by-path/*video-index0')
                    if 'usb' in os.path.basename(path)
                )
            path = cameras[0] if device == '/dev/video0' and len(cameras) == 1 else device
            if not os.path.exists(path):
                time.sleep(1.)
                continue
            capture = cv2.VideoCapture(path, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                capture = None
                time.sleep(1.)
                continue
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_FPS, fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        started = time.monotonic()
        ok, frame = capture.read()
        heartbeat.value = time.monotonic()
        if not ok or frame is None:
            capture.release()
            capture = None
            time.sleep(1.)
            continue
        if frame.shape != shared.shape:
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        # A sequence lock without an OS lock: a stopped writer cannot block us.
        sequence.value += 1
        np.copyto(shared, frame)
        sequence.value += 1
        time.sleep(max(0., 1. / fps - (time.monotonic() - started)))


class CaptureWorker:
    def __init__(self, config, target=capture_loop):
        self.context = mp.get_context('spawn')
        self.config = config
        self.target = target
        self.restarts = 0
        self.start()

    def start(self):
        _, width, height, _ = self.config
        self.pixels = self.context.RawArray('B', width * height * 3)
        self.sequence = self.context.Value('Q', 0, lock=False)
        self.last_sequence = 0
        self.heartbeat = self.context.Value('d', time.monotonic(), lock=False)
        self.process = self.context.Process(target=self.target,
            args=(self.config, self.pixels, self.sequence, self.heartbeat), daemon=True)
        self.process.start()

    def read(self):
        if not self.process.is_alive() or time.monotonic() - self.heartbeat.value > 8.:
            self.close()
            self.restarts += 1
            self.start()
        before = self.sequence.value
        if before == self.last_sequence or before % 2:
            return None
        _, width, height, _ = self.config
        frame = np.frombuffer(self.pixels, dtype=np.uint8).reshape(height, width, 3).copy()
        if self.sequence.value != before:
            return None
        self.last_sequence = before
        return frame

    def close(self):
        if self.process.is_alive():
            self.process.kill()
        self.process.join(timeout=.2)
