"""Run with both ROS workspace packages on PYTHONPATH."""
import time
import unittest
import cv2
import numpy as np
from rover_web.qr_worker import Decoder, QRWorker
from frob_sign_detection.capture_worker import CaptureWorker


def hang(*args):
    time.sleep(60)


class RecoveryTests(unittest.TestCase):
    def test_small_qr_and_negative(self):
        decoder = Decoder()
        code = cv2.QRCodeEncoder_create().encode('FROB-distance-test')
        for size in (90, 120, 180):
            image = np.full((720, 1280, 3), 210, np.uint8)
            qr = cv2.resize(code, (size, size), interpolation=cv2.INTER_NEAREST)
            image[300:300+size, 800:800+size] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
            result = decoder.decode(image)
            self.assertTrue(any(text == 'FROB-distance-test' for text, _ in result), size)
            for _, points in result:
                self.assertTrue(np.all(points[:, 0] >= 800))
                self.assertTrue(np.all(points[:, 0] <= 800 + size))
        self.assertEqual(decoder.decode(np.full((720, 1280, 3), 220, np.uint8)), [])

    def test_qr_hang_is_killed_without_blocking_caller(self):
        worker = QRWorker(timeout=.3, target=hang)
        try:
            started = time.monotonic()
            worker.submit(np.zeros((720, 1280, 3), np.uint8))
            self.assertLess(time.monotonic() - started, .15)
            time.sleep(.4)
            result = worker.poll()
            self.assertIn('restarted', result[1])
            self.assertEqual(worker.restarts, 1)
        finally:
            worker.close()

    def test_capture_hang_and_crash_recover(self):
        worker = CaptureWorker(('/dev/nonexistent', 640, 480, 5.), target=hang)
        try:
            old_pid = worker.process.pid
            worker.heartbeat.value = time.monotonic() - 9
            self.assertIsNone(worker.read())
            self.assertNotEqual(old_pid, worker.process.pid)
            worker.process.kill()
            worker.process.join(timeout=1.)
            self.assertIsNone(worker.read())
            self.assertEqual(worker.restarts, 2)
        finally:
            worker.close()

    def test_incomplete_frame_never_blocks_reader(self):
        worker = CaptureWorker(('/dev/nonexistent', 640, 480, 5.), target=hang)
        try:
            worker.sequence.value = 1
            started = time.monotonic()
            self.assertIsNone(worker.read())
            self.assertLess(time.monotonic() - started, .1)
            worker.sequence.value = 2
            frame = worker.read()
            self.assertEqual(frame.shape, (480, 640, 3))
            self.assertIsNone(worker.read())
        finally:
            worker.close()


if __name__ == '__main__':
    unittest.main()
