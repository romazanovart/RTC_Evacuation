#!/usr/bin/env python3

from datetime import datetime
import json
import base64
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .qr_worker import QRWorker
from .stand_detector import StandTracker

class QrDetector(Node):
    def __init__(self) -> None:
        super().__init__('qr_detector')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/qr_annotated')
        self.declare_parameter('text_topic', '/qr/text')
        self.declare_parameter('processing_fps', 2.0)
        self.declare_parameter(
            'font_path',
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        )
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        text_topic = str(self.get_parameter('text_topic').value)
        fps = max(0.5, float(self.get_parameter('processing_fps').value))
        self._period = 1.0 / fps
        self._last_processed = 0.0
        self._bridge = CvBridge()
        self.get_logger().info('QR engine: isolated ZXing worker with watchdog')
        self._font = None
        font_path = str(self.get_parameter('font_path').value)
        try:
            self._font = cv2.freetype.createFreeType2()
            self._font.loadFontData(fontFileName=font_path, id=0)
        except (AttributeError, cv2.error) as exc:
            self.get_logger().warning(f'Unicode font unavailable: {exc}')
        self._active: set[str] = set()
        self._last_text = ''
        self._last_seen = 0.0
        cv2.setNumThreads(1)
        self._image_publisher = self.create_publisher(Image, output_topic, 2)
        self._text_publisher = self.create_publisher(String, text_topic, 10)
        self._detections_publisher = self.create_publisher(String, '/qr/detections', 2)
        self._worker = QRWorker()
        self._stand_tracker = StandTracker()
        self._pending_message = None
        self._pending_frame = None
        self._last_input = time.monotonic()
        self._last_clear = 0.
        self.create_timer(.05, self._poll_worker)
        self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.get_logger().info(
            f'QR detector: {input_topic} -> {output_topic}, {fps:.1f} FPS'
        )

    @staticmethod
    def _center(points):
        polygon = np.asarray(points, dtype=np.float64).reshape(4, 2)
        try:
            fractions = np.linalg.solve(
                np.column_stack((polygon[2] - polygon[0], -(polygon[3] - polygon[1]))),
                polygon[1] - polygon[0],
            )
            return polygon[0] + fractions[0] * (polygon[2] - polygon[0])
        except np.linalg.LinAlgError:
            return polygon.mean(axis=0)

    def _draw(self, frame: np.ndarray, text: str, points: np.ndarray, decoded: bool = True) -> None:
        polygon = np.asarray(points, dtype=np.int32).reshape(-1, 2)
        color = (0, 255, 0) if decoded else (0, 220, 255)
        cv2.polylines(frame, [polygon], True, color, 3, cv2.LINE_AA)
        center = tuple(np.rint(self._center(points)).astype(int))
        cv2.drawMarker(frame, center, (0, 220, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(frame, center, 5, (0, 220, 255), 2)
        cv2.putText(frame, f'QR center: {center[0]}, {center[1]}',
                    (max(0, min(center[0] + 12, frame.shape[1] - 250)), max(22, center[1] - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 220, 255), 2, cv2.LINE_AA)
        label = ' '.join(text.split()) if decoded else 'QR candidate - not decoded'
        label = label if len(label) <= 80 else label[:77] + '...'
        x = max(0, int(polygon[:, 0].min()))
        y = max(30, int(polygon[:, 1].min()) - 8)
        if self._font is not None:
            size = self._font.getTextSize(label, 20, -1)[0]
        else:
            size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0]
        right = min(frame.shape[1] - 1, x + size[0] + 10)
        cv2.rectangle(frame, (x, y - size[1] - 10), (right, y + 5), (0, 0, 0), -1)
        if self._font is not None:
            self._font.putText(
                frame,
                label,
                (x + 5, y),
                20,
                color,
                -1,
                cv2.LINE_AA,
                True,
            )
        else:
            cv2.putText(
                frame,
                label,
                (x + 5, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        self._last_input = now
        # The live feed never waits for QR, including during worker recovery.
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8').copy()
            points = self._stand_tracker.detect(frame)
            detections = []
            if points is not None:
                center = points.mean(axis=0)
                detections.append({'text': '', 'candidate': True, 'source': self._stand_tracker.source,
                    'center_px': center.tolist(), 'corners': points.tolist(),
                    'offset_normalized': [float(2*center[0]/frame.shape[1]-1), float(2*center[1]/frame.shape[0]-1)],
                    'side_ratio': float(np.ptp(points[:, 0])/frame.shape[1])})
            self._detections_publisher.publish(String(data=json.dumps({'detections': detections})))
            if points is not None:
                preview = frame.copy()
                self._draw(preview, '', points, decoded=False)
                output = self._bridge.cv2_to_imgmsg(preview, encoding='bgr8')
                output.header = message.header
                self._image_publisher.publish(output)
            else:
                self._image_publisher.publish(message)
        except Exception as exc:
            self._detections_publisher.publish(String(data='{"detections": []}'))
            self._image_publisher.publish(message)
            self.get_logger().warning(str(exc), throttle_duration_sec=5.)
            return
        if self._worker.pending:
            return
        if now - self._last_processed < self._period:
            return
        self._last_processed = now
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            if self._worker.submit(frame, roi_hint=points):
                self._pending_message = message
                self._pending_frame = frame.copy()
        except Exception as exc:
            self.get_logger().warning(f'QR submit failed: {exc}', throttle_duration_sec=5.)

    def _poll_worker(self):
        now = time.monotonic()
        result = self._worker.poll()
        if result is not None:
            decoded, error = result
            if error:
                self.get_logger().warning(error, throttle_duration_sec=5.)
            elif self._pending_message is not None and now - self._last_processed < 2.:
                self._publish_decoded(self._pending_message, self._pending_frame, decoded)
            self._pending_message = self._pending_frame = None
        if now - self._last_seen > 1.5 and now - self._last_clear > .5:
            self._last_clear = now
            self._text_publisher.publish(String(data=''))
        if now - self._last_input > .7:
            self._detections_publisher.publish(String(data='{"detections": []}'))

    def _publish_decoded(self, message, frame, decoded):
        if not decoded:
            return
        now = time.monotonic()
        try:
            detections = []
            for text, points in decoded:
                center = self._center(points)
                detections.append({'text': text, 'center_px': center.tolist(),
                                   'offset_normalized': [float(2 * center[0] / frame.shape[1] - 1),
                                                         float(2 * center[1] / frame.shape[0] - 1)],
                                   'corners': np.asarray(points).reshape(4, 2).tolist(),
                                   'candidate': False})
            current = {text for text, _ in decoded}
            for text, points in decoded:
                self._draw(frame, text, points)
                if text not in self._active:
                    stamp = datetime.now().isoformat(timespec='seconds')
                    self.get_logger().info(f'[{stamp}] QR: {text}')
            if decoded:
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    detections[0]['imageData'] = 'data:image/jpeg;base64,' + base64.b64encode(jpeg.tobytes()).decode('ascii')
            self._detections_publisher.publish(String(data=json.dumps({
                'width': frame.shape[1], 'height': frame.shape[0],
                'capture_age_sec': max(0., now-self._last_processed),
                'stamp': {'sec': message.header.stamp.sec, 'nanosec': message.header.stamp.nanosec},
                'detections': detections,
            }, ensure_ascii=False)))
            if current:
                self._active = current
                self._last_text = ' | '.join(text for text, _ in decoded)
                self._last_seen = now
            elif now - self._last_seen >= 1.5:
                self._active.clear()
                self._last_text = ''
            text_message = String()
            text_message.data = self._last_text
            self._text_publisher.publish(text_message)
            output = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            output.header = message.header
            self._image_publisher.publish(output)
        except Exception as exc:
            self.get_logger().warning(f'QR frame failed: {exc}')

    def destroy_node(self):
        self._worker.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QrDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
