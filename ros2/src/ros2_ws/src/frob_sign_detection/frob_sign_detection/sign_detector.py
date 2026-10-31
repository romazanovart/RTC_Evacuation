import json
import os
import threading
import time
from http import server

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from ultralytics import YOLO


RUSSIAN_NAMES = {
    'straight': 'Движение прямо',
    'left': 'Движение налево',
    'right': 'Движение направо',
    'no_left': 'Поворот налево запрещён',
    'no_right': 'Поворот направо запрещён',
    'bus_stop': 'Место остановки',
    'parking': 'Парковка',
    'danger': 'Опасный объект',
    'obstacle': 'Опасный объект',
}


class StreamState:
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None

    def update(self, jpeg):
        with self.condition:
            self.jpeg = jpeg
            self.condition.notify_all()


class StreamHandler(server.BaseHTTPRequestHandler):
    state = None

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            page = '''<!doctype html><html lang="ru"><head><meta charset="utf-8">
<title>Frob - распознавание знаков</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{max-width:96vw;max-height:82vh;margin-top:12px}.legend{padding:10px;color:#bbb}</style></head>
<body><h2>Frob: распознавание дорожных знаков</h2><img src="/stream.mjpg">
<div class="legend">straight - прямо · left/right - направление · no_left/no_right - запрет · bus_stop - остановка · parking - парковка · obstacle - опасный объект</div>
</body></html>'''.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path == '/snapshot.jpg':
            with self.state.condition:
                jpeg = self.state.jpeg
            if jpeg is None:
                self.send_error(503, 'Camera frame is not ready')
                return
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if self.path != '/stream.mjpg':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with self.state.condition:
                    self.state.condition.wait(timeout=2.0)
                    jpeg = self.state.jpeg
                if jpeg is None:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, _format, *_args):
        return


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('confidence', 0.45)
        self.declare_parameter('image_size', 320)
        self.declare_parameter('max_fps', 4.0)
        self.declare_parameter('http_port', 8080)

        model_path = self.get_parameter('model_path').value
        if not model_path:
            from ament_index_python.packages import get_package_share_directory
            model_path = os.path.join(
                get_package_share_directory('frob_sign_detection'), 'models', 'best.pt'
            )
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.max_fps = max(0.5, float(self.get_parameter('max_fps').value))
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Model classes: {self.model.names}')

        device = int(self.get_parameter('camera_device').value)
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter('camera_width').value))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter('camera_height').value))
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.capture.isOpened():
            raise RuntimeError(f'Cannot open camera /dev/video{device}')

        self.raw_image_pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.image_pub = self.create_publisher(Image, '/sign_detector/image', 1)
        self.compressed_pub = self.create_publisher(
            CompressedImage, '/sign_detector/image/compressed', 1
        )
        self.detections_pub = self.create_publisher(String, '/sign_detector/detections', 10)
        self.classes_pub = self.create_publisher(String, '/sign_detector/classes', 1)
        self.classes_pub.publish(String(data=json.dumps(RUSSIAN_NAMES, ensure_ascii=False)))

        self.stream_state = StreamState()
        StreamHandler.state = self.stream_state
        port = int(self.get_parameter('http_port').value)
        self.http_server = server.ThreadingHTTPServer(('0.0.0.0', port), StreamHandler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'MJPEG stream: http://0.0.0.0:{port}/')

        self.last_frame_time = 0.0
        self.timer = self.create_timer(0.001, self.process_frame)

    def process_frame(self):
        now = time.monotonic()
        if now - self.last_frame_time < 1.0 / self.max_fps:
            return
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning('Camera frame read failed', throttle_duration_sec=2.0)
            return
        self.last_frame_time = now

        raw_msg = self.cv_to_image_message(frame)
        self.raw_image_pub.publish(raw_msg)

        result = self.model.predict(
            source=frame,
            conf=self.confidence,
            imgsz=self.image_size,
            device='cpu',
            verbose=False,
        )[0]
        annotated = result.plot()
        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                class_name = str(result.names[class_id])
                xyxy = [round(float(value), 1) for value in box.xyxy[0].tolist()]
                detections.append({
                    'class_id': class_id,
                    'class': class_name,
                    'label_ru': RUSSIAN_NAMES.get(class_name, class_name),
                    'confidence': round(float(box.conf[0].item()), 4),
                    'box_xyxy': xyxy,
                })

        stamp = self.get_clock().now().to_msg()
        image_msg = self.cv_to_image_message(annotated, stamp)
        self.image_pub.publish(image_msg)

        encoded, jpeg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if encoded:
            jpeg_bytes = jpeg.tobytes()
            compressed = CompressedImage()
            compressed.header = image_msg.header
            compressed.format = 'jpeg'
            compressed.data = jpeg_bytes
            self.compressed_pub.publish(compressed)
            self.stream_state.update(jpeg_bytes)

        payload = {'stamp_ns': self.get_clock().now().nanoseconds, 'detections': detections}
        self.detections_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def cv_to_image_message(self, frame, stamp=None):
        msg = Image()
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    def destroy_node(self):
        self.capture.release()
        self.http_server.shutdown()
        self.http_server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SignDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
