"""Camera capture is independent from inference; one process owns the device."""
import os
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
import yaml
import time
from .capture_worker import CaptureWorker


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_driver')
        for key, val in dict(device='/dev/video0', width=640, height=480, fps=20.,
                             camera_info_file='', frame_id='camera_optical_frame').items():
            self.declare_parameter(key, val)
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.info = CameraInfo()
        path = self.get_parameter('camera_info_file').value
        if path:
            with open(path, encoding='utf-8') as stream:
                data = yaml.safe_load(stream)
            self.info.width, self.info.height = data['image_width'], data['image_height']
            self.info.k, self.info.d = data['camera_matrix']['data'], data['distortion_coefficients']['data']
            self.info.r, self.info.p = data['rectification_matrix']['data'], data['projection_matrix']['data']
            self.info.distortion_model = data['distortion_model']
        else:
            self.get_logger().warning('No camera calibration; approach-based sign association only.')
        cv2.setNumThreads(1)
        self.capture = None
        self._config = None
        self._last_frame = time.monotonic()
        self._frame_timeout = 15.0
        self._heartbeat_path = '/run/user/1000/frob-camera-frame'
        self._heartbeat_written = 0.0
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.create_timer(.04, self.capture_once)

    def parameters_changed(self, parameters):
        for parameter in parameters:
            if parameter.name in {'width', 'height'} and not 120 <= parameter.value <= 4096:
                return SetParametersResult(successful=False, reason='Invalid camera dimensions')
            if parameter.name == 'fps' and not 1. <= parameter.value <= 60.:
                return SetParametersResult(successful=False, reason='FPS must be 1..60')
        return SetParametersResult(successful=True)

    def capture_once(self):
        config = tuple(self.get_parameter(key).value for key in ('device', 'width', 'height', 'fps'))
        if self.capture is None or config != self._config:
            if self.capture is not None:
                self.capture.close()
            self.capture = CaptureWorker(config)
            self._config = config
        frame = self.capture.read()
        if frame is None:
            if time.monotonic() - self._last_frame > 3.:
                self.get_logger().warning('No camera frames; automatic USB reconnect active', throttle_duration_sec=10.)
            if time.monotonic() - self._last_frame > self._frame_timeout:
                raise RuntimeError('Camera frame timeout; restarting camera node')
            return
        self._last_frame = time.monotonic()
        if self._last_frame - self._heartbeat_written >= 1.0:
            self._heartbeat_written = self._last_frame
            try:
                with open(self._heartbeat_path, 'a', encoding='utf-8'):
                    os.utime(self._heartbeat_path)
            except OSError:
                pass
        target = (int(self.get_parameter('width').value), int(self.get_parameter('height').value))
        if (frame.shape[1], frame.shape[0]) != target:
            frame = cv2.resize(frame, target, interpolation=cv2.INTER_AREA)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.height, msg.width = frame.shape[:2]
        msg.encoding, msg.step, msg.data = 'bgr8', msg.width * 3, frame.tobytes()
        self.pub.publish(msg)
        self.info.header = msg.header
        self.info.width, self.info.height = msg.width, msg.height
        self.info_pub.publish(self.info)

    def destroy_node(self):
        if self.capture is not None:
            self.capture.close()
        super().destroy_node()


def main(args=None):
    # Keep rclpy's SIGINT handler for this OpenCV process; without it the USB
    # capture thread can delay Python's default handler past launch's timeout.
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
