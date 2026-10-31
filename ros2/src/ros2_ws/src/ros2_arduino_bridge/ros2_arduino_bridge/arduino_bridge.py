from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist, Vector3Stamped
import rclpy
from serial import Serial, SerialException
from time import sleep
import sys
import glob
import os
import uuid
import time
import math
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.signals import SignalHandlerOptions

from frob_interfaces.srv import Turn, Forward
from ros2_arduino_bridge.connection import ArduinoConnection

class Arduino_bridge(Node):
    MAX_LIN_SPEED = 0.4863  # [m/s]
    MAX_ANG_SPEED = 5.4     # [rad/s]

    def __init__(self, connection: ArduinoConnection):
        unique_id = str(uuid.uuid4())[:8]
        super().__init__(f'arduino_bridge_{unique_id}')
        self._connect = connection
        # Every operation shares one request/reply serial stream.  Reentrant
        # callbacks can overlap reconnect, telemetry and watchdog transactions
        # and corrupt the framing even if individual writes are locked.
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self._connected = True
        self.declare_parameter('enable_legacy_motion', True)
        self.declare_parameter('cmd_vel_topic', '/frob/motor_cmd_vel')
        self.last_command_time = time.monotonic()
        self.command_timeout = 0.5
        self.telemetry_failures = 0
        self.pair_stamp = None
        self.telemetry_failure_limit = 3
        self._connect.setSpeeds(0.0, 0.0)

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group
        )
        self.pair_pub = self.create_publisher(Vector3Stamped, '/wheel/encoder_delta', 10)
        self.left_delta_pub = self.create_publisher(Int32, 'left_motor/encoder/delta', 10)
        self.right_delta_pub = self.create_publisher(Int32, 'right_motor/encoder/delta', 10)

        self.left_speed_pub = self.create_publisher(Float32, 'left_motor/speed', 10)
        self.right_speed_pub = self.create_publisher(Float32, 'right_motor/speed', 10)   

        # Services
        self.rotate_srv = self.create_service(Turn, 'rotate_robot', self.handle_rotate_request, callback_group=self.callback_group)
        self.forward_srv = self.create_service(Forward, 'forward_robot', self.handle_forward_request, callback_group=self.callback_group)

        # State variables
        self.last_linear = None
        self.last_angular = None

        # Timers
        self.data_request_timer = self.create_timer(0.1, self.data, callback_group=self.callback_group)
        self.connection_check_timer = self.create_timer(1.0, self.check_connection, callback_group=self.callback_group)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog, callback_group=self.callback_group)

    @classmethod
    def clamp_speed(cls, value, max_value):
        return max(min(value, max_value), -max_value)

    def handle_rotate_request(self, request, response):
        if not self.get_parameter('enable_legacy_motion').value:
            response.success = False
            self.get_logger().error('Legacy blocking movement disabled; use velocity-controlled navigation.')
            return response
        if not self._connected:
            self._print_error("Service call failed: No active connection!")
            response.success = False
            return response
            
        try:
            result = self._connect.turn_robot(request.angle, request.speed)
            response.success = bool(result)
            if result:
                self.get_logger().info(f"Rotated by {request.angle}°, firmware speed={request.speed}")
        except SerialException as e:
            self._handle_disconnection()
            response.success = False
            self._print_error(f"Rotation failed: {str(e)}")
        return response

    def handle_forward_request(self, request, response):
        if not self.get_parameter('enable_legacy_motion').value:
            response.success = False
            self.get_logger().error('Legacy distance units require firmware verification; service disabled.')
            return response
        if not self._connected:
            self._print_error("Service call failed: No active connection!")
            response.success = False
            return response
            
        try:
            result = self._connect.go_dist(request.dist, request.speed)
            response.success = bool(result)
            if result:
                self.get_logger().info(f"Legacy distance request: {request.dist} firmware units, speed={request.speed}")
        except SerialException as e:
            self._handle_disconnection()
            response.success = False
            self._print_error(f"Movement failed: {str(e)}")
        return response

    def cmd_vel_callback(self, msg):
        if not self._connected:
            return
            
        try:
            linear = float(self.clamp_speed(msg.linear.x, self.MAX_LIN_SPEED))
            angular = float(self.clamp_speed(msg.angular.z, self.MAX_ANG_SPEED))
            
            if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
                linear = angular = 0.0
            self._connect.setSpeeds(linear, angular)
            self.last_command_time = time.monotonic()
            self.last_linear, self.last_angular = linear, angular
            
        except SerialException as e:
            self._handle_disconnection()

    def data(self):
        if not self._connected:
            return
            
        try:
            arduino_data = self._connect.get_data()
            self.telemetry_failures = 0
            received = time.monotonic()
            if self.pair_stamp is not None:
                pair = Vector3Stamped()
                pair.header.stamp = self.get_clock().now().to_msg()
                pair.header.frame_id = 'base_footprint'
                pair.vector.x = float(arduino_data.left_delta)
                pair.vector.y = float(arduino_data.right_delta)
                pair.vector.z = received - self.pair_stamp
                self.pair_pub.publish(pair)
            self.pair_stamp = received
            self.left_delta_pub.publish(Int32(data=arduino_data.left_delta))
            self.right_delta_pub.publish(Int32(data=arduino_data.right_delta))
            self.left_speed_pub.publish(Float32(data=arduino_data.left_speed))
            self.right_speed_pub.publish(Float32(data=arduino_data.right_speed))
            #print(f'left_delta {arduino_data.left_delta} right_delta {arduino_data.right_delta} left_speed {arduino_data.left_speed} left_speed {arduino_data.right_speed}')
        except SerialException as e:
            self.telemetry_failures += 1
            if self.telemetry_failures < self.telemetry_failure_limit:
                try:
                    self._connect.discard_partial_reply()
                except SerialException:
                    pass
                self.get_logger().warning(
                    f'Telemetry timeout {self.telemetry_failures}/'
                    f'{self.telemetry_failure_limit}; keeping motor link alive: {e}')
            else:
                self._handle_disconnection()

    def watchdog(self):
        if self._connected and time.monotonic() - self.last_command_time > self.command_timeout:
            try:
                self._connect.setSpeeds(0.0, 0.0)
                self.last_linear = self.last_angular = 0.0
            except SerialException:
                self._handle_disconnection()

    def check_connection(self):
        if not self._connected:
            self._try_reconnect()

    def _handle_disconnection(self):
        if self._connected:
            self._print_error("Connection lost! Check Arduino connection.")
            self._connected = False
            self._connect.close()

    def _try_reconnect(self):
        ports = self._find_arduino_ports()
        for port in ports:
            try:
                self.get_logger().info(f"Attempting connection to {port}...")
                connection = ArduinoConnection(Serial(port, 115200))
                sleep(2)
                
                if connection.is_arduino():
                    self._connect = connection
                    self._connected = True
                    self._connect.setSpeeds(0.0, 0.0)
                    self.last_linear = self.last_angular = 0.0
                    self.telemetry_failures = 0
                    self.pair_stamp = None
                    self.last_command_time = time.monotonic()
                    self.get_logger().info(f"Successfully connected to Arduino at {port}")
                    return True
                else:
                    connection.close()
                    self.get_logger().warning(f"Invalid response from {port}")
                    
            except (SerialException, OSError) as e:
                self.get_logger().warning(f"Connection failed to {port}: {str(e)}")
                continue
            except Exception as e:
                self._print_error(f"Unexpected error: {str(e)}")
                continue
                
        self._print_error("No valid Arduino devices found!")
        return False

    def _find_arduino_ports(self):
        if sys.platform.startswith('win'):
            ports = [f'COM{i+1}' for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = linux_arduino_ports()
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*')
        else:
            raise EnvironmentError('Unsupported OS')
        return ports

    def _print_error(self, message):
        self.get_logger().error('\033[1;31m' + message + '\033[0m')

    def shutdown(self):
        if self._connected:
            try:
                self._connect.setSpeeds(0.0, 0.0)
                self._connect.close()
            except Exception as e:
                self._print_error(f"Connection closure error: {str(e)}")
        
        self.destroy_node()

def find_arduino_ports():
    if sys.platform.startswith('win'):
        ports = [f'COM{i+1}' for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = linux_arduino_ports()
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*')
    else:
        raise EnvironmentError('Unsupported OS')
    return ports


def linux_arduino_ports():
    by_id = sorted(glob.glob('/dev/serial/by-id/*Arduino*'))
    acm = sorted(glob.glob('/dev/ttyACM*'))
    # USB UART devices include the lidar on this robot.  Probe them only when
    # no Arduino-style CDC device is present.
    return by_id or acm or sorted(glob.glob('/dev/ttyUSB*'))

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    arduino_bridge = None

    try:
        # Используем argparse для обработки аргументов
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--port', type=str, help='Serial port for Arduino', default=None)
        args, unknown = parser.parse_known_args()

        # Получаем порт из аргументов
        port = args.port
        ports = find_arduino_ports()

        if port:
            if not os.path.exists(port):
                print(f"\033[1;31mPort {port} not found!\033[0m")
                return
            ports = [port]  # Если порт указан, проверяем только его
        else:
            print("\033[36mNo port specified, searching for Arduino devices...\033[0m")

        for p in ports:
            try:
                print(f"\033[36mConnecting to {p}...\033[0m")
                connection = ArduinoConnection(Serial(p, 115200))
                sleep(2)

                if connection.is_arduino():
                    arduino_bridge = Arduino_bridge(connection)
                    print(f"\033[1;32mSuccessfully connected to Arduino at {p}\033[0m")
                    break
                else:
                    print(f"\033[33mDevice at {p} is not responding as Arduino\033[0m")
                    connection.close()

            except Exception as e:
                print(f"\033[33mConnection to {p} failed: {str(e)}\033[0m")
                continue
        else:
            print("\033[1;31mNo valid Arduino devices found!\033[0m")
            return

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
        executor.add_node(arduino_bridge)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            arduino_bridge.shutdown()

    except Exception as e:
        print(f"\033[1;31mCritical error: {str(e)}\033[0m")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
