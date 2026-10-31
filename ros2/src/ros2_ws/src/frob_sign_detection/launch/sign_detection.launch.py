from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import shlex


def detector_node(context):
    prefix = LaunchConfiguration('python_prefix').perform(context)
    if not prefix:
        # The system interpreter owns ROS, while the existing YOLO environment
        # includes its system packages. Resolve it relative to the workspace so
        # both standalone and operator launches work without shell activation.
        share = Path(get_package_share_directory('frob_sign_detection'))
        for parent in share.parents:
            interpreter = parent / 'venv-yolo' / 'bin' / 'python'
            if interpreter.is_file():
                prefix = shlex.quote(str(interpreter))
                break
    return [Node(package='frob_sign_detection', executable='sign_detector',
                 name='sign_detector', prefix=prefix, output='screen',
                 parameters=[{'max_fps': LaunchConfiguration('detector_fps'),
                              'confidence': LaunchConfiguration('detector_confidence')}],
                 additional_env={'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'})]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_detector', default_value='true'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_info_file', default_value=''),
        DeclareLaunchArgument('camera_width', default_value='320'),
        DeclareLaunchArgument('camera_height', default_value='240'),
        DeclareLaunchArgument('camera_fps', default_value='4.0'),
        DeclareLaunchArgument('detector_fps', default_value='2.5'),
        DeclareLaunchArgument('detector_confidence', default_value='0.45'),
        DeclareLaunchArgument('python_prefix', default_value=''),
        Node(package='frob_sign_detection', executable='camera_driver', name='camera_driver',
             condition=IfCondition(LaunchConfiguration('start_camera')),
             respawn=True, respawn_delay=2.0,
             parameters=[{'device': LaunchConfiguration('camera_device'),
                          'camera_info_file': LaunchConfiguration('camera_info_file'),
                          'width': LaunchConfiguration('camera_width'),
                          'height': LaunchConfiguration('camera_height'),
                          'fps': LaunchConfiguration('camera_fps')}], output='screen'),
        OpaqueFunction(function=detector_node,
                       condition=IfCondition(LaunchConfiguration('start_detector'))),
    ])
