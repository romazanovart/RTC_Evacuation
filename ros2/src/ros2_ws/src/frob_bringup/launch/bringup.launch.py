from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Узел ros2_arduino_bridge
    arduino_bridge = Node(
        package='ros2_arduino_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['--port', LaunchConfiguration('arduino_port')],
        parameters=[{'cmd_vel_topic': LaunchConfiguration('motor_cmd_topic')}]
    )
    #Lidar YDLIDAR X4
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_lidar')),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id': 'laser',
            'inverted': 'false',
            'angle_compensate': 'true',
        }.items()
    )
    lidar_filter = Node(
        package='lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter',
        condition=IfCondition(LaunchConfiguration('start_lidar')),
        output='screen'
    )
    #Imu mpu6050
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_mpu6050'), 'launch', 'ros2_mpu6050.launch.py')
        )
    )

    #Description
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frob_description'), 'launch', 'description.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_description')),
    )

    #Odometry
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frob_odometry'), 'launch', 'odometry.launch.py')
        )
    )

    return LaunchDescription([
        # The current field/navigation stack uses encoder + IMU odometry and
        # camera guidance.  Keep the lidar opt-in so it does not consume CPU,
        # USB bandwidth and power on every boot.
        DeclareLaunchArgument('start_lidar', default_value='false'),
        # TF visualization publishers are unnecessary for field motion when
        # lidar is disabled.  They remain opt-in for diagnostics.
        DeclareLaunchArgument('start_description', default_value='false'),
        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/ttyACM0',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
        ),
        DeclareLaunchArgument('motor_cmd_topic', default_value='/frob/motor_cmd_vel'),
        Node(package='frob_control', executable='velocity_gate', name='velocity_gate', output='screen',
             parameters=[{'output_topic': LaunchConfiguration('motor_cmd_topic')}],
             respawn=True, respawn_delay=2.0),
        arduino_bridge,
        lidar_launch,
        imu_launch,
        description_launch,
        odometry_launch,
        lidar_filter
    ])
