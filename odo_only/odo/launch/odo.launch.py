# launch/path_follower_c_zone.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('odo')
    config    = os.path.join(pkg_share, 'config', 'c_params.yaml')

    return LaunchDescription([
        # 1) sensor_node
        Node(
            package='odo',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
        ),

        # 2) odo_node
        Node(
            package='odo',
            executable='odo_node',
            name='odo_node',
            output='screen',
            parameters=[{
                'ticks_meter': 8565.0,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'rate_hz': 10.0,
                'encoder_min': -32768,
                'encoder_max': 32768
            }],
        ),

        # 3) Twist 입력 퍼블리셔 (키보드 입력 → /twist)
        Node(
            package='odo',
            executable='twist_input_publisher',
            name='twist_input_publisher',
            output='screen',
            parameters=[{
                'goal_x': 5.0,
                'goal_y': 2.0,
                'slowdown_dist': 0.5,
                'steering_slow_threshold': 20.0,
                'Kp': 5.0,
                'Ki': 2.0,
                'Kd': 0.5
            }],
        ),
    ])
