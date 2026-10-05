"""RealSense → perception → planning → control 전체 실행.

ros2 launch lv2_module5/launch/bringup.launch.py
ros2 launch lv2_module5/launch/bringup.launch.py camera:=false motor_enable:=false
health 진단 발행 노드는 별도 연결 필요. 기본 health.enabled=true는 유지한다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    config_dir = os.path.join(launch_dir, '..', 'config')
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('motor_enable', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'perception.launch.py')),
            launch_arguments={'camera': LaunchConfiguration('camera')}.items()),
        Node(package='planning', executable='planning_master', name='planning_master',
             output='screen', parameters=[os.path.join(config_dir, 'planning.yaml')]),
        Node(package='control', executable='control_master', name='control_master',
             output='screen', parameters=[os.path.join(config_dir, 'control.yaml'), {
                 'port': LaunchConfiguration('port'),
                 'motor_enable': ParameterValue(LaunchConfiguration('motor_enable'), value_type=bool),
             }]),
    ])
