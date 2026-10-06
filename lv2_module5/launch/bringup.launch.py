"""RealSense → perception → planning → control 전체 실행.

ros2 launch lv2_module5/launch/bringup.launch.py
ros2 launch lv2_module5/launch/bringup.launch.py camera:=false motor_enable:=false
ros2 launch lv2_module5/launch/bringup.launch.py event_log:=results/logs/planning_events.txt
    # 상태 전이 기록을 파일로 (상태 전이는 이 화면에도 'TRACKING -> FAULT reason=...' 로 찍힘)

대시보드(dashboard:=true)는 화면을 지우며 다시 그리므로 다른 노드 로그와 같은 화면에 띄우지 않는다.
대시보드를 보려면 planning 만 빼고 띄운 뒤, planning 을 다른 터미널에서 따로 실행:
  ros2 launch lv2_module5/launch/bringup.launch.py planning:=false
  ros2 run planning planning_master --ros-args --params-file lv2_module5/config/planning.yaml -p dashboard:=true
health 진단 발행 노드는 별도 연결 필요. 기본 health.enabled=true는 유지한다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('planning', default_value='true'),     # false: 대시보드용으로 따로 실행할 때
        DeclareLaunchArgument('event_log', default_value=''),        # 상태 전이 기록 파일 ('' 이면 안 남김)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'perception.launch.py')),
            launch_arguments={'camera': LaunchConfiguration('camera')}.items()),
        Node(package='planning', executable='planning_master', name='planning_master',
             output='screen', condition=IfCondition(LaunchConfiguration('planning')),
             parameters=[os.path.join(config_dir, 'planning.yaml'),
                         {'event_log': LaunchConfiguration('event_log')}]),
        Node(package='control', executable='control_master', name='control_master',
             output='screen', parameters=[os.path.join(config_dir, 'control.yaml'), {
                 'port': LaunchConfiguration('port'),
                 'motor_enable': ParameterValue(LaunchConfiguration('motor_enable'), value_type=bool),
             }]),
    ])
