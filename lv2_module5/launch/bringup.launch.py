"""RealSense → perception → planning → control 전체 실행.

ros2 launch lv2_module5/launch/bringup.launch.py
ros2 launch lv2_module5/launch/bringup.launch.py camera:=false motor_enable:=false
ros2 launch lv2_module5/launch/bringup.launch.py dashboard:=true
    # 화면에는 planning 대시보드만. 나머지 노드 로그는 ~/.ros/log/<날짜>/ 로 (ros2 launch 로그 폴더)
ros2 launch lv2_module5/launch/bringup.launch.py event_log:=results/logs/planning_events.txt
    # 상태·reason 전이 기록을 파일로 (dashboard 와 함께 써도 됨)
ros2 launch lv2_module5/launch/bringup.launch.py planning:=false
    # planning 만 빼고 실행 (planning 을 다른 터미널에서 따로 돌릴 때)
ros2 launch lv2_module5/launch/bringup.launch.py pin_cpu:=true
    # 코어 1~3 은 perception(NCNN 3스레드) 전용, 코어 0 에 RealSense · control · planning
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
CONFIG_DIR = os.path.join(LAUNCH_DIR, '..', 'config')


def _true(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('true', '1', 'yes')


def launch_setup(context):
    dashboard = _true(context, 'dashboard')
    out = 'log' if dashboard else 'screen'     # 대시보드를 켜면 다른 노드는 화면에 안 찍음
    pin = _true(context, 'pin_cpu')
    core0 = 'taskset -c 0' if pin else None    # RealSense · control · planning

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'perception.launch.py')),
            launch_arguments={'camera': LaunchConfiguration('camera'), 'output': out,
                              'camera_cpus': '0' if pin else '',
                              'perception_cpus': '1-3' if pin else ''}.items()),
        Node(package='control', executable='control_master', name='control_master',
             output=out, prefix=core0, parameters=[os.path.join(CONFIG_DIR, 'control.yaml'), {
                 'port': LaunchConfiguration('port'),
                 'motor_enable': ParameterValue(LaunchConfiguration('motor_enable'), value_type=bool),
             }]),
    ]
    if _true(context, 'planning'):
        planning_kwargs = {}
        if dashboard:
            # 대시보드는 화면을 지우고 다시 그리므로 launch 의 "[planning_master-3]" 줄 머리를 뗀다
            planning_kwargs = {'output_format': '{line}', 'emulate_tty': True}
        actions.append(Node(
            package='planning', executable='planning_master', name='planning_master',
            output='screen', prefix=core0,
            parameters=[os.path.join(CONFIG_DIR, 'planning.yaml'), {
                'dashboard': dashboard,
                'event_log': LaunchConfiguration('event_log'),
            }],
            **planning_kwargs))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('motor_enable', default_value='true'),
        DeclareLaunchArgument('planning', default_value='true'),     # false: planning 만 따로 실행할 때
        DeclareLaunchArgument('dashboard', default_value='false'),   # true: 화면에 planning 대시보드만
        DeclareLaunchArgument('event_log', default_value=''),        # 상태 전이 기록 파일 ('' 이면 안 남김)
        DeclareLaunchArgument('pin_cpu', default_value='false'),     # true: 코어 1~3 perception 전용
        OpaqueFunction(function=launch_setup),
    ])
