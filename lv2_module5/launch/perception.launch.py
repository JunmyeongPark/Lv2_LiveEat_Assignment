# RealSense + perception_master 실행 (설정: ../config/camera.yaml, perception.yaml)
#
#   ros2 launch lv2_module5/launch/perception.launch.py
#   ros2 launch lv2_module5/launch/perception.launch.py camera:=false            # 카메라를 따로 띄운 경우
#   ros2 launch lv2_module5/launch/perception.launch.py output_topic:=/target    # 판단 노드 구독 토픽에 맞출 때
#   ros2 launch lv2_module5/launch/perception.launch.py output:=log              # 화면 대신 ~/.ros/log 로 (bringup 대시보드용)
#   ros2 launch lv2_module5/launch/perception.launch.py perception_cpus:=1-3 camera_cpus:=0
#       # CPU 코어 지정 (taskset). perception 을 지정하면 NCNN 스레드 수도 코어 수에 맞춤
#
# bringup.launch.py에서 IncludeLaunchDescription으로 그대로 포함하면 된다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

CONFIG_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config')


def _cpu_count(cpus):
    """'1-3' → 3, '1,2,3' → 3, '0' → 1"""
    n = 0
    for part in cpus.split(','):
        a, _, b = part.strip().partition('-')
        n += (int(b) - int(a) + 1) if b else 1
    return n


def generate_launch_description():
    default_model_dir = os.path.join(
        get_package_share_directory('perception'), 'models', 'target_blue_256')
    model_dir = LaunchConfiguration('model_dir')

    camera_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        # 측정 조건과 같은 설정 (results/realtime_ncnn_vs_onnx.md 1절)
        launch_arguments={
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
            'output': LaunchConfiguration('output'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('camera')))

    def camera_group(context):
        # rs_launch 안의 노드에 taskset 을 붙이려면 launch-prefix 를 이 그룹 안에서만 설정
        cpus = LaunchConfiguration('camera_cpus').perform(context).strip()
        actions = [SetLaunchConfiguration('launch-prefix', f'taskset -c {cpus}')] if cpus else []
        return [GroupAction(actions + [camera_include], scoped=True)]

    def perception_node(context):
        # output 은 문자열로 확정해서 넘긴다 (screen | log)
        cpus = LaunchConfiguration('perception_cpus').perform(context).strip()
        extra = {'num_threads': _cpu_count(cpus)} if cpus else {}   # 지정한 코어 수 = NCNN 스레드 수
        return [Node(
        prefix=f'taskset -c {cpus}' if cpus else None,
        package='perception',
        executable='perception_master',
        name='perception_master',
        output=LaunchConfiguration('output').perform(context),
        parameters=[
            os.path.join(CONFIG_DIR, 'perception.yaml'),
            {
                'model_param': PathJoinSubstitution([model_dir, 'model.ncnn.param']),
                'model_bin': PathJoinSubstitution([model_dir, 'model.ncnn.bin']),
                'output_topic': LaunchConfiguration('output_topic'),
            },
            extra,
        ])]

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true', description='RealSense 노드도 함께 실행'),
        DeclareLaunchArgument('model_dir', default_value=default_model_dir,
                              description='model.ncnn.param / model.ncnn.bin 이 있는 폴더'),
        DeclareLaunchArgument('output_topic', default_value='/detection',
                              description='PointStamped 발행 토픽'),
        DeclareLaunchArgument('output', default_value='screen', description='screen | log'),
        DeclareLaunchArgument('camera_cpus', default_value='', description="RealSense 코어 (예: '0', 비우면 지정 안 함)"),
        DeclareLaunchArgument('perception_cpus', default_value='', description="perception 코어 (예: '1-3')"),
        OpaqueFunction(function=camera_group),
        OpaqueFunction(function=perception_node),
    ])
