# RealSense + perception_master 실행 (설정: ../config/camera.yaml, perception.yaml)
#
#   ros2 launch lv2_module5/launch/perception.launch.py
#   ros2 launch lv2_module5/launch/perception.launch.py camera:=false            # 카메라를 따로 띄운 경우
#   ros2 launch lv2_module5/launch/perception.launch.py output_topic:=/target    # 판단 노드 구독 토픽에 맞출 때
#
# bringup.launch.py에서 IncludeLaunchDescription으로 그대로 포함하면 된다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

CONFIG_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config')


def generate_launch_description():
    default_model_dir = os.path.join(
        get_package_share_directory('perception'), 'models', 'target_blue_256')
    model_dir = LaunchConfiguration('model_dir')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        # 측정 조건과 같은 설정 (results/realtime_ncnn_vs_onnx.md 1절)
        launch_arguments={
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
        }.items(),
        condition=IfCondition(LaunchConfiguration('camera')))

    perception = Node(
        package='perception',
        executable='perception_master',
        name='perception_master',
        output='screen',
        parameters=[
            os.path.join(CONFIG_DIR, 'perception.yaml'),
            {
                'model_param': PathJoinSubstitution([model_dir, 'model.ncnn.param']),
                'model_bin': PathJoinSubstitution([model_dir, 'model.ncnn.bin']),
                'output_topic': LaunchConfiguration('output_topic'),
            },
        ])

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true', description='RealSense 노드도 함께 실행'),
        DeclareLaunchArgument('model_dir', default_value=default_model_dir,
                              description='model.ncnn.param / model.ncnn.bin 이 있는 폴더'),
        DeclareLaunchArgument('output_topic', default_value='/detection',
                              description='PointStamped 발행 토픽'),
        camera,
        perception,
    ])
