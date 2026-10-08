# RealSense + perception_master 실행 (설정: ../config/camera.yaml, perception.yaml)
#
#   ros2 launch lv2_module5/launch/perception.launch.py
#   ros2 launch lv2_module5/launch/perception.launch.py camera:=false            # 카메라를 따로 띄운 경우
#   ros2 launch lv2_module5/launch/perception.launch.py model:=v3                # 모델 선택 (v4 기본 | v1 | v2 | v3)
#   ros2 launch lv2_module5/launch/perception.launch.py output_topic:=/target    # 판단 노드 구독 토픽에 맞출 때
#   ros2 launch lv2_module5/launch/perception.launch.py output:=log              # 화면 대신 ~/.ros/log 로 (bringup 대시보드용)
#   ros2 launch lv2_module5/launch/perception.launch.py perception_cpus:=1-3 camera_cpus:=0
#       # CPU 코어 지정 (taskset). perception 을 지정하면 NCNN 스레드 수도 코어 수에 맞춤
#   ros2 launch lv2_module5/launch/perception.launch.py num_threads:=4           # NCNN 스레드 수 직접 지정 (2/4 비교)
#
# bringup.launch.py에서 IncludeLaunchDescription으로 그대로 포함하면 된다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONFIG_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config')

# model:=<이름> → (패키지 models/ 아래 폴더, export 입력 높이). 입력 너비는 모두 320 (perception.yaml)
# 폴더마다 NCNN(model.ncnn.*)과 ONNX(model.onnx)가 같은 best.pt에서 export되어 있어 backend와 무관하게 같은 모델
# 카메라가 640x360(16:9)이라 v4는 imgsz=[192,320] (0.5배 축소 → 320x180 + 위아래 6px 패딩. 180은 32의 배수가 아니라 192로 올림)
# v1~v3는 4:3용 [256,320] export → 16:9 영상이면 위아래 38px 패딩으로 돌아감 (비교용)
MODELS = {
    'v1': ('target_blue_256', 256),     # perception_test/yolo/runs/target_blue
    'v2': ('target_blue_v2_256', 256),  # perception_test/yolo/runs/target_blue_v2
    'v3': ('target_blue_v3_256', 256),  # perception_test/yolo/runs/target_blue_v3_imgsz320 (4:3 데이터, 320으로 학습)
    'v4': ('target_blue_v4_192', 192),  # perception_test/yolo/runs/target_blue_v4_169 (v3 데이터를 16:9로 변환, 320으로 학습)
}


def _cpu_count(cpus):
    """'1-3' → 3, '1,2,3' → 3, '0' → 1"""
    n = 0
    for part in cpus.split(','):
        a, _, b = part.strip().partition('-')
        n += (int(b) - int(a) + 1) if b else 1
    return n


def generate_launch_description():
    default_model_dir = PathJoinSubstitution([
        get_package_share_directory('perception'), 'models',
        PythonExpression([repr(MODELS), "['", LaunchConfiguration('model'), "'][0]"])])
    default_input_height = PythonExpression([repr(MODELS), "['", LaunchConfiguration('model'), "'][1]"])
    model_dir = LaunchConfiguration('model_dir')

    camera_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        # color 640x360 = 16:9 센서 전체 (D435 실측 fx 456, HFOV 70.1°, VFOV 43.1°). 4:3(640x480)은 좌우가 잘린 55.5°
        # 화각이 320x180과 같아 모델 입력(320x180)에서는 같은 영상. 추론 전 letterbox가 0.5배 축소
        # 30 FPS: 추론이 카메라 주기보다 느리면 처리 FPS는 같고 지연만 줄어듦 (results/fps_root_cause.md H2)
        # depth는 16:9 중 가장 작은 424x240 (최소 측정 거리 ≈ 10 cm), align이 color 640x360에 맞춤
        launch_arguments={
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640x360x30',
            'depth_module.depth_profile': '424x240x30',
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
        threads = LaunchConfiguration('num_threads').perform(context).strip()
        if threads:
            extra['num_threads'] = int(threads)   # 직접 지정하면 perception.yaml · 코어 수보다 우선 (2/4 비교용)
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
                'model_onnx': PathJoinSubstitution([model_dir, 'model.onnx']),
                'output_topic': LaunchConfiguration('output_topic'),
                'input_height': ParameterValue(LaunchConfiguration('input_height'), value_type=int),
            },
            extra,
        ])]

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true', description='RealSense 노드도 함께 실행'),
        DeclareLaunchArgument('model', default_value='v4', choices=list(MODELS),
                              description='배포 모델 선택 (model_dir를 주면 무시)'),
        DeclareLaunchArgument('model_dir', default_value=default_model_dir,
                              description='model.ncnn.param / model.ncnn.bin / model.onnx 가 있는 폴더'),
        DeclareLaunchArgument('input_height', default_value=default_input_height,
                              description='모델 입력 높이 (export imgsz의 높이). model_dir를 직접 줄 때 맞춰 줄 것'),
        DeclareLaunchArgument('output_topic', default_value='/detection',
                              description='PointStamped 발행 토픽'),
        DeclareLaunchArgument('output', default_value='screen', description='screen | log'),
        DeclareLaunchArgument('camera_cpus', default_value='', description="RealSense 코어 (예: '0', 비우면 지정 안 함)"),
        DeclareLaunchArgument('perception_cpus', default_value='', description="perception 코어 (예: '1-3')"),
        DeclareLaunchArgument('num_threads', default_value='',
                              description="NCNN 스레드 수 (예: '2', '4'. 비우면 perception.yaml 값)"),
        OpaqueFunction(function=camera_group),
        OpaqueFunction(function=perception_node),
    ])
