#!/usr/bin/env python3
"""fake_planning.py — 시뮬레이션용 planning 실행기

실제 planning/planning_master.py 의 PlanningMaster 를 그대로 가져와 실행한다 (로직 복사 없음).
다른 점은 두 가지뿐이다.
  1. 모든 입출력 토픽 앞에 /sim 을 붙여 실제 로봇 노드와 섞이지 않게 한다.
       /sim/detection, /sim/control/imu, /sim/control/joint_states, /sim/control/odom_yaw_deg,
       /sim/planning/cmd_vel, /sim/planning/arm_command, /sim/tracking_status, /sim/.../health
  2. 터미널에 상태 대시보드를 출력한다 (planning/dashboard.py, 실기에서도 -p dashboard:=true 로 같은 화면).

실행 (ROS 2 환경 source 후, fake_control · fake_perception 과 각각 다른 터미널에서)
  python3 fake_planning.py                         # 파라미터는 config/planning.yaml, 진단 검사 OFF (diag=off[...])
  python3 fake_planning.py --health                # health 진단 검사 켜기 (fake 노드에서 'h' 로 발행)
  python3 fake_planning.py --ros-args -p search_angular_vel:=0.5   # 파라미터 덮어쓰기
"""
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE5 = os.path.normpath(os.path.join(HERE, '..', '..'))                 # lv2_module5/
PLANNING_SRC = os.path.join(MODULE5, 'ros2_ws', 'src', 'planning')
PARAMS_FILE = os.path.join(MODULE5, 'config', 'planning.yaml')
PREFIX = '/sim'

try:
    from planning.planning_master import PlanningMaster              # colcon 빌드 후 source 했으면
except ImportError:
    sys.path.insert(0, PLANNING_SRC)                                  # 빌드 안 했으면 소스에서 직접
    from planning.planning_master import PlanningMaster

TOPIC_PARAMS = {
    'detection_topic': '/detection',
    'imu_topic': '/control/imu',
    'odom_yaw_topic': '/control/odom_yaw_deg',
    'joint_states_topic': '/control/joint_states',
    'cmd_vel_topic': '/planning/cmd_vel',
    'arm_cmd_topic': '/planning/arm_command',
    'status_topic': '/tracking_status',
    'health.topics.camera': '/perception/camera_health',
    'health.topics.imu': '/control/imu_health',
    'health.topics.arm_motor': '/control/arm_motor_health',
    'health.topics.wheel_motor': '/control/wheel_motor_health',
    'health.topics.mcu': '/control/opencr',
}


class FakePlanning(PlanningMaster):
    """실제 PlanningMaster 그대로. 대시보드는 dashboard:=true 파라미터로 planning_master 가 직접 띄운다."""


def build_args(argv):
    health = '--health' in argv
    user = [a for a in argv[1:] if a != '--health']
    args = [argv[0]]
    if '--ros-args' in user:                                         # 사용자가 준 ros-args 는 맨 뒤에서 우선 적용
        i = user.index('--ros-args')
        pre_user, user_ros = user[:i], user[i:]
    else:
        pre_user, user_ros = user, []
    args += pre_user + ['--ros-args', '-r', '__node:=planning_master']
    if os.path.exists(PARAMS_FILE):
        args += ['--params-file', PARAMS_FILE]
    for name, topic in TOPIC_PARAMS.items():
        args += ['-p', f'{name}:={PREFIX}{topic}']
    args += ['-p', f'health.enabled:={"true" if health else "false"}']
    args += ['-p', 'dashboard:=true']
    return args + user_ros


def main():
    rclpy.init(args=build_args(sys.argv))
    node = FakePlanning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_and_publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
