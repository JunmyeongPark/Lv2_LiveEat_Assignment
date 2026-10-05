#!/usr/bin/env python3
"""fake_planning.py — 시뮬레이션용 planning 실행기

실제 planning/planning_master.py 의 PlanningMaster 를 그대로 가져와 실행한다 (로직 복사 없음).
다른 점은 두 가지뿐이다.
  1. 모든 입출력 토픽 앞에 /sim 을 붙여 실제 로봇 노드와 섞이지 않게 한다.
       /sim/detection, /sim/control/imu, /sim/control/joint_states, /sim/control/odom_yaw_deg,
       /sim/planning/cmd_vel, /sim/planning/arm_command, /sim/tracking_status, /sim/.../health
  2. 터미널에 상태 대시보드를 출력한다.

실행 (ROS 2 환경 source 후, fake_control · fake_perception 과 각각 다른 터미널에서)
  python3 fake_planning.py                         # 파라미터는 config/planning.yaml, health 꺼짐
  python3 fake_planning.py --health                # health 진단 검사 켜기 (fake 노드에서 'h' 로 발행)
  python3 fake_planning.py --ros-args -p search_angular_vel:=0.5   # 파라미터 덮어쓰기
"""
import os
import sys
import time

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


class StatusDashboard:
    """터미널 대시보드. 상태 전이는 매 제어 주기마다 기록하고, 화면은 0.2 s 마다 갱신."""

    COLORS = {'idle': '\033[37m', 'tracking': '\033[32m', 'searching': '\033[33m', 'lost': '\033[31m',
              'fault': '\033[35m'}
    B, D, R, G, Y, C, X = '\033[1m', '\033[2m', '\033[31m', '\033[32m', '\033[33m', '\033[36m', '\033[0m'

    def __init__(self, node, max_events=10):
        self.n = node
        self.t0 = time.monotonic()
        self.events = []
        self.max_events = max_events
        self.prev = None
        self.state_since = self.t0
        node.create_timer(0.2, self.draw)

    def _t(self):
        return time.monotonic() - self.t0

    def on_cycle(self):
        n = self.n
        key = (n.state, n.health_reason)
        if key != self.prev:
            if self.prev is None or key[0] != self.prev[0]:
                self.state_since = time.monotonic()
            frm = self.prev[0].upper() if self.prev else 'START'
            col = self.COLORS.get(n.state, '')
            extra = ''
            if n.state == 'searching' and (self.prev is None or self.prev[0] != 'searching'):
                extra = f' (lost_side={n.lost_side})'
            self.events.append(f'{self._t():7.2f}s  {frm:>9} → {col}{n.state.upper():<9}{self.X} '
                               f'reason={n.health_reason}{extra}')
            self.events = self.events[-self.max_events:]
            self.prev = key

    def _age(self, last):
        if last is None:
            return f'{self.R}  none{self.X}'
        age = self.n._now_s() - last
        return f'{self.G if age <= 0.5 else self.R}{age:5.2f}s{self.X}'

    def draw(self):
        n = self.n
        st = n.state
        col = self.COLORS.get(st, '')
        reason = n.health_reason
        if reason == 'ok' and st == 'lost':
            reason = 'target_lost'
        obj = (f'거리 {n.cur_object_planar_dis:4.2f} m   방위 {n.cur_object_yaw_deg:+6.1f}°'
               if n.cur_object_planar_dis is not None and n.cur_object_yaw_deg is not None else '-')
        search = ''
        if st == 'searching':
            search = (f'   탐색: {n.lost_side} → ω {n.cmd_vel_msg.angular.z:+.2f}, '
                      f'누적 {abs(n.search_rot_deg):5.0f}° / {360 * n.search_max_turns}°')
        streak = '■' * n.detect_streak + '□' * max(0, n.recover_frames - n.detect_streak)
        lines = [
            '\033[H\033[J' + f'{self.B}{self.C}━━━━━━━━━━━━━━━━━━━━  PLANNING MASTER  ━━━━━━━━━━━━━━━━━━━━{self.X}',
            f'  STATE  {self.B}{col}{st.upper():<10}{self.X} {time.monotonic() - self.state_since:6.1f}s   '
            f'reason={self.Y if reason not in ("ok", "health_disabled") else ""}{reason}{self.X}   '
            f'health={"on" if n.health_enabled else "off"}',
            f'  status {self.D}{n._status_text()}{self.X}',
            '',
            f'{self.B}  입력{self.X}                 마지막 수신',
            f'   /detection          {self._age(n.last_cam_time)}   e_x {n.cur_bbox_error_x:+6.3f}  e_y {n.cur_bbox_error_y:+6.3f}'
            f'  depth {n.cur_depth:5.2f}  연속 {streak}',
            f'   /control/imu        {self._age(n.last_imu_time)}   yaw {n.cur_imu_yaw_deg:+7.1f}°',
            f'   /control/odom_yaw   {self._age(n.last_odom_time)}   yaw {n.cur_encoder_yaw_deg:+7.1f}°'
            f'   heading 소스: {self.G if n.heading_source == "imu" else self.Y}{n.heading_source}{self.X}',
            f'   /control/joint      {self._age(n.last_joint_time)}   pan {n.cur_arm_pose[0]:+6.1f}°  tilt {n.cur_arm_pose[1]:+6.1f}°'
            f'   {"" if n.cur_arm_pose_valid else self.R + "INVALID" + self.X}',
            '',
            f'{self.B}  추정 · 출력{self.X}',
            f'   물체      {obj}',
            f'   cmd_vel   v {n.cmd_vel_msg.linear.x:+5.2f} m/s   ω {n.cmd_vel_msg.angular.z:+5.2f} rad/s{search}',
            f'   arm cmd   pan {n.tgt_arm_pose[0]:+6.1f}°  tilt {n.tgt_arm_pose[1]:+6.1f}°'
            f'   {"" if n.arm_publish_enabled else self.Y + "(발행 중지)" + self.X}',
            '',
            f'{self.B}  상태 전이 기록{self.X}',
        ] + [f'   {e}' for e in self.events] + ['', f'{self.D}  Ctrl+C 종료{self.X}']
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()



class FakePlanning(PlanningMaster):
    """실제 PlanningMaster 와 동일. 매 제어 주기 끝에 대시보드 기록만 추가."""

    def __init__(self):
        super().__init__()
        self.dashboard = StatusDashboard(self)

    def run(self):
        super().run()
        self.dashboard.on_cycle()


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
