"""dashboard.py — planning_master 터미널 대시보드 (실기 · 시뮬 공용)

planning_master 를 dashboard:=true 로 실행하면 0.2 s 마다 화면을 갱신한다.
  - 현재 상태 · reason · 진단 · 입력별 마지막 수신 시간 · 추정 위치 · 출력 명령
  - 상태 전이 기록: 매 제어 주기마다 검사하므로 한 주기(약 33 ms)만 스친 FAULT 도 남는다
event_log 경로를 주면 전이 기록을 파일에도 남긴다 (색 코드 없이, 화면에서 밀려나도 확인 가능).

실행 예
  ros2 run planning planning_master --ros-args --params-file lv2_module5/config/planning.yaml -p dashboard:=true
  ... -p dashboard:=true -p event_log:=results/logs/planning_events.txt
"""
import os
import re
import sys
import time

_ANSI = re.compile(r'\033\[[0-9;]*[A-Za-z]|\x1b\[[0-9;]*[A-Za-z]')


class StatusDashboard:
    """터미널 대시보드. 상태 전이는 매 제어 주기마다 기록하고, 화면은 0.2 s 마다 갱신."""

    COLORS = {'idle': '\033[37m', 'tracking': '\033[32m', 'searching': '\033[33m', 'lost': '\033[31m',
              'fault': '\033[35m'}
    B, D, R, G, Y, C, X = '\033[1m', '\033[2m', '\033[31m', '\033[32m', '\033[33m', '\033[36m', '\033[0m'

    def __init__(self, node, max_events=10, log_path=''):
        self.n = node
        self.log = None
        if log_path:
            os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
            self.log = open(log_path, 'a', buffering=1)       # 줄 단위로 바로 기록
            self.log.write(f'# planning events {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
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
        key = (n.state, n.status_reason())
        if key != self.prev:
            if self.prev is None or key[0] != self.prev[0]:
                self.state_since = time.monotonic()
            frm = self.prev[0].upper() if self.prev else 'START'
            col = self.COLORS.get(n.state, '')
            extra = ''
            if n.state == 'searching' and (self.prev is None or self.prev[0] != 'searching'):
                extra = f' (lost_side={n.lost_side})'
            self.events.append(f'{self._t():7.2f}s  {frm:>9} → {col}{n.state.upper():<9}{self.X} '
                               f'reason={n.status_reason()}{extra}')
            self.events = self.events[-self.max_events:]
            if self.log is not None:
                self.log.write(time.strftime('%H:%M:%S ') + _ANSI.sub('', self.events[-1]) + '\n')
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
        reason = n.status_reason()
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
            f'reason={self.Y if reason != "ok" else ""}'
            f'{self.G if reason.startswith("recovered_from") else ""}{reason}{self.X}   '
            f'diag={self.G if n.health_enabled else self.Y}{n.diag_text()}{self.X}',
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
