"""dashboard.py — planning_master 터미널 대시보드 (실기 · 시뮬 공용)

planning_master 를 dashboard:=true 로 실행하면 0.2 s 마다 화면을 갱신한다.
  - 현재 상태 · reason · 진단 · 입력별 마지막 수신 시간 · 추정 위치 · 출력 명령
  - 상태 전이 기록: 매 제어 주기마다 검사하므로 한 주기(약 33 ms)만 스친 FAULT 도 남는다
전이 기록 파일(event_log)은 planning_master 가 직접 남긴다 (대시보드를 꺼도 동작).

실행 예
  ros2 run planning planning_master --ros-args --params-file lv2_module5/config/planning.yaml -p dashboard:=true
"""
import sys
import time
from collections import deque

from rclpy.parameter_client import AsyncParameterClient


def _read_cpu_times():
    """/proc/stat 의 코어별 (전체, 쉰 시간) jiffies. [전체 합계, cpu0, cpu1, ...] 순서."""
    out = []
    try:
        with open('/proc/stat') as f:
            for line in f:
                if not line.startswith('cpu'):
                    break
                v = [int(x) for x in line.split()[1:]]
                idle = v[3] + (v[4] if len(v) > 4 else 0)        # idle + iowait
                out.append((sum(v[:8]), idle))                   # guest 는 user 에 이미 포함
    except OSError:
        pass
    return out


def _read_int(path, scale):
    try:
        with open(path) as f:
            return int(f.read().strip()) / scale
    except (OSError, ValueError):
        return None


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
        # 라즈베리파이 CPU · /detection 수신 주기 · perception 스레드 수 (스레드 2/4 비교용)
        self.cpu_prev = _read_cpu_times()
        self.cpu_pct = []                       # [전체, cpu0, cpu1, ...] %
        self.det_times = deque()                # 최근 det_window_s 동안 /detection 수신 시각
        self.det_window_s = 10.0
        self.last_det_seen = None
        self.perc_threads = None                # perception_master num_threads (파라미터로 읽음)
        self.perc_params = AsyncParameterClient(node, 'perception_master')
        node.create_timer(0.2, self.draw)
        node.create_timer(1.0, self.update_cpu)
        node.create_timer(3.0, self.query_threads)

    def _t(self):
        return time.monotonic() - self.t0

    def on_cycle(self):
        n = self.n
        # /detection 새 메시지마다 수신 시각 기록 (제어 주기 30 Hz > 인지 약 7 Hz 라 놓치지 않음)
        if n.last_cam_time is not None and n.last_cam_time != self.last_det_seen:
            self.last_det_seen = n.last_cam_time
            self.det_times.append(n.last_cam_time)
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
            self.prev = key

    def update_cpu(self):
        """1 초 동안의 코어별 사용률 = 1 − (쉰 시간 증가 / 전체 시간 증가)."""
        cur = _read_cpu_times()
        if len(cur) == len(self.cpu_prev):
            pct = []
            for (t1, i1), (t0, i0) in zip(cur, self.cpu_prev):
                dt = t1 - t0
                pct.append(100.0 * (1.0 - (i1 - i0) / dt) if dt > 0 else 0.0)
            self.cpu_pct = pct
        self.cpu_prev = cur

    def query_threads(self):
        """perception_master 의 num_threads (시작할 때 읽는 값이라 3 초마다 한 번 확인이면 충분)."""
        if not self.perc_params.services_are_ready():
            self.perc_threads = None
            return
        fut = self.perc_params.get_parameters(['num_threads'])
        fut.add_done_callback(self._threads_done)

    def _threads_done(self, fut):
        try:
            self.perc_threads = fut.result().values[0].integer_value
        except Exception:   # 응답 실패 · 파라미터 없음
            self.perc_threads = None

    def _det_stats(self):
        """최근 det_window_s 초 동안 /detection 평균 Hz 와 최대 수신 간격 [s]."""
        now = self.n._now_s()
        while self.det_times and now - self.det_times[0] > self.det_window_s:
            self.det_times.popleft()
        t = list(self.det_times)
        if len(t) < 2:
            return None, None
        gaps = [b - a for a, b in zip(t, t[1:])]
        return (len(t) - 1) / (t[-1] - t[0]), max(gaps)

    def _bar(self, pct, width=20):
        fill = int(round(max(0.0, min(100.0, pct)) / 100.0 * width))
        col = self.G if pct < 60 else (self.Y if pct < 85 else self.R)
        return f'{col}{"█" * fill}{self.D}{"░" * (width - fill)}{self.X} {pct:5.1f}%'

    def _system_lines(self):
        lines = [f'{self.B}  라즈베리파이 · 인지 처리{self.X}']
        if self.cpu_pct:
            lines.append(f'   CPU 전체  {self._bar(self.cpu_pct[0])}')
            lines += [f'   코어 {i}    {self._bar(p)}' for i, p in enumerate(self.cpu_pct[1:])]
        else:
            lines.append('   CPU      측정 중…')
        temp = _read_int('/sys/class/thermal/thermal_zone0/temp', 1000.0)
        mhz = _read_int('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq', 1000.0)
        lines.append(f'   온도 {f"{temp:4.1f}°C" if temp is not None else "-"}   '
                     f'클럭 {f"{mhz:4.0f} MHz" if mhz is not None else "-"}   (클럭이 내려가면 발열 스로틀링)')
        hz, gap = self._det_stats()
        timeout = self.n.detection_timeout_s
        gap_txt = '-' if gap is None else (
            f'{self.G if gap < timeout * 0.8 else self.R}{gap:4.2f} s{self.X}')
        lines.append(f'   perception 스레드 {self.perc_threads if self.perc_threads is not None else "-"}   '
                     f'/detection 최근 {self.det_window_s:.0f}s 평균 {f"{hz:4.1f} Hz" if hz else "-"}   '
                     f'최대 간격 {gap_txt} (타임아웃 {timeout:.1f} s)')
        return lines

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
        ] + self._system_lines() + [
            '',
            f'{self.B}  상태 전이 기록{self.X}',
        ] + [f'   {e}' for e in self.events] + ['', f'{self.D}  Ctrl+C 종료{self.X}']
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()
