#!/usr/bin/env python3
"""fake_perception.py — 가상 인지 모듈 (RealSense + perception_master 대체)

아래 토픽 앞에는 모두 topic_prefix(기본 /sim)가 붙는다. 실제 노드와 섞이지 않게 하기 위함.
(fake_planning.py 가 planning 의 토픽도 같은 접두사로 바꿔 연결한다.)

가상 세계에 퍽 하나를 두고, fake_control 이 발행하는 실제 자세(/sim/truth)로
카메라에 퍽이 어떻게 보이는지 계산해 perception_master 와 같은 형식으로 발행한다.

  구독  /sim/truth   std_msgs/Float32MultiArray  [x, y, yaw_rad, pan_deg, tilt_deg]  (fake_control)
  발행  /detection   geometry_msgs/PointStamped  best-effort, depth 1, 30 Hz
            x = e_x, y = e_y  (정규화 중심 오차, 오른쪽·아래 +)
            z = depth [m]     검출 + depth 0.2~3.0 m
            z = 0             미검출 / 가림 / depth 범위 밖
        /perception/camera_health  diagnostic_msgs/DiagnosticStatus  ('h' 키로 켤 때만)
        /puck_truth  std_msgs/Float32MultiArray  [x, y, z, 보임]  시각화 전용 (sim_viz.py)

카메라 모델 (planning_master 의 역변환과 같은 선형 화각 근사)
  카메라 원점 = 로봇 원점(pan/tilt 축), 높이 cam_height(기본 0.20 m), 퍽은 바닥(z=0)에서 시작
  θ = atan(l/f), φ = atan(u/f)  →  e_x = -θ/(HFOV/2),  e_y = -φ/(VFOV/2),  depth = f

실행 (ROS 2 환경 source 후, fake_control 먼저 실행)
  python3 fake_perception.py
"""
import math
import os
import random
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray

try:
    from diagnostic_msgs.msg import DiagnosticStatus
except ImportError:
    DiagnosticStatus = None


class Keyboard:
    """터미널을 cbreak 모드로 바꿔 키 하나씩 non-blocking 으로 읽는다. Ctrl+C 는 그대로 동작."""

    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self._old = None

    def __enter__(self):
        if self.enabled:
            self._old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *exc):
        if self._old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)

    def keys(self):
        out = []
        while self.enabled and select.select([sys.stdin], [], [], 0)[0]:
            ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
            if ch:
                out.append(ch)
        return out


C = {'g': '\033[32m', 'r': '\033[31m', 'y': '\033[33m', 'c': '\033[36m', 'b': '\033[1m', 'd': '\033[2m', '0': '\033[0m'}


def onoff(flag, on='ON', off='OFF', bad_when_on=False):
    good = not flag if bad_when_on else flag
    return f"{C['g'] if good else C['r']}{on if flag else off}{C['0']}"


class FakePerception(Node):
    def __init__(self, kb):
        super().__init__('fake_perception')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.rate_hz = float(p('rate_hz', 30.0))
        self.hfov = float(p('hfov_deg', 69.0))
        self.vfov = float(p('vfov_deg', 42.0))
        self.cam_height = float(p('cam_height', 0.20))     # 카메라(=pan/tilt 축) 높이 [m]
        self.min_depth = float(p('min_depth', 0.2))
        self.max_depth = float(p('max_depth', 3.0))
        self.pan_forward = float(p('pan_forward_deg', -0.09))
        self.kb = kb
        pre = str(p('topic_prefix', '/sim')).rstrip('/')   # 실제 노드와 섞이지 않게 모든 토픽 앞에 붙임

        self.truth = None                       # [x, y, yaw, pan, tilt]
        self.truth_t = None
        self.puck = [1.0, 0.0, 0.0]             # 월드 좌표 [m]
        self.puck_backup = None                 # 'g' 로 치웠을 때 원래 위치

        # 스위치
        self.publishing = True                  # p: /detection 발행 자체를 멈춤 (토픽 침묵)
        self.occluded = False                   # o: 가림 (z=0 계속 발행)
        self.occlude_until = 0.0                # t: 2초 가림
        self.invalid = False                    # n: NaN 발행 (규약 위반 입력 시험)
        self.noise = False                      # f: 측정 노이즈 + 2 % 미검출
        self.circle = False                     # m: 퍽 원운동
        self.circle_center = None
        self.circle_phase = 0.0
        self.dash = None                        # [ / ]: 옆으로 빠르게 빠져나감 (vx, vy, 남은 시간)
        self.health_on = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(PointStamped, pre + '/detection', qos)
        self.pub_health = (self.create_publisher(DiagnosticStatus, pre + '/perception/camera_health', 10)
                           if DiagnosticStatus is not None else None)
        self.sub = self.create_subscription(Float32MultiArray, pre + '/truth', self.truth_cb, 10)
        # 시각화용: 퍽의 실제 월드 위치 [x, y, z, 보임(1)/안보임(0)] (sim_viz.py 가 구독)
        self.pub_puck = self.create_publisher(Float32MultiArray, pre + '/puck_truth', 10)

        self.last = (0.0, 0.0, 0.0)
        self.view = '-'
        self.n_pub = 0
        self.events = []
        self.last_step = time.monotonic()
        self.create_timer(1.0 / self.rate_hz, self.step)
        self.create_timer(0.05, self.poll_keys)
        self.create_timer(0.2, self.draw)

    def truth_cb(self, m):
        if len(m.data) >= 5:
            self.truth = list(m.data[:5])
            self.truth_t = time.monotonic()

    # ───────── 카메라 모델 ─────────
    def observe(self):
        """(e_x, e_y, depth, 설명) — 안 보이면 depth=0."""
        x, y, yaw, pan, tilt = self.truth if self.truth else (0.0, 0.0, 0.0, 0.0, 0.0)
        dx, dy = self.puck[0] - x, self.puck[1] - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        rx, ry, rz = c * dx - s * dy, s * dx + c * dy, self.puck[2] - self.cam_height   # 로봇 기준
        P, T = math.radians(pan - self.pan_forward), math.radians(tilt)
        a = math.cos(-P) * rx - math.sin(-P) * ry                                        # Rz(-pan)
        l = math.sin(-P) * rx + math.cos(-P) * ry
        f = math.cos(T) * a + math.sin(T) * rz                                           # Ry(tilt)
        u = -math.sin(T) * a + math.cos(T) * rz
        if f <= 0.05:
            return 0.0, 0.0, 0.0, '카메라 뒤쪽'
        th, ph = math.degrees(math.atan(l / f)), math.degrees(math.atan(u / f))
        if abs(th) > self.hfov / 2:
            return 0.0, 0.0, 0.0, f'화각 밖 (수평 {th:+.0f}°)'
        if abs(ph) > self.vfov / 2:
            return 0.0, 0.0, 0.0, f'화각 밖 (수직 {ph:+.0f}°)'
        if not self.min_depth <= f <= self.max_depth:
            return 0.0, 0.0, 0.0, f'depth 범위 밖 ({f:.2f} m)'
        return -th / (self.hfov / 2), -ph / (self.vfov / 2), f, '보임'

    def robot_frame_offset(self, fwd, left):
        yaw = self.truth[2] if self.truth else 0.0
        return fwd * math.cos(yaw) - left * math.sin(yaw), fwd * math.sin(yaw) + left * math.cos(yaw)

    # ───────── 주기 실행 ─────────
    def step(self):
        now = time.monotonic()
        dt = min(now - self.last_step, 0.1)
        self.last_step = now
        if self.circle and self.circle_center:
            self.circle_phase += 0.4 * dt                    # 반경 0.5 m, 0.4 rad/s (0.2 m/s)
            cx, cy = self.circle_center
            self.puck[0] = cx + 0.5 * math.cos(self.circle_phase)
            self.puck[1] = cy + 0.5 * math.sin(self.circle_phase)
        if self.dash:
            vx, vy, remain = self.dash
            self.puck[0] += vx * dt
            self.puck[1] += vy * dt
            self.dash = (vx, vy, remain - dt) if remain - dt > 0 else None

        ex, ey, z, self.view = self.observe()
        if self.occluded or now < self.occlude_until:
            ex = ey = z = 0.0
            self.view = '가림'
        if self.noise and z > 0:
            if random.random() < 0.02:
                ex = ey = z = 0.0
                self.view = '노이즈 미검출'
            else:
                ex += random.gauss(0, 0.01)
                ey += random.gauss(0, 0.01)
                z += random.gauss(0, 0.01)
        if self.invalid:
            ex, ey, z = float('nan'), float('nan'), float('nan')
            self.view = 'NaN 주입'
        self.last = (ex, ey, z)
        seen = 1.0 if (self.publishing and math.isfinite(z) and z > 0) else 0.0
        self.pub_puck.publish(Float32MultiArray(data=[float(v) for v in self.puck] + [seen]))

        if self.publishing:
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.point.x, msg.point.y, msg.point.z = float(ex), float(ey), float(z)
            self.pub.publish(msg)
            self.n_pub += 1
        if self.health_on and self.pub_health is not None:
            st = DiagnosticStatus()
            st.name = 'fake_perception/camera'
            st.hardware_id = 'fake'
            st.level = DiagnosticStatus.OK if self.publishing else DiagnosticStatus.STALE
            st.message = 'ok' if self.publishing else 'stream stopped'
            self.pub_health.publish(st)

    # ───────── 키보드 ─────────
    def move(self, fwd, left):
        dx, dy = self.robot_frame_offset(fwd, left)
        self.puck[0] += dx
        self.puck[1] += dy
        self.circle = False

    def poll_keys(self):
        for k in self.kb.keys():
            if k in 'wasdWASD':
                step = 0.5 if k.isupper() else 0.1
                fwd, left = {'w': (step, 0), 's': (-step, 0), 'a': (0, step), 'd': (0, -step)}[k.lower()]
                self.move(fwd, left)
                self.log(f'퍽 이동 (로봇 기준 앞 {fwd:+.1f}, 왼쪽 {left:+.1f} m)')
            elif k == 'z':
                self.puck[2] += 0.1; self.log(f'퍽 높이 {self.puck[2]:.1f} m')
            elif k == 'x':
                self.puck[2] = max(0.0, self.puck[2] - 0.1); self.log(f'퍽 높이 {self.puck[2]:.1f} m')
            elif k in '[]':
                sgn = 1.0 if k == '[' else -1.0                    # [ 왼쪽, ] 오른쪽
                vx, vy = self.robot_frame_offset(0.0, 5.0 * sgn)
                self.dash = (vx, vy, 0.4)
                self.circle = False
                self.log(f'퍽이 {"왼쪽" if sgn > 0 else "오른쪽"}으로 빠르게 빠져나감 (5 m/s × 0.4 s)')
            elif k == 'o':
                self.occluded = not self.occluded; self.log(f'가림 {"ON" if self.occluded else "OFF"}')
            elif k == 't':
                self.occlude_until = time.monotonic() + 2.0; self.log('2초 가림')
            elif k == 'g':
                if self.puck_backup is None:
                    self.puck_backup = list(self.puck); self.puck = [100.0, 100.0, 0.0]; self.log('퍽 제거')
                else:
                    self.puck = self.puck_backup; self.puck_backup = None; self.log('퍽 원위치')
            elif k == 'p':
                self.publishing = not self.publishing; self.log(f'/detection 발행 {"ON" if self.publishing else "OFF (토픽 침묵)"}')
            elif k == 'n':
                self.invalid = not self.invalid; self.log(f'NaN 주입 {"ON" if self.invalid else "OFF"}')
            elif k == 'f':
                self.noise = not self.noise; self.log(f'노이즈 {"ON" if self.noise else "OFF"}')
            elif k == 'm':
                self.circle = not self.circle
                if self.circle:
                    # 로봇 반대쪽 0.5 m 를 중심으로 반경 0.5 m 원 → 퍽이 로봇 밑으로 지나가지 않음
                    x, y = self.truth[:2] if self.truth else (0.0, 0.0)
                    d = math.hypot(self.puck[0] - x, self.puck[1] - y) or 1.0
                    ux, uy = (self.puck[0] - x) / d, (self.puck[1] - y) / d
                    self.circle_center = (self.puck[0] + 0.5 * ux, self.puck[1] + 0.5 * uy)
                    self.circle_phase = math.atan2(-uy, -ux)
                self.log(f'원운동 {"ON" if self.circle else "OFF"}')
            elif k == 'h':
                if self.pub_health is None:
                    self.log('diagnostic_msgs 없음: health 발행 불가')
                else:
                    self.health_on = not self.health_on; self.log(f'camera_health 발행 {"ON" if self.health_on else "OFF"}')
            elif k == 'r':
                dx, dy = self.robot_frame_offset(1.0, 0.0)
                base = self.truth[:2] if self.truth else (0.0, 0.0)
                self.puck = [base[0] + dx, base[1] + dy, 0.0]
                self.puck_backup = None; self.circle = False; self.dash = None
                self.log('퍽을 로봇 앞 1 m 로 리셋')

    def log(self, text):
        self.events.append(f'{time.strftime("%H:%M:%S")}  {text}')
        self.events = self.events[-6:]

    def draw(self):
        if not self.kb.enabled:
            return
        ex, ey, z = self.last
        truth_ok = self.truth_t is not None and time.monotonic() - self.truth_t < 0.5
        if self.truth:
            x, y, yaw, pan, tilt = self.truth
            dist = math.hypot(self.puck[0] - x, self.puck[1] - y)
            bearing = math.degrees(math.atan2(self.puck[1] - y, self.puck[0] - x) - yaw)
            bearing = (bearing + 180) % 360 - 180
            rel = f'거리 {dist:4.2f} m, 방위 {bearing:+6.1f}° (로봇 기준, 왼쪽+)'
        else:
            rel = '-'
        seen = z > 0 and math.isfinite(z)
        lines = [
            '\033[H\033[J' + f"{C['b']}{C['c']}━━━━━━━━━━━━━━━━  FAKE PERCEPTION  ━━━━━━━━━━━━━━━━{C['0']}",
            f" /sim/truth   {onoff(truth_ok, '수신 중', '없음 (fake_control 실행?)')}",
            f" 퍽 (월드)    x {self.puck[0]:+6.2f}  y {self.puck[1]:+6.2f}  높이 {self.puck[2]:.1f} m",
            f" 퍽 (상대)    {rel}",
            f" 카메라       {onoff(seen, 'DETECTED', 'NO DETECTION')}  ({self.view})",
            f" /detection   e_x {ex:+6.3f}   e_y {ey:+6.3f}   z {z:6.3f}   발행 {self.n_pub}회",
            '',
            f"{C['b']} 키{C['0']}",
            "  [w/a/s/d] 퍽 이동 0.1 m (로봇 기준 앞/왼/뒤/오)   대문자 0.5 m",
            "  [ [ / ] ] 왼쪽/오른쪽으로 빠르게 빠져나감          [z/x] 퍽 높이 ±0.1 m",
            f"  [o] 가림        {onoff(self.occluded, bad_when_on=True)}     [t] 2초 가림",
            f"  [g] 퍽 제거     {onoff(self.puck_backup is not None, bad_when_on=True)}     "
            f"[p] /detection 발행 {onoff(self.publishing)}",
            f"  [n] NaN 주입    {onoff(self.invalid, bad_when_on=True)}     [f] 노이즈 {onoff(self.noise, bad_when_on=True)}"
            f"     [m] 원운동 {onoff(self.circle)}",
            f"  [h] camera_health 발행 {onoff(self.health_on)}     [r] 퍽을 로봇 앞 1 m 로     Ctrl+C 종료",
            '',
            f"{C['b']} 최근 이벤트{C['0']}",
        ] + [f"  {C['d']}{e}{C['0']}" for e in self.events]
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main():
    rclpy.init()
    with Keyboard() as kb:
        node = FakePerception(kb)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
