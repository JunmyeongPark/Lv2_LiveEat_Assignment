#!/usr/bin/env python3
"""fake_control.py — 가상 제어 모듈 (control_master + OpenCR 대체)

아래 토픽 앞에는 모두 topic_prefix(기본 /sim)가 붙는다. 실제 노드와 섞이지 않게 하기 위함.
(fake_planning.py 가 planning 의 토픽도 같은 접두사로 바꿔 연결한다.)

planning 이 보내는 명령을 받아 차체·팔을 단순 운동학으로 움직이고,
control_master 와 같은 토픽·타입으로 상태를 발행한다. 키보드로 장애를 주입할 수 있다.

  구독  /planning/cmd_vel      geometry_msgs/Twist          v [m/s], ω [rad/s]
        /planning/arm_command  std_msgs/Float32MultiArray   [pan, tilt] deg
  발행  /control/imu           sensor_msgs/Imu              yaw 만 담은 쿼터니언
        /control/joint_states  sensor_msgs/JointState       wheel_left/right, arm_yaw, arm_pitch [rad]
        /control/odom          nav_msgs/Odometry            엔코더 odom (yaw, 속도)
        /control/imu_yaw_deg   std_msgs/Float32
        /control/odom_yaw_deg  std_msgs/Float32
        /control/{imu_health, arm_motor_health, wheel_motor_health, opencr}
                               diagnostic_msgs/DiagnosticStatus  ('h' 키로 켤 때만)
        /sim/truth             std_msgs/Float32MultiArray   [x, y, yaw_rad, pan_deg, tilt_deg]
                               시뮬레이션 전용 '실제 자세'. fake_perception 이 시야 계산에 사용.
                               장애 주입과 무관하게 항상 발행한다.

control_master 와 같은 안전 동작
  - /planning/cmd_vel 이 cmd_timeout_s(0.3 s) 동안 없으면 바퀴 정지
  - /planning/arm_command 가 arm_cmd_timeout_s(0.3 s) 동안 없으면 현재 자세 유지
  - OpenCR 끊김('c'): /control/* 발행 중단, 바퀴 0, 팔 유지 (펌웨어 watchdog 재현)

실행 (ROS 2 환경 source 후)
  python3 fake_control.py
"""
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Float32MultiArray

try:
    from diagnostic_msgs.msg import DiagnosticStatus
except ImportError:  # diagnostic_msgs 가 없으면 health 발행만 끈다
    DiagnosticStatus = None


# ───────────────────────── 키보드 (터미널 raw 입력) ─────────────────────────
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


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


def _yaw_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (z, w)


C = {'g': '\033[32m', 'r': '\033[31m', 'y': '\033[33m', 'c': '\033[36m', 'b': '\033[1m', 'd': '\033[2m', '0': '\033[0m'}


def onoff(flag, on='ON', off='OFF', bad_when_on=False):
    good = not flag if bad_when_on else flag
    return f"{C['g'] if good else C['r']}{on if flag else off}{C['0']}"


class FakeControl(Node):
    def __init__(self, kb):
        super().__init__('fake_control')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.rate_hz = float(p('rate_hz', 50.0))
        self.cmd_timeout = float(p('cmd_timeout_s', 0.3))
        self.arm_cmd_timeout = float(p('arm_cmd_timeout_s', 0.3))
        self.wheel_radius = float(p('wheel_radius', 0.033))
        self.wheel_sep = float(p('wheel_separation', 0.287))
        self.max_v = float(p('max_v', 0.26))
        self.max_w = float(p('max_w', 1.82))
        self.max_acc_v = float(p('max_acc_v', 0.5))
        self.max_acc_w = float(p('max_acc_w', 3.0))
        self.yaw_limit = float(p('arm_yaw_limit_deg', 120.0))
        self.pitch_min = float(p('arm_pitch_min_deg', -80.0))
        self.pitch_max = float(p('arm_pitch_max_deg', 85.0))
        self.arm_dps = float(p('arm_max_dps', 120.0))
        self.kb = kb
        pre = str(p('topic_prefix', '/sim')).rstrip('/')   # 실제 노드와 섞이지 않게 모든 토픽 앞에 붙임
        self.pre = pre

        # 실제 자세 (truth)
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.pan = self.tilt = 0.0           # deg
        self.wheel_pos = [0.0, 0.0]          # rad
        self.wheel_vel = [0.0, 0.0]
        self.encoder_yaw = 0.0               # 엔코더 적분 yaw (truth 와 같게 시작)

        # 명령
        self.cmd_v = self.cmd_w = 0.0
        self.cmd_t = None
        self.arm_goal = None
        self.arm_goal_t = None
        self.arm_hold = None

        # 장애 주입 스위치
        self.imu_on = True
        self.imu_invalid = False      # orientation_covariance[0] = -1
        self.imu_zero = False         # yaw 를 0 으로 고정 (현재 펌웨어 IMU 미통합 상태 재현)
        self.joint_on = True
        self.odom_on = True
        self.opencr_down = False
        self.motor_enable = True
        self.health_on = False
        self.health_error = False

        self.sub_cmd = self.create_subscription(Twist, pre + '/planning/cmd_vel', self.cmd_cb, 10)
        self.sub_arm = self.create_subscription(Float32MultiArray, pre + '/planning/arm_command', self.arm_cb, 10)
        self.pub_imu = self.create_publisher(Imu, pre + '/control/imu', 10)
        self.pub_js = self.create_publisher(JointState, pre + '/control/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, pre + '/control/odom', 10)
        self.pub_imu_deg = self.create_publisher(Float32, pre + '/control/imu_yaw_deg', 10)
        self.pub_odom_deg = self.create_publisher(Float32, pre + '/control/odom_yaw_deg', 10)
        self.pub_truth = self.create_publisher(Float32MultiArray, pre + '/truth', 10)
        self.pub_health = {}
        if DiagnosticStatus is not None:
            for name, topic in (('imu', pre + '/control/imu_health'), ('arm_motor', pre + '/control/arm_motor_health'),
                                ('wheel_motor', pre + '/control/wheel_motor_health'), ('mcu', pre + '/control/opencr')):
                self.pub_health[name] = self.create_publisher(DiagnosticStatus, topic, 10)

        self.last_t = time.monotonic()
        self.n_cmd = 0
        self.events = []
        self.create_timer(1.0 / self.rate_hz, self.step)
        self.create_timer(0.05, self.poll_keys)
        self.create_timer(0.2, self.draw)

    # ───────── 구독 ─────────
    def cmd_cb(self, m):
        self.cmd_v, self.cmd_w = m.linear.x, m.angular.z
        self.cmd_t = time.monotonic()
        self.n_cmd += 1

    def arm_cb(self, m):
        if len(m.data) >= 2 and all(math.isfinite(v) for v in m.data[:2]):
            self.arm_goal = (float(m.data[0]), float(m.data[1]))
            self.arm_goal_t = time.monotonic()
            self.arm_hold = None

    # ───────── 시뮬레이션 1 스텝 ─────────
    def step(self):
        now = time.monotonic()
        dt = min(now - self.last_t, 0.1)
        self.last_t = now

        fresh = self.cmd_t is not None and now - self.cmd_t < self.cmd_timeout
        if self.opencr_down or not self.motor_enable or not fresh:
            v_ref = w_ref = 0.0
        else:
            v_ref = _clip(self.cmd_v, -self.max_v, self.max_v)
            w_ref = _clip(self.cmd_w, -self.max_w, self.max_w)
        self.v += _clip(v_ref - self.v, -self.max_acc_v * dt, self.max_acc_v * dt)
        self.w += _clip(w_ref - self.w, -self.max_acc_w * dt, self.max_acc_w * dt)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt))
        self.encoder_yaw = self.yaw
        wl = (self.v - self.w * self.wheel_sep / 2) / self.wheel_radius
        wr = (self.v + self.w * self.wheel_sep / 2) / self.wheel_radius
        self.wheel_vel = [wl, wr]
        self.wheel_pos = [self.wheel_pos[0] + wl * dt, self.wheel_pos[1] + wr * dt]

        arm_fresh = self.arm_goal is not None and now - self.arm_goal_t < self.arm_cmd_timeout
        if self.opencr_down or not self.motor_enable or not arm_fresh:
            if self.arm_hold is None:
                self.arm_hold = (self.pan, self.tilt)
            goal = self.arm_hold
        else:
            goal = self.arm_goal
        goal = (_clip(goal[0], -self.yaw_limit, self.yaw_limit), _clip(goal[1], self.pitch_min, self.pitch_max))
        mv = self.arm_dps * dt
        self.pan += _clip(goal[0] - self.pan, -mv, mv)
        self.tilt += _clip(goal[1] - self.tilt, -mv, mv)

        self.pub_truth.publish(Float32MultiArray(data=[
            float(self.x), float(self.y), float(self.yaw), float(self.pan), float(self.tilt)]))
        if not self.opencr_down:
            self.publish_state()
        if self.health_on:
            self.publish_health()

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()
        imu_yaw = 0.0 if self.imu_zero else self.yaw
        if self.imu_on:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'imu_link'
            imu.orientation.z, imu.orientation.w = _yaw_quat(imu_yaw)
            if self.imu_invalid:
                imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.z = self.w
            self.pub_imu.publish(imu)
            self.pub_imu_deg.publish(Float32(data=float(math.degrees(imu_yaw))))
        if self.joint_on:
            js = JointState()
            js.header.stamp = stamp
            js.name = ['wheel_left_joint', 'wheel_right_joint', 'arm_yaw_joint', 'arm_pitch_joint']
            js.position = [self.wheel_pos[0], self.wheel_pos[1], math.radians(self.pan), math.radians(self.tilt)]
            js.velocity = [self.wheel_vel[0], self.wheel_vel[1], 0.0, 0.0]
            self.pub_js.publish(js)
        if self.odom_on:
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = 'odom'
            od.child_frame_id = 'base_link'
            od.twist.twist.linear.x = self.v
            od.twist.twist.angular.z = self.w
            od.pose.pose.orientation.z, od.pose.pose.orientation.w = _yaw_quat(self.encoder_yaw)
            self.pub_odom.publish(od)
            self.pub_odom_deg.publish(Float32(data=float(math.degrees(self.encoder_yaw))))

    def publish_health(self):
        level = DiagnosticStatus.ERROR if self.health_error else DiagnosticStatus.OK
        for name, pub in self.pub_health.items():
            if self.opencr_down and name != 'mcu':
                continue
            st = DiagnosticStatus()
            st.name = f'fake_control/{name}'
            st.hardware_id = 'fake'
            if name == 'mcu' and self.opencr_down:
                st.level = DiagnosticStatus.ERROR
                st.message = 'opencr disconnected'
            else:
                st.level = level
                st.message = 'error (sim)' if self.health_error else 'ok'
            pub.publish(st)

    # ───────── 키보드 ─────────
    def poll_keys(self):
        for k in self.kb.keys():
            if k == 'i':
                self.imu_on = not self.imu_on; self.log(f'IMU 발행 {"ON" if self.imu_on else "OFF"}')
            elif k == 'v':
                self.imu_invalid = not self.imu_invalid; self.log(f'IMU 무효 표시 {"ON" if self.imu_invalid else "OFF"}')
            elif k == 'y':
                self.imu_zero = not self.imu_zero; self.log(f'IMU yaw 0 고정 {"ON" if self.imu_zero else "OFF"}')
            elif k == 'j':
                self.joint_on = not self.joint_on; self.log(f'joint_states 발행 {"ON" if self.joint_on else "OFF"}')
            elif k == 'o':
                self.odom_on = not self.odom_on; self.log(f'odom 발행 {"ON" if self.odom_on else "OFF"}')
            elif k == 'c':
                self.opencr_down = not self.opencr_down; self.log(f'OpenCR 끊김 {"ON" if self.opencr_down else "OFF"}')
            elif k == 'm':
                self.motor_enable = not self.motor_enable; self.log(f'모터 {"ON" if self.motor_enable else "OFF"}')
            elif k == 'h':
                if DiagnosticStatus is None:
                    self.log('diagnostic_msgs 없음: health 발행 불가')
                else:
                    self.health_on = not self.health_on; self.log(f'health 발행 {"ON" if self.health_on else "OFF"}')
            elif k == 'e':
                self.health_error = not self.health_error; self.log(f'health 레벨 {"ERROR" if self.health_error else "OK"}')
            elif k == 'r':
                self.x = self.y = self.yaw = self.v = self.w = 0.0
                self.log('로봇 자세 리셋 (0, 0, 0°)')

    def log(self, text):
        self.events.append(f'{time.strftime("%H:%M:%S")}  {text}')
        self.events = self.events[-6:]

    def draw(self):
        if not self.kb.enabled:
            return
        now = time.monotonic()
        cmd_age = f'{now - self.cmd_t:5.2f}s' if self.cmd_t else '  -  '
        fresh = self.cmd_t is not None and now - self.cmd_t < self.cmd_timeout
        lines = [
            '\033[H\033[J' + f"{C['b']}{C['c']}━━━━━━━━━━━━━━━━  FAKE CONTROL  ━━━━━━━━━━━━━━━━{C['0']}",
            f" 로봇   x {self.x:+6.2f} m   y {self.y:+6.2f} m   yaw {math.degrees(self.yaw):+7.1f}°",
            f" 속도   v {self.v:+5.2f} m/s   ω {self.w:+5.2f} rad/s",
            f" 팔     pan {self.pan:+6.1f}°   tilt {self.tilt:+6.1f}°   "
            f"목표 {('%+.1f°, %+.1f°' % self.arm_goal) if self.arm_goal else '-'}",
            f" 명령   cmd_vel 수신 {self.n_cmd}회, 마지막 {cmd_age} 전  "
            f"{onoff(fresh, 'FRESH', 'TIMEOUT→정지')}",
            '',
            f"{C['b']} 발행 상태{C['0']}",
            f"  [i] IMU 발행        {onoff(self.imu_on)}      [v] IMU 무효표시   {onoff(self.imu_invalid, bad_when_on=True)}",
            f"  [y] IMU yaw 0 고정  {onoff(self.imu_zero, bad_when_on=True)}     [j] joint_states   {onoff(self.joint_on)}",
            f"  [o] odom 발행       {onoff(self.odom_on)}      [c] OpenCR 끊김    {onoff(self.opencr_down, bad_when_on=True)}",
            f"  [m] 모터 enable     {onoff(self.motor_enable)}      [h] health 발행    {onoff(self.health_on)}"
            f"   [e] health ERROR {onoff(self.health_error, bad_when_on=True)}",
            f"  [r] 로봇 자세 리셋   Ctrl+C 종료",
            '',
            f"{C['b']} 최근 이벤트{C['0']}",
        ] + [f"  {C['d']}{e}{C['0']}" for e in self.events]
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main():
    rclpy.init()
    with Keyboard() as kb:
        node = FakeControl(kb)
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
