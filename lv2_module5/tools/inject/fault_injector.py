#!/usr/bin/env python3
"""fault_injector.py — 실제 로봇용 장애 주입기 (tools/sim 의 fake 키를 실제 시스템에서)

진짜 노드(perception_master, control_master)와 planning_master 사이에 끼어서
평소에는 받은 메시지를 그대로 넘기고, 키를 누르면 끊거나 바꿔서 넘긴다.
로봇은 항상 진짜 카메라로 진짜 퍽을 보고 따라간다. 퍽 이동은 손으로 한다.

  perception_master ── /detection ───────────────┐
  control_master ───── /control/imu ──────────────┤          /inject/detection, /inject/imu,
                       /control/odom_yaw_deg ─────┼─> 주입기 ─> /inject/odom_yaw_deg, /inject/joint_states,
                       /control/joint_states ─────┤          /inject/health/<센서>  ─> planning_master
                       health 5개 ────────────────┘
  planning_master ──── /inject/planning_cmd_vel ──> 주입기 ─> /planning/cmd_vel ─> control_master
  (/planning/arm_command 는 그대로 직접 연결)

  IMU 끊김(i)·모터 OFF(m)는 control_master 파라미터(imu_ignore, motor_enable)를 실행 중에 바꾼다.
  → control 안에서도 진짜로 IMU 를 안 쓰고(엔코더로만 yaw 이어감), 진짜로 모터에 0 을 보낸다.

실행 (터미널 3개, 모두 ROS 2 + 워크스페이스 source 후, lv2_module5 폴더에서)
  1) ros2 launch launch/bringup.launch.py planning:=false
  2) ros2 run planning planning_master --ros-args \\
       --params-file config/planning.yaml --params-file tools/inject/planning_inject.yaml
  3) python3 tools/inject/fault_injector.py
  주입기를 끄면 planning 명령이 control 로 안 가므로 control 이 명령 타임아웃(0.3 s)으로 감속 정지한다.
  끌 때 control 의 imu_ignore=false, motor_enable=true 로 되돌린다.

키 (시뮬레이션 fake_perception / fake_control 과 같은 글자, 겹치는 것만 바꿈)
  [인지 /detection]
    o  가림 켜기/끄기 (z=0 계속)          t  2초 가림          g  퍽 제거 (= o, 실제로는 같은 효과)
    p  /detection 침묵 (토픽 끊김, camera health 도 STALE)
    n  NaN 주입                           f  노이즈 (±1 cm·0.01 + 2 % 미검출)
  [제어 /control/*]
    i  IMU 끊김 (control imu_ignore + planning 쪽 IMU 끊김)
    v  IMU 무효 표시 (orientation_covariance[0] = -1)       y  IMU yaw 0 고정
    j  joint_states 끊김                  u  odom_yaw 끊김 (시뮬의 o)
    c  OpenCR 끊김 (planning 이 보기에: 상태·IMU 끊김, mcu ERROR, 명령 전달 중단 → control 감속 정지)
       ※ 보드 자체 watchdog 정지는 USB 를 직접 뽑아서 시험
    m  모터 OFF (control motor_enable=false, 모터 health ERROR)
    h  health 전부 침묵                   e  control health 전부 ERROR
  [명령 /planning/cmd_vel]
    w/s  수동 전진/후진     a/d  수동 반시계/시계 회전     k  수동 정지(planning 무시, 0 유지)
    space  수동 해제 (planning 명령으로 복귀)              + / -  수동 속도 단계 올리기/내리기
    x  명령 끊김 (전달 중단 → control 타임아웃 감속 정지)  z  명령 NaN 주입
  [공통]
    0  모든 주입 해제     q  끝내기 (Ctrl+C 도 같음)

기록
  키를 누를 때마다 /inject/event (std_msgs/String) 발행 + CSV 한 줄
  (results/logs/inject_날짜시간.csv: wall_time, ros_time, key, event)
  bag 에 /inject/event 와 /tracking_status 를 같이 넣으면 "주입 시각 → 상태 전이 시각" 을 바로 잴 수 있다.
"""
import copy
import csv
import math
import os
import random
import select
import sys
import termios
import time
import tty

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, String

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.normpath(os.path.join(HERE, '..', '..', 'results', 'logs'))

# perception_master 발행 QoS 와 같게 (best-effort, depth 1)
DETECTION_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

HEALTH_IN = {   # 센서 이름 → 진짜 진단 토픽 (planning.yaml 기본값과 같음)
    'camera': '/perception/camera_health',
    'imu': '/control/imu_health',
    'arm_motor': '/control/arm_motor_health',
    'wheel_motor': '/control/wheel_motor_health',
    'mcu': '/control/opencr',
}
CONTROL_HEALTH = ('imu', 'arm_motor', 'wheel_motor', 'mcu')

MAX_V, MAX_W = 0.26, 1.82        # control.yaml 의 max_v, max_w
SPEED_LEVELS = 10                # 수동 속도 단계: v = MAX_V × 단계/10, ω = MAX_W × 단계/10

C = {'g': '\033[32m', 'r': '\033[31m', 'y': '\033[33m', 'c': '\033[36m', 'b': '\033[1m', 'd': '\033[2m', '0': '\033[0m'}


class Keyboard:
    """터미널을 cbreak 모드로 바꿔 키 하나씩 non-blocking 으로 읽는다 (tools/sim 과 같음)."""

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


def onoff(on, bad_when_on=False):
    color = C['r'] if on == bad_when_on else C['g']
    return f"{color}{'ON ' if on else 'OFF'}{C['0']}"


class FaultInjector(Node):
    def __init__(self, kb):
        super().__init__('fault_injector')
        self.kb = kb

        # ---------- 주입 상태 (전부 False = 평소와 같음) ----------
        self.occluded = False        # o / g
        self.occlude_until = 0.0     # t
        self.det_silent = False      # p
        self.det_nan = False         # n
        self.det_noise = False       # f
        self.imu_off = False         # i
        self.imu_invalid = False     # v
        self.imu_zero = False        # y
        self.joint_off = False       # j
        self.odom_off = False        # u
        self.opencr_down = False     # c
        self.motor_off = False       # m
        self.health_silent = False   # h
        self.health_error = False    # e
        self.cmd_cut = False         # x
        self.cmd_nan = False         # z
        self.manual = None           # None: planning 명령 전달, (v, ω): 수동 명령
        self.manual_dir = None       # 'w' 's' 'a' 'd' 'k' (속도 단계 바뀌면 다시 계산)
        self.speed_level = 4         # v 0.104 m/s, ω 0.728 rad/s

        # ---------- 화면 표시용 ----------
        self.status = '-'            # 마지막 /tracking_status
        self.planning_cmd = (0.0, 0.0)
        self.n_det = 0
        self.events = []
        self.param_note = ''

        # ---------- 기록 ----------
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = os.path.join(LOG_DIR, time.strftime('inject_%Y%m%d_%H%M%S.csv'))
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_csv = csv.writer(self.log_file)
        self.log_csv.writerow(['wall_time', 'ros_time', 'key', 'event'])

        # ---------- 토픽 ----------
        self.pub_det = self.create_publisher(PointStamped, '/inject/detection', DETECTION_QOS)
        self.pub_imu = self.create_publisher(Imu, '/inject/imu', 10)
        self.pub_odom_yaw = self.create_publisher(Float32, '/inject/odom_yaw_deg', 10)
        self.pub_js = self.create_publisher(JointState, '/inject/joint_states', 10)
        self.pub_health = {s: self.create_publisher(DiagnosticStatus, f'/inject/health/{s}', 10) for s in HEALTH_IN}
        self.pub_cmd = self.create_publisher(Twist, '/planning/cmd_vel', 10)
        self.pub_event = self.create_publisher(String, '/inject/event', 10)

        self.create_subscription(PointStamped, '/detection', self.det_cb, DETECTION_QOS)
        self.create_subscription(Imu, '/control/imu', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, '/control/odom_yaw_deg', self.odom_yaw_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, '/control/joint_states', self.js_cb, qos_profile_sensor_data)
        for s, topic in HEALTH_IN.items():
            self.create_subscription(
                DiagnosticStatus, topic, lambda m, s=s: self.health_cb(s, m), qos_profile_sensor_data)
        self.create_subscription(Twist, '/inject/planning_cmd_vel', self.cmd_cb, 10)
        self.create_subscription(String, '/tracking_status', self.status_cb, 10)

        # control_master 파라미터 (imu_ignore, motor_enable) 를 실행 중에 바꾸는 클라이언트
        self.control_params = AsyncParameterClient(self, 'control_master')

        self.create_timer(0.05, self.manual_tick)   # 수동 명령 20 Hz (control 타임아웃 0.3 s 보다 충분히 짧게)
        self.create_timer(0.05, self.poll_keys)
        self.create_timer(0.2, self.draw)
        self.event('-', f'시작, 기록 {self.log_path}')

    # ───────── 인지 ─────────
    def det_cb(self, msg):
        if self.det_silent:
            return
        out = copy.deepcopy(msg)   # header.stamp 는 그대로 (planning 이 촬영 시각 팔 자세를 찾는 데 씀)
        p = out.point
        if self.occluded or time.monotonic() < self.occlude_until:
            p.x = p.y = p.z = 0.0
        elif self.det_noise and p.z > 0.0:
            if random.random() < 0.02:
                p.x = p.y = p.z = 0.0
            else:
                p.x += random.gauss(0, 0.01)
                p.y += random.gauss(0, 0.01)
                p.z += random.gauss(0, 0.01)
        if self.det_nan:
            p.x = p.y = p.z = float('nan')
        self.pub_det.publish(out)
        self.n_det += 1

    # ───────── 제어 상태 ─────────
    def imu_cb(self, msg):
        if self.imu_off or self.opencr_down:
            return
        out = copy.deepcopy(msg)
        if self.imu_zero:
            out.orientation.z, out.orientation.w = 0.0, 1.0
        if self.imu_invalid:
            out.orientation_covariance[0] = -1.0
        self.pub_imu.publish(out)

    def odom_yaw_cb(self, msg):
        if not (self.odom_off or self.opencr_down):
            self.pub_odom_yaw.publish(msg)

    def js_cb(self, msg):
        if not (self.joint_off or self.opencr_down):
            self.pub_js.publish(msg)

    def health_cb(self, sensor, msg):
        if self.health_silent:
            return
        out = copy.deepcopy(msg)
        if sensor in CONTROL_HEALTH and self.opencr_down:
            # control_master 와 같게: OpenCR 끊기면 mcu=ERROR, 나머지는 STALE (상태 모름)
            self.set_level(out, DiagnosticStatus.ERROR if sensor == 'mcu' else DiagnosticStatus.STALE,
                           'no packet (inject)' if sensor == 'mcu' else 'opencr down (inject)')
        elif sensor in CONTROL_HEALTH and self.health_error:
            self.set_level(out, DiagnosticStatus.ERROR, 'error (inject)')
        elif sensor in ('arm_motor', 'wheel_motor') and self.motor_off:
            self.set_level(out, DiagnosticStatus.ERROR, 'motor disabled (inject)')
        elif sensor == 'imu' and self.imu_off:
            self.set_level(out, DiagnosticStatus.ERROR, 'ignored (inject)')
        elif sensor == 'camera' and self.det_silent:
            self.set_level(out, DiagnosticStatus.STALE, 'stream stopped (inject)')
        self.pub_health[sensor].publish(out)

    @staticmethod
    def set_level(msg, level, text):
        msg.level = level
        msg.message = text

    # ───────── 명령 ─────────
    def cmd_cb(self, msg):
        self.planning_cmd = (msg.linear.x, msg.angular.z)
        if self.manual is None:
            self.send_cmd(msg.linear.x, msg.angular.z)

    def manual_tick(self):
        if self.manual is not None:
            self.send_cmd(*self.manual)

    def send_cmd(self, v, w):
        if self.cmd_cut or self.opencr_down:
            return
        out = Twist()
        if self.cmd_nan:
            out.linear.x = out.angular.z = float('nan')
        else:
            out.linear.x, out.angular.z = float(v), float(w)
        self.pub_cmd.publish(out)

    def manual_speed(self):
        f = self.speed_level / SPEED_LEVELS
        return MAX_V * f, MAX_W * f

    def set_manual(self, key):
        v, w = self.manual_speed()
        self.manual_dir = key
        self.manual = {'w': (v, 0.0), 's': (-v, 0.0), 'a': (0.0, w), 'd': (0.0, -w), 'k': (0.0, 0.0)}[key]

    def status_cb(self, msg):
        self.status = msg.data

    # ───────── control_master 파라미터 ─────────
    def set_control_param(self, name, value):
        if not self.control_params.services_are_ready():
            self.param_note = f'{C["r"]}control_master 파라미터 서비스 없음 → {name} 안 바뀜{C["0"]}'
            self.event('!', f'{name}={value} 실패: control_master 파라미터 서비스 없음')
            return
        fut = self.control_params.set_parameters([Parameter(name, value=value)])
        fut.add_done_callback(lambda f: self.param_done(f, name, value))

    def param_done(self, fut, name, value):
        try:
            res = fut.result().results[0]
        except Exception as e:   # 서비스 응답 실패
            self.param_note = f'{C["r"]}{name}={value} 실패: {e}{C["0"]}'
            self.event('!', f'{name}={value} 실패: {e}')
            return
        if res.successful:
            self.param_note = f'control_master {name}={value} 적용'
            self.event('!', f'control_master {name}={value} 적용')
        else:
            self.param_note = f'{C["r"]}{name}={value} 거부: {res.reason}{C["0"]}'
            self.event('!', f'{name}={value} 거부: {res.reason}')

    # ───────── 키보드 ─────────
    def toggle(self, attr, key, text):
        setattr(self, attr, not getattr(self, attr))
        on = getattr(self, attr)
        self.event(key, f'{text} {"ON" if on else "OFF"}')
        return on

    def poll_keys(self):
        for k in self.kb.keys():
            if k in 'og':
                self.toggle('occluded', k, '가림')
            elif k == 't':
                self.occlude_until = time.monotonic() + 2.0
                self.event(k, '2초 가림')
            elif k == 'p':
                self.toggle('det_silent', k, '/detection 침묵')
            elif k == 'n':
                self.toggle('det_nan', k, '/detection NaN')
            elif k == 'f':
                self.toggle('det_noise', k, '/detection 노이즈')
            elif k == 'i':
                on = self.toggle('imu_off', k, 'IMU 끊김')
                self.set_control_param('imu_ignore', on)
            elif k == 'v':
                self.toggle('imu_invalid', k, 'IMU 무효 표시')
            elif k == 'y':
                self.toggle('imu_zero', k, 'IMU yaw 0 고정')
            elif k == 'j':
                self.toggle('joint_off', k, 'joint_states 끊김')
            elif k == 'u':
                self.toggle('odom_off', k, 'odom_yaw 끊김')
            elif k == 'c':
                self.toggle('opencr_down', k, 'OpenCR 끊김')
            elif k == 'm':
                on = self.toggle('motor_off', k, '모터 OFF')
                self.set_control_param('motor_enable', not on)
            elif k == 'h':
                self.toggle('health_silent', k, 'health 침묵')
            elif k == 'e':
                self.toggle('health_error', k, 'health ERROR')
            elif k in 'wsadk':
                self.set_manual(k)
                v, w = self.manual
                self.event(k, f'수동 명령 v={v:+.3f} m/s, ω={w:+.3f} rad/s')
            elif k == ' ':
                if self.manual is not None:
                    self.manual = self.manual_dir = None
                    self.event('space', '수동 해제 → planning 명령')
            elif k in '+=-_':
                step = 1 if k in '+=' else -1
                self.speed_level = max(1, min(SPEED_LEVELS, self.speed_level + step))
                if self.manual_dir is not None:
                    self.set_manual(self.manual_dir)
                v, w = self.manual_speed()
                self.event(k, f'수동 속도 {self.speed_level}단계 (v {v:.3f} m/s, ω {w:.3f} rad/s)')
            elif k == 'x':
                self.toggle('cmd_cut', k, '명령 끊김')
            elif k == 'z':
                self.toggle('cmd_nan', k, '명령 NaN')
            elif k == '0':
                self.clear_all()
            elif k == 'q':
                raise KeyboardInterrupt

    def clear_all(self):
        if self.imu_off:
            self.set_control_param('imu_ignore', False)
        if self.motor_off:
            self.set_control_param('motor_enable', True)
        for attr in ('occluded', 'det_silent', 'det_nan', 'det_noise', 'imu_off', 'imu_invalid', 'imu_zero',
                     'joint_off', 'odom_off', 'opencr_down', 'motor_off', 'health_silent', 'health_error',
                     'cmd_cut', 'cmd_nan'):
            setattr(self, attr, False)
        self.occlude_until = 0.0
        self.manual = self.manual_dir = None
        self.event('0', '모든 주입 해제')

    def restore_control(self):
        """끝낼 때 control_master 를 평소 설정으로 되돌린다 (응답을 최대 1초 기다림)."""
        if not self.control_params.services_are_ready():
            return
        fut = self.control_params.set_parameters([
            Parameter('imu_ignore', value=False), Parameter('motor_enable', value=True)])
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    # ───────── 기록 · 화면 ─────────
    def event(self, key, text):
        ros_t = self.get_clock().now().nanoseconds * 1e-9
        self.log_csv.writerow([f'{time.time():.3f}', f'{ros_t:.3f}', key, text])
        self.log_file.flush()
        self.pub_event.publish(String(data=f'{key} {text}'))
        self.events.append(f'{time.strftime("%H:%M:%S")}  [{key}] {text}')
        self.events = self.events[-8:]

    def draw(self):
        if not self.kb.enabled:
            return
        occl_left = self.occlude_until - time.monotonic()
        v, w = self.manual_speed()
        pv, pw = self.planning_cmd
        mode = (f"{C['y']}수동 v={self.manual[0]:+.3f} ω={self.manual[1]:+.3f}{C['0']}"
                if self.manual is not None else f"{C['g']}planning 전달{C['0']}")
        lines = [
            '\033[H\033[J' + f"{C['b']}{C['c']}━━━━━━━━━━━━━━━━  FAULT INJECTOR (실제 로봇)  ━━━━━━━━━━━━━━━━{C['0']}",
            f" 상태   {self.status}",
            f" 명령   {mode}   planning v={pv:+.3f} ω={pw:+.3f}   수동 속도 {self.speed_level}단계 "
            f"(v {v:.3f}, ω {w:.3f})",
            f" 검출   전달 {self.n_det}회" + (f"   {C['r']}2초 가림 {occl_left:.1f}s{C['0']}" if occl_left > 0 else ''),
            '',
            f"{C['b']} 인지{C['0']}   [o/g] 가림 {onoff(self.occluded, True)}  [t] 2초 가림  "
            f"[p] 침묵 {onoff(self.det_silent, True)}  [n] NaN {onoff(self.det_nan, True)}  "
            f"[f] 노이즈 {onoff(self.det_noise, True)}",
            f"{C['b']} 제어{C['0']}   [i] IMU 끊김 {onoff(self.imu_off, True)}  [v] IMU 무효 {onoff(self.imu_invalid, True)}  "
            f"[y] yaw 0 {onoff(self.imu_zero, True)}  [j] joint 끊김 {onoff(self.joint_off, True)}  "
            f"[u] odom 끊김 {onoff(self.odom_off, True)}",
            f"         [c] OpenCR 끊김 {onoff(self.opencr_down, True)}  [m] 모터 OFF {onoff(self.motor_off, True)}  "
            f"[h] health 침묵 {onoff(self.health_silent, True)}  [e] health ERROR {onoff(self.health_error, True)}",
            f"{C['b']} 명령{C['0']}   [w/s] 전진/후진  [a/d] 반시계/시계  [k] 수동 정지  [space] 수동 해제  "
            f"[+/-] 속도  [x] 끊김 {onoff(self.cmd_cut, True)}  [z] NaN {onoff(self.cmd_nan, True)}",
            f"         [0] 모든 주입 해제   [q] 끝내기",
            f" {self.param_note}",
            '',
            f"{C['b']} 최근 이벤트{C['0']}  (기록 {self.log_path})",
        ] + [f"  {C['d']}{e}{C['0']}" for e in self.events]
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()

    def close(self):
        self.event('q', '종료')
        self.log_file.close()


def main():
    # rclpy 의 Ctrl+C 처리기를 끈다: 끝낼 때 control 파라미터를 되돌리려면 ROS 연결이 살아 있어야 함
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    with Keyboard() as kb:
        node = FaultInjector(kb)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.restore_control()
            node.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
