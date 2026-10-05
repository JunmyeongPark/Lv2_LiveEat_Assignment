"""health_monitor.py — 인지·제어 health 5개 구독 → 종합 판정 (이슈 #6, FDIR의 Recovery)

판정 우선순위: mcu_fault > motor_fault > camera_stale > imu_fallback > ok
- 센서가 "비정상"이 되는 경우 두 가지
    (1) 받은 메시지의 level 이 ERROR(2) 또는 STALE(3)
    (2) 메시지가 아예 안 옴 (마지막 수신 후 stale_timeout_s 초 경과)
- 복귀: OK 메시지가 recover_ok_count 번 연속 와야 "정상"으로 인정 (깜빡임 방지)
- 시작 직후에는 OK를 아직 못 받았으므로 "비정상"으로 시작 (보수적)
"""
from dataclasses import dataclass
from functools import partial

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.qos import qos_profile_sensor_data   # best-effort: 발행자가 reliable이든 아니든 연결됨

OK, WARN, ERROR, STALE = 0, 1, 2, 3             # DiagnosticStatus.level 상수 값
SENSORS = ('camera', 'imu', 'arm_motor', 'wheel_motor', 'mcu')
DEFAULT_TOPICS = {                              # 기본값. 실제 값은 planning.yaml 에서 덮어씀
    'camera': '/perception/camera_health',
    'imu': '/control/imu_health',               # 이슈 원문의 /control/imu_sensor 는 오타 — 팀장님 확인 완료(/control/imu_health)
    'arm_motor': '/control/arm_motor_health',
    'wheel_motor': '/control/wheel_motor_health',
    'mcu': '/control/opencr',                   # OpenCR 보드 자체의 health (모터·IMU 를 잇는 MCU)
}


def _level_to_int(level):
    # ROS 2 Humble 의 rclpy 는 byte 필드를 bytes(b'\x00')로 넘겨줌 → 정수로 변환
    if isinstance(level, (bytes, bytearray)):
        return int.from_bytes(level, 'little')
    return int(level)


@dataclass
class HealthResult:
    camera_ok: bool
    imu_ok: bool
    motor_ok: bool
    mcu_ok: bool
    reason: str          # 'ok' | 'imu_fallback' | 'camera_stale' | 'motor_fault' | 'mcu_fault'

    @property
    def use_encoder_heading(self):
        # 메인 노드는 이 판정과 실제 heading 입력 신선도를 함께 검사한다.
        return not self.imu_ok


class HealthMonitor:
    def __init__(self, node):
        self._node = node
        self._timeout = float(self._param('health.stale_timeout_s', 0.5))
        self._recover_n = int(self._param('health.recover_ok_count', 3))
        self._warn_ok = bool(self._param('health.warn_is_ok', True))
        if self._timeout <= 0 or self._recover_n < 1:
            raise ValueError('health timeout must be positive and recover_ok_count >= 1')

        self._last_rx = {s: None for s in SENSORS}   # 마지막 수신 시각(초)
        self._ok_cnt = {s: 0 for s in SENSORS}       # 연속 OK 메시지 수
        self._subscriptions = []
        for s in SENSORS:
            topic = self._param(f'health.topics.{s}', DEFAULT_TOPICS[s])
            self._subscriptions.append(node.create_subscription(
                DiagnosticStatus, topic, partial(self._cb, s), qos_profile_sensor_data))

    def _param(self, name, default):
        if not self._node.has_parameter(name):
            self._node.declare_parameter(name, default)
        return self._node.get_parameter(name).value

    def _now(self):
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _cb(self, sensor, msg):
        level = _level_to_int(msg.level)
        now = self._now()
        last = self._last_rx[sensor]
        # evaluate 사이에 끊겼다가 재개된 경우도 연속 정상 횟수를 새로 센다.
        if last is None or not 0 <= now - last <= self._timeout:
            self._ok_cnt[sensor] = 0
        self._last_rx[sensor] = now
        good = level == OK or (level == WARN and self._warn_ok)
        self._ok_cnt[sensor] = min(self._ok_cnt[sensor] + 1, self._recover_n) if good else 0

    def _sensor_ok(self, sensor, now):
        last = self._last_rx[sensor]
        if last is None or not 0 <= now - last <= self._timeout:
            self._ok_cnt[sensor] = 0
            return False
        return self._ok_cnt[sensor] >= self._recover_n

    def evaluate(self):
        now = self._now()
        ok = {s: self._sensor_ok(s, now) for s in SENSORS}
        motor_ok = ok['arm_motor'] and ok['wheel_motor']
        if not ok['mcu']:
            reason = 'mcu_fault'            # OpenCR 가 죽으면 모터·IMU 값도 믿을 수 없으므로 최우선
        elif not motor_ok:
            reason = 'motor_fault'
        elif not ok['camera']:
            reason = 'camera_stale'
        elif not ok['imu']:
            reason = 'imu_fallback'
        else:
            reason = 'ok'
        return HealthResult(ok['camera'], ok['imu'], motor_ok, ok['mcu'], reason)
