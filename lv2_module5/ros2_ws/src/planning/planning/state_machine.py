"""state_machine.py — health 결과를 상태 전이에 반영 + /tracking_status 발행 (이슈 #6)

기존 상태 전이(검출/타임아웃, planning_master.update_state)는 그대로 두고,
그 뒤에 health 규칙을 덧씌운다.
  motor_fault  : 모든 상태 → 'lost'  (정지)
  camera_stale : tracking/searching → 'lost'  (정지)
  imu_fallback : 상태 유지 (정지하지 않음). 사유만 기록. 엔코더 heading 전환은 TODO(선택 기능)
  ok 인데 state == 'lost' : 사유 'target_lost' (센서는 정상, 물체를 놓친 것)
'lost' 는 calc_cmd_vel 에서 속도 0 이므로 fail-safe 정지로 그대로 재사용한다.

health.enabled: false 로 끄면 상태를 건드리지 않고 통과시킨다
(다른 팀원의 health 발행이 아직 없을 때, 시작부터 LOST 로 고정되는 것을 피하기 위한 스위치).

/tracking_status 형식(임시, 팀 확정 필요): std_msgs/String  "state=lost reason=camera_stale"
"""
from std_msgs.msg import String

try:
    from .health_monitor import HealthMonitor   # 패키지로 import 될 때 (planning.state_machine)
except ImportError:
    from health_monitor import HealthMonitor    # 같은 폴더에서 단독 실행할 때


class HealthGate:
    def __init__(self, node, monitor=None):
        self._node = node
        self._enabled = bool(self._param('health.enabled', True))
        self.monitor = monitor or HealthMonitor(node)
        topic = self._param('status_topic', '/tracking_status')
        self._pub = node.create_publisher(String, topic, 10)
        self._last_msg = None
        self.health = None                      # 가장 최근 판정 (HealthResult)

    def _param(self, name, default):
        if not self._node.has_parameter(name):
            self._node.declare_parameter(name, default)
        return self._node.get_parameter(name).value

    def decide(self, state):
        """현재 state 를 받아 health 를 반영한 새 state 를 돌려주고, /tracking_status 를 발행."""
        if not self._enabled:
            self._publish(state, 'health_disabled')
            return state

        h = self.monitor.evaluate()
        self.health = h
        new_state, reason = state, h.reason

        if h.reason == 'motor_fault':
            new_state = 'lost'
        elif h.reason == 'camera_stale' and state in ('tracking', 'searching'):
            new_state = 'lost'
        elif h.reason == 'ok' and state == 'lost':
            reason = 'target_lost'

        self._publish(new_state, reason)
        return new_state

    def _publish(self, state, reason):
        msg = f'state={state} reason={reason}'
        self._pub.publish(String(data=msg))
        if msg != self._last_msg:               # 바뀔 때만 로그
            self._node.get_logger().info(msg)
            self._last_msg = msg
