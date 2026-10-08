#!/usr/bin/env python3
"""mock_inputs.py — 발제 평가 4 모의 입력 5종을 끊김 없이 순서대로 /detection 에 발행 (모터 OFF 에서)

  구간          보내는 값 (20 Hz)                 기대 (코드 기준)
  IN1  10 s     x=0,    z=d   (검출, 정면)          TRACKING, 불필요한 회전 없음 (cmd_w ≈ 0)
  IN2  10 s     x=+0.4, z=d   (오른쪽)            오른쪽 오차를 줄이는 명령 = cmd_w < 0 (시계)
  IN3  10 s     x=-0.4, z=d   (왼쪽)              반대 부호 = cmd_w > 0
  (준비) 3 s    x=+0.4, z=d                       IN4 직전 TRACKING 만들기
  IN4  6 s      x=+0.4, z=0   (미검출)            첫 프레임부터 정지(cmd 0), 3 프레임 뒤 SEARCHING
  (준비) 3 s    x=+0.4, z=d                       IN5 직전 TRACKING 만들기
  IN5  5 s      발행 중단                          0.5 s 뒤 FAULT reason=detection_timeout, 정지

  z 는 우리 규약의 depth[m] (발제의 면적비 아님, 0 = 미검출). d 기본 0.4 m (= tgt_dis), 유효 0.2~3.0 m.
  팔을 정면 [0, 0] 에 둔 뒤 모터를 꺼야 x=0 이 "몸통 정면" 이 된다 (팔이 돌아가 있으면 그만큼 치우침).
  구간이 바뀔 때마다 /inject/event 로 'IN1 start' 같은 문자열을 보내므로 bag 에서 시각을 바로 맞출 수 있다.

실행 (perception 은 끄고 — /detection 이 섞이지 않게)
  python3 tools/inject/mock_inputs.py [depth_m, 기본 0.4]
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

RATE_HZ = 20.0


def make_sequence(d):
    """(이름, 길이 s, x, z) — z=None 이면 발행 안 함. d = 검출 구간의 depth [m]"""
    return [
        ('IN1 x=0', 10.0, 0.0, d),
        ('IN2 x=+0.4', 10.0, 0.4, d),
        ('IN3 x=-0.4', 10.0, -0.4, d),
        ('prep tracking', 3.0, 0.4, d),
        ('IN4 z=0', 6.0, 0.4, 0.0),
        ('prep tracking', 3.0, 0.4, d),
        ('IN5 stop publishing', 5.0, 0.0, None),
    ]


def main():
    # 인자 1개: 검출 구간 depth [m]. 기본 0.4 = planning tgt_dis (팔이 정면 [0,0] 이면 앞뒤 명령 ≈ 0)
    depth = float(sys.argv[1]) if len(sys.argv) > 1 else 0.4
    SEQUENCE = make_sequence(depth)
    rclpy.init()
    node = Node('mock_inputs')
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    pub = node.create_publisher(PointStamped, '/detection', qos)
    ev = node.create_publisher(String, '/inject/event', 10)
    time.sleep(1.0)   # 구독자 연결 대기
    try:
        for name, dur, x, z in SEQUENCE:
            ev.publish(String(data=f'{name} start depth={depth}'))
            node.get_logger().info(f'{name} ({dur:.0f} s)')
            end = time.monotonic() + dur
            while time.monotonic() < end:
                if z is not None:
                    m = PointStamped()
                    m.header.stamp = node.get_clock().now().to_msg()   # 보낸 시각 (촬영 시각 아님)
                    m.header.frame_id = 'mock'
                    m.point.x, m.point.y, m.point.z = x, 0.0, z
                    pub.publish(m)
                time.sleep(1.0 / RATE_HZ)
        ev.publish(String(data='done'))
        node.get_logger().info('끝')
        time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
