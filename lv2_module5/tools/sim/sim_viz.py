#!/usr/bin/env python3
"""sim_viz.py — 시뮬레이션 RViz 시각화

시뮬레이션의 '실제' 위치(로봇·팔·퍽)와 planning 상태를 MarkerArray 로 그린다.
제어·판단에는 관여하지 않는 순수 시각화 노드.

  구독  /sim/truth            [x, y, yaw_rad, pan_deg, tilt_deg]   (fake_control)
        /sim/puck_truth       [x, y, z, 보임]                       (fake_perception)
        /sim/tracking_status  planning 상태 문자열                  (fake_planning)
        /sim/planning/cmd_vel planning 이 낸 속도 명령
  발행  /sim/markers          visualization_msgs/MarkerArray  (frame: odom)
        TF odom → base_link   로봇 실제 자세 (RViz 에서 base_link 를 따라가게 할 때 사용)

그리는 것
  - 와플: 상자(상태별 색) + 진행 방향 화살표 + 지나온 경로
  - 카메라: 시선 화살표 + 화각(FOV) 사각뿔 (바닥에 닿으면 바닥까지만)
  - 퍽: 원기둥 — 초록 = 카메라에 보임, 빨강 = 안 보임
  - 목표 거리 원 (tgt_dis 0.40 m), 상태·reason·속도 텍스트

실행 (ROS 2 환경 source 후)
  python3 lv2_module5/tools/sim/sim_viz.py
  rviz2 -d lv2_module5/tools/sim/sim.rviz
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
except ImportError:  # tf2_ros 가 없으면 TF 없이 마커만 그린다
    TransformBroadcaster = None

STATE_COLORS = {                      # r, g, b
    'IDLE': (0.75, 0.75, 0.75),
    'TRACKING': (0.20, 0.80, 0.30),
    'SEARCHING': (0.95, 0.80, 0.15),
    'LOST': (0.90, 0.25, 0.20),
    'FAULT': (0.80, 0.30, 0.85),
}


def _pt(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _yaw_q(marker, yaw):
    marker.pose.orientation.z = math.sin(yaw / 2.0)
    marker.pose.orientation.w = math.cos(yaw / 2.0)


class SimViz(Node):
    def __init__(self):
        super().__init__('sim_viz')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        pre = str(p('topic_prefix', '/sim')).rstrip('/')
        self.frame = str(p('frame_id', 'odom'))
        self.cam_height = float(p('cam_height', 0.20))      # fake_perception 과 같게
        self.hfov = float(p('hfov_deg', 69.0))
        self.vfov = float(p('vfov_deg', 42.0))
        self.max_depth = float(p('max_depth', 3.0))
        self.tgt_dis = float(p('tgt_dis', 0.40))
        self.pan_forward = float(p('pan_forward_deg', -0.09))
        self.trail_len = int(p('trail_length', 600))

        self.truth = None
        self.puck = None
        self.status = 'waiting for /sim/tracking_status'
        self.cmd = (0.0, 0.0)
        self.trail = []

        self.create_subscription(Float32MultiArray, pre + '/truth', self.truth_cb, 10)
        self.create_subscription(Float32MultiArray, pre + '/puck_truth', self.puck_cb, 10)
        self.create_subscription(String, pre + '/tracking_status', self.status_cb, 10)
        self.create_subscription(Twist, pre + '/planning/cmd_vel', self.cmd_cb, 10)
        self.pub = self.create_publisher(MarkerArray, pre + '/markers', 10)
        self.tf = TransformBroadcaster(self) if TransformBroadcaster else None
        self.create_timer(1.0 / float(p('rate_hz', 20.0)), self.draw)

    # ───────── 구독 ─────────
    def truth_cb(self, m):
        if len(m.data) >= 5:
            self.truth = [float(v) for v in m.data[:5]]
            x, y = self.truth[:2]
            if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.01:
                self.trail.append((x, y))
                self.trail = self.trail[-self.trail_len:]

    def puck_cb(self, m):
        if len(m.data) >= 4:
            self.puck = [float(v) for v in m.data[:4]]

    def status_cb(self, m):
        self.status = m.data

    def cmd_cb(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    # ───────── 그리기 ─────────
    def _marker(self, ns, mid, mtype, rgba):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.pose.orientation.w = 1.0
        return m

    def _state_key(self):
        head = self.status.split()[0] if self.status else ''
        return head.split('|')[0].upper()

    def _cam_dir(self, yaw, pan, tilt, th, ph):
        """카메라 좌표 방향 (1, tanθ, tanφ) → 월드 방향 (Ry(-tilt) 후 Rz(yaw+pan))."""
        f, l, u = 1.0, math.tan(th), math.tan(ph)
        ct, st = math.cos(tilt), math.sin(tilt)
        a, z = ct * f - st * u, st * f + ct * u
        c, s = math.cos(yaw + pan), math.sin(yaw + pan)
        return c * a - s * l, s * a + c * l, z

    def _ray_end(self, ox, oy, oz, d):
        """깊이(전방 성분) max_depth 까지, 바닥(z=0)에 닿으면 거기까지."""
        t = self.max_depth
        if d[2] < -1e-6:
            t = min(t, oz / -d[2])
        return ox + d[0] * t, oy + d[1] * t, oz + d[2] * t

    def draw(self):
        if self.truth is None:
            return
        x, y, yaw, pan_deg, tilt_deg = self.truth
        pan = math.radians(pan_deg - self.pan_forward)
        tilt = math.radians(tilt_deg)
        state = self._state_key()
        rgb = STATE_COLORS.get(state, (0.6, 0.6, 0.9))
        out = MarkerArray()

        # 와플 몸체 (Waffle Pi 약 0.28 × 0.31 × 0.14 m)
        body = self._marker('robot', 0, Marker.CUBE, (*rgb, 0.85))
        body.pose.position = _pt(x, y, 0.07)
        _yaw_q(body, yaw)
        body.scale.x, body.scale.y, body.scale.z = 0.281, 0.306, 0.141
        out.markers.append(body)

        heading = self._marker('robot', 1, Marker.ARROW, (0.2, 0.5, 1.0, 1.0))
        heading.points = [_pt(x, y, 0.15), _pt(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw), 0.15)]
        heading.scale.x, heading.scale.y, heading.scale.z = 0.02, 0.05, 0.06
        out.markers.append(heading)

        trail = self._marker('robot', 2, Marker.LINE_STRIP, (0.4, 0.6, 1.0, 0.6))
        trail.points = [_pt(px, py, 0.005) for px, py in self.trail]
        trail.scale.x = 0.01
        if len(trail.points) >= 2:
            out.markers.append(trail)

        ring = self._marker('robot', 3, Marker.LINE_STRIP, (1.0, 1.0, 1.0, 0.35))
        ring.points = [_pt(x + self.tgt_dis * math.cos(a), y + self.tgt_dis * math.sin(a), 0.003)
                       for a in [i * 2 * math.pi / 48 for i in range(49)]]
        ring.scale.x = 0.006
        out.markers.append(ring)

        # 카메라 시선 + FOV
        oz = self.cam_height
        d = self._cam_dir(yaw, pan, tilt, 0.0, 0.0)
        gaze = self._marker('camera', 0, Marker.ARROW, (1.0, 0.55, 0.1, 1.0))
        gaze.points = [_pt(x, y, oz), _pt(x + 0.35 * d[0], y + 0.35 * d[1], oz + 0.35 * d[2])]
        gaze.scale.x, gaze.scale.y, gaze.scale.z = 0.015, 0.04, 0.05
        out.markers.append(gaze)

        hh, hv = math.radians(self.hfov / 2), math.radians(self.vfov / 2)
        corners = [self._ray_end(x, y, oz, self._cam_dir(yaw, pan, tilt, th, ph))
                   for th, ph in ((hh, hv), (-hh, hv), (-hh, -hv), (hh, -hv))]
        fov = self._marker('camera', 1, Marker.LINE_LIST, (1.0, 0.55, 0.1, 0.55))
        for i, c in enumerate(corners):
            n = corners[(i + 1) % 4]
            fov.points += [_pt(x, y, oz), _pt(*c), _pt(*c), _pt(*n)]
        fov.scale.x = 0.008
        out.markers.append(fov)

        # 퍽 (지름 76 mm, 두께 25 mm)
        if self.puck is not None:
            px, py, pz, seen = self.puck
            puck = self._marker('puck', 0, Marker.CYLINDER,
                                (0.15, 0.9, 0.3, 1.0) if seen > 0.5 else (0.95, 0.2, 0.2, 1.0))
            puck.pose.position = _pt(px, py, pz + 0.0125)
            puck.scale.x = puck.scale.y = 0.076
            puck.scale.z = 0.025
            out.markers.append(puck)
            link = self._marker('puck', 1, Marker.LINE_LIST, (1.0, 1.0, 1.0, 0.25))
            link.points = [_pt(x, y, 0.01), _pt(px, py, 0.01)]
            link.scale.x = 0.004
            out.markers.append(link)
            dist = math.hypot(px - x, py - y)
        else:
            dist = float('nan')

        text = self._marker('text', 0, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        text.pose.position = _pt(x, y, 0.55)
        text.scale.z = 0.07
        text.text = (f'{self.status}\n'
                     f'v {self.cmd[0]:+.2f} m/s  w {self.cmd[1]:+.2f} rad/s\n'
                     f'puck {dist:.2f} m  pan {pan_deg:+.0f}  tilt {tilt_deg:+.0f}')
        out.markers.append(text)

        self.pub.publish(out)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.frame
            t.child_frame_id = 'base_link'
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = SimViz()
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
