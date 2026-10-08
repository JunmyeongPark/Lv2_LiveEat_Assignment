#!/usr/bin/env python3
"""jpeg_tap.py — 컬러 영상을 N 프레임마다 1장 JPEG 로 줄여 /camera/color/jpeg 로 발행 (bag 기록 부하 줄이기)

원본 /camera/camera/color/image_raw (640x360 rgb8, 30 fps ≈ 21 MB/s) 를 그대로 bag 에 넣으면
라즈베리파이 SD 카드가 못 따라가 메시지 손실 · 진단 FAULT 가 난다 (10/8 B0 1차 시도).
이 노드는 원본을 직렬화된 채로 받아 N 번째 프레임만 풀어서 JPEG 로 압축해 보낸다 (나머지는 풀지도 않음).
header.stamp 는 원본 그대로 → 같은 시각의 /detection · planning CSV 와 맞춰 볼 수 있다.

  python3 tools/inject/jpeg_tap.py [N, 기본 6 → 30 fps 에서 5 fps] [JPEG 품질, 기본 85]
  bag: bash ~/start_bag_jpeg.sh BAG-OK
  이미지 보기 (PC): bag 을 재생하고 /camera/color/jpeg (CompressedImage) 를 받는다
"""
import io
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
except ImportError:
    cv2 = None
    from PIL import Image as PILImage


class JpegTap(Node):
    def __init__(self, every, quality):
        super().__init__('jpeg_tap')
        self.every, self.quality, self.n, self.sent = every, quality, 0, 0
        self.recent = deque(maxlen=8)   # 최근 프레임 stamp (두 구독에서 순서가 섞여 와도 중복 제거)
        self.pub = self.create_publisher(CompressedImage, '/camera/color/jpeg', 10)
        # raw=True: 직렬화된 bytes 로 받아서, 고른 프레임만 역직렬화
        # best-effort · reliable 둘 다 구독: 카메라 QoS 가 어느 쪽이든 받게 (큰 메시지는 환경에 따라 한쪽이 안 옴).
        # 같은 프레임이 두 번 오면 header.stamp 바이트로 한 번만 센다.
        topic = '/camera/camera/color/image_raw'
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data, raw=True)
        self.create_subscription(Image, topic, self.cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), raw=True)
        self.create_timer(5.0, lambda: self.get_logger().info(f'받음 {self.n}, 보냄 {self.sent}'))
        self.get_logger().info(f'{every} 프레임마다 1장, JPEG 품질 {quality}, 인코더 {"cv2" if cv2 else "PIL"}')

    def cb(self, raw):
        key = bytes(raw[4:12])   # CDR 머리(4) 다음 header.stamp sec·nanosec
        if key in self.recent:
            return
        self.recent.append(key)
        self.n += 1
        if (self.n - 1) % self.every:
            return
        m = deserialize_message(raw, Image)
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        if cv2 is not None:
            bgr = a if m.encoding == 'bgr8' else a[:, :, ::-1]
            ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            data = buf.tobytes() if ok else b''
        else:
            rgb = a if m.encoding == 'rgb8' else a[:, :, ::-1]
            out = io.BytesIO()
            PILImage.fromarray(rgb).save(out, 'JPEG', quality=self.quality)
            data = out.getvalue()
        c = CompressedImage()
        c.header = m.header
        c.format = 'jpeg'
        c.data = data
        self.pub.publish(c)
        self.sent += 1


def main():
    every = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    quality = int(sys.argv[2]) if len(sys.argv) > 2 else 85
    rclpy.init()
    node = JpegTap(every, quality)
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
