#!/usr/bin/env python3
"""extract_images.py — bag 안의 RGB 영상을 <run_dir>/images/ 에 이미지 파일로 저장

사용:
  python3 extract_images.py <run_dir> [--every N] [--fmt png|jpg] [--topic 토픽]
  run_dir = experiment_logs/<run_id>  (안에 bag/ 있음)

  --every N  N프레임마다 1장 저장 (기본 1 = 전부)
  --fmt      png(기본, 무손실) / jpg(용량 작음)
  --topic    기본 /camera/camera/color/image_raw (CompressedImage 토픽이면 데이터를 그대로 저장)

파일명: <번호 6자리>_<bag 수신 시각 epoch초>.png  → analyze_run.py 의 timeseries.csv 시각(t)과 맞춰 볼 수 있다.
rosbag2_py, numpy, PIL 필요. 노드를 켜지 않고 파일만 읽는다.
"""
import argparse, os, sys
import numpy as np
import rosbag2_py
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def to_pil(m):
    h, w, enc = m.height, m.width, m.encoding
    buf = np.frombuffer(bytes(m.data), dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        a = buf.reshape(h, m.step)[:, :w * 3].reshape(h, w, 3)
        return Image.fromarray(a if enc == 'rgb8' else a[:, :, ::-1], 'RGB')
    if enc in ('rgba8', 'bgra8'):
        a = buf.reshape(h, m.step)[:, :w * 4].reshape(h, w, 4)[:, :, :3]
        return Image.fromarray(a if enc == 'rgba8' else a[:, :, ::-1], 'RGB')
    if enc == 'mono8':
        return Image.fromarray(buf.reshape(h, m.step)[:, :w], 'L')
    sys.exit(f'지원하지 않는 인코딩: {enc} (rgb8/bgr8/rgba8/bgra8/mono8만 지원)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--every', type=int, default=1)
    ap.add_argument('--fmt', choices=['png', 'jpg'], default='png')
    ap.add_argument('--topic', default='/camera/camera/color/image_raw')
    a = ap.parse_args()
    if a.every < 1:
        sys.exit('--every 는 1 이상')
    bag = os.path.join(a.run_dir, 'bag')
    out = os.path.join(a.run_dir, 'images')
    if os.path.exists(out) and os.listdir(out):
        sys.exit(f'{out} 에 이미 파일이 있음. 지우지 말고 다른 폴더로 옮기거나 확인하세요')
    os.makedirs(out, exist_ok=True)

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if a.topic not in types:
        sys.exit(f'bag에 {a.topic} 토픽이 없음. 있는 토픽: {sorted(types)}')
    msg_type = get_message(types[a.topic])

    n_seen = n_saved = 0
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic != a.topic:
            continue
        n_seen += 1
        if (n_seen - 1) % a.every:
            continue
        m = deserialize_message(data, msg_type)
        name = os.path.join(out, f'{n_saved + 1:06d}_{ts * 1e-9:.3f}.{a.fmt}')
        if hasattr(m, 'format'):                      # CompressedImage: 이미 jpg/png 데이터
            with open(name, 'wb') as f:
                f.write(bytes(m.data))
        else:
            to_pil(m).save(name, quality=90) if a.fmt == 'jpg' else to_pil(m).save(name)
        n_saved += 1
    print(f'{a.topic}: 프레임 {n_seen}개 중 {n_saved}장 저장 → {out}')
    if n_seen == 0:
        print('주의: 영상 메시지가 0개. start_bag_rgb.sh 로 기록했는지 확인')


if __name__ == '__main__':
    main()
