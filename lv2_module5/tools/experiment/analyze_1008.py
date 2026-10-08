#!/usr/bin/env python3
"""analyze_1008.py — 10/8 실험 지표를 bag 만으로 다시 계산 (노드 실행 · 모터 없음, 파일만 읽음)

  python3 analyze_1008.py transitions <bag>                    상태 전이 시각 · 사유 · 구간별 몸통 회전량
  python3 analyze_1008.py inputs <bag>                         모의 입력 5종 (IN) 구간별 명령 · 상태
  python3 analyze_1008.py tracking <bag> [시작s] [끝s]          정상 추적 지표 (FAULT·SEARCHING, 유효 추적 비율, 처리 FPS, 수평 RMSE)
  python3 analyze_1008.py replay <원본 bag> <재처리 bag>        입력 재처리: /detection ↔ /target_replay 같은 촬영 시각끼리 비교

시각 기준: bag 수신 시각 (첫 /tracking_status = 0 s). 각 bag 의 시작이 다르므로 bag 안에서만 비교한다.
필요: ROS 2 (rosbag2_py). bag 경로는 metadata.yaml 이 있는 폴더.
"""
import bisect
import math
import statistics as st
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while r.has_next():
        tp, data, ts = r.read_next()
        if tp in out:
            out[tp].append((ts * 1e-9, deserialize_message(data, get_message(ty[tp]))))
    return out


def status_seq(msgs):
    """[(t, state, reason)] 상태 · 사유가 바뀔 때만"""
    seq = []
    for t, m in msgs:
        p = m.data.split()
        state = p[0].split('|')[0]
        reason = next((x[7:] for x in p if x.startswith('reason=')), '')
        if not seq or seq[-1][1:] != (state, reason):
            seq.append((t, state, reason))
    return seq


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def rotation(imu, a, b):
    ys = [(t, m.data) for t, m in imu if a <= t <= b]
    return sum(wrap(y2 - y1) for (_, y1), (_, y2) in zip(ys, ys[1:]))


def cmd_transitions(bag):
    d = read(bag, ['/tracking_status', '/control/imu_yaw_deg'])
    seq = status_seq(d['/tracking_status'])
    t0 = d['/tracking_status'][0][0]
    print(f'{"t(s)":>7}  {"상태":<10} {"사유":<38} 직전 상태 지속(s) · 몸통 회전(°)')
    prev = None
    for t, s, r in seq:
        extra = ''
        if prev and s != prev[1]:
            extra = f'{t - prev[0]:6.2f} s · {rotation(d["/control/imu_yaw_deg"], prev[0], t):+7.0f}°'
        print(f'{t - t0:7.2f}  {s:<10} {r:<38} {extra}')
        if not prev or s != prev[1]:
            prev = (t, s)


def cmd_inputs(bag):
    d = read(bag, ['/inject/event', '/planning/cmd_vel', '/tracking_status', '/detection'])
    ev = [(t, m.data) for t, m in d['/inject/event']]
    seq = status_seq(d['/tracking_status'])
    st_t = [t for t, _ in d['/tracking_status']]

    def state_at(t):
        i = bisect.bisect_right(st_t, t) - 1
        return d['/tracking_status'][max(i, 0)][1].data

    for (a, name), (b, _) in zip(ev, ev[1:]):
        name = name.split(' start')[0]
        # 명령마다 그 시각 직전의 상태를 찾아 TRACKING 인 명령만 (메시지 순서가 아니라 시각으로 짝지음)
        trk = [m for t, m in d['/planning/cmd_vel']
               if a <= t < b and state_at(t).startswith('TRACKING')]
        w = [m.angular.z for m in trk] or [0.0]
        v = [m.linear.x for m in trk] or [0.0]
        tr = [f'{s}({t - a:+.3f}s)' for t, s, _ in seq if a <= t < b]
        print(f'{name:<20} TRACKING 중 ω 중앙 {st.median(w):+.3f} rad/s, v 중앙 {st.median(v):+.3f} m/s  전이: {" ".join(tr) or "-"}')
        if name.startswith('IN4'):   # 첫 미검출 프레임 이후 첫 명령
            miss = next(t for t, m in d['/detection'] if t >= a and m.point.z == 0.0)
            c = next(m for t, m in d['/planning/cmd_vel'] if t >= miss)
            print(f'{"":20} 첫 미검출 수신 후 첫 명령 v={c.linear.x:+.3f} ω={c.angular.z:+.3f}')
        if name.startswith('IN5'):   # 마지막 검출 → FAULT
            last = max(t for t, m in d['/detection'] if t < b)
            fault = next(t for t, s, _ in seq if t >= a and s == 'FAULT')
            print(f'{"":20} 마지막 /detection → FAULT {fault - last:.3f} s')


def cmd_tracking(bag, s0=None, s1=None):
    d = read(bag, ['/tracking_status', '/detection'])
    t0 = d['/tracking_status'][0][0]
    a = t0 + (s0 or 0.0)
    b = t0 + s1 if s1 is not None else d['/tracking_status'][-1][0]
    sts = [(t, m.data.split()[0].split('|')[0]) for t, m in d['/tracking_status'] if a <= t <= b]
    names = [s for _, s in sts]
    trans = sum(1 for x, y in zip(names, names[1:]) if x != y)
    det = [(t, m) for t, m in d['/detection'] if a <= t <= b]
    ts = [t for t, _ in det]
    sk = [t for t, _ in sts]
    use = [m.point.x for t, m in det if m.point.z > 0 and names[max(0, bisect.bisect_right(sk, t) - 1)] == 'TRACKING']
    print(f'구간 {sts[-1][0] - sts[0][0]:.1f} s, 상태 전이 {trans}, FAULT {names.count("FAULT")}, SEARCHING {names.count("SEARCHING")}')
    print(f'유효 추적 비율 {100 * names.count("TRACKING") / len(names):.1f}% ({names.count("TRACKING")}/{len(names)} /tracking_status)')
    print(f'처리 FPS {(len(ts) - 1) / (ts[-1] - ts[0]):.2f} Hz ({len(ts)}개 → {len(ts) - 1} 간격 / {ts[-1] - ts[0]:.2f} s), '
          f'최대 간격 {max(y - x for x, y in zip(ts, ts[1:])) * 1000:.0f} ms')
    print(f'수평 RMSE {math.sqrt(st.mean(x * x for x in use)):.4f} (검출 · TRACKING {len(use)}/{len(det)} 프레임)')


def cmd_replay(orig, rep):
    key = lambda m: (m.header.stamp.sec, m.header.stamp.nanosec)
    o = {key(m): m.point for _, m in read(orig, ['/detection'])['/detection']}
    p = {key(m): m.point for _, m in read(rep, ['/target_replay'])['/target_replay']}
    common = sorted(set(o) & set(p))
    agree = sum(1 for k in common if (o[k].z > 0) == (p[k].z > 0))
    both = [k for k in common if o[k].z > 0 and p[k].z > 0]
    print(f'원본 /detection {len(o)}, 재처리 /target_replay {len(p)}, 같은 촬영 시각 {len(common)}')
    print(f'검출/미검출 일치 {agree}/{len(common)}')
    for n in ('x', 'y', 'z'):
        diff = [abs(getattr(o[k], n) - getattr(p[k], n)) for k in both]
        print(f'  {n} 차이 평균 {st.mean(diff):.5f}, 최대 {max(diff):.5f} ({len(both)} 프레임)')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == 'transitions':
        cmd_transitions(args[0])
    elif cmd == 'inputs':
        cmd_inputs(args[0])
    elif cmd == 'tracking':
        cmd_tracking(args[0], *(float(x) for x in args[1:3]))
    elif cmd == 'replay':
        cmd_replay(args[0], args[1])
    else:
        sys.exit(__doc__)
