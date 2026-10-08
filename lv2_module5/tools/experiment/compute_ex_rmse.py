#!/usr/bin/env python3
"""compute_ex_rmse.py — bag 하나에서 수평 오차 e_x 의 RMSE 계산

사용:
  python3 compute_ex_rmse.py <run_dir>          # 예: experiment_logs/R1-02
  결과: <run_dir>/ex_rmse.csv 와 화면 출력

e_x = /detection 의 point.x (화면 중심 대비 가로 오차, -1~+1 정규화, 오른쪽 +).
RMSE = sqrt( sum(e_x^2) / N ),  N = z>0(검출됨) 인 /detection 프레임 수. 단위는 정규화 값(각도 ≈ e_x × 34.5°).

두 구간을 함께 낸다.
  pre : 마지막 TRACKING 진입 ~ 가림 시작(SEARCHING 직전 연속 미검출 시작). 가림 전 추적 구간
  all : bag 전체에서 z>0 인 모든 프레임
시각은 bag 수신 시각 기준 t_rel(s) (analyze_run.py 의 t_rel 과 같은 기준).
"""
import argparse, csv, math, os, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TRACK_TOPICS = ('tracking_status', '/detection', 'planning/cmd_vel', 'planning/arm_command', 'control/joint_states')


def read_bag(bag_dir):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, data, ts = r.read_next()
        yield ts * 1e-9, topic, deserialize_message(data, get_message(types[topic]))


def stats(xs):
    n = len(xs)
    if n == 0:
        return dict(n=0, rmse='', mean='', max_abs='')
    return dict(n=n, rmse=round(math.sqrt(sum(x * x for x in xs) / n), 4),
                mean=round(sum(xs) / n, 4), max_abs=round(max(abs(x) for x in xs), 4))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    a = ap.parse_args()
    det, states, t0 = [], [], None          # det: (t, ex, z)   states: (t, state)
    for t, topic, m in read_bag(os.path.join(a.run_dir, 'bag')):
        if t0 is None and topic.endswith(TRACK_TOPICS):
            t0 = t
        if topic.endswith('/detection'):
            det.append((t, m.point.x, m.point.z))
        elif topic.endswith('tracking_status'):
            s = m.data.split()[0].split('|')[0].lower()
            if not states or states[-1][1] != s:
                states.append((t, s))
    if not det or t0 is None:
        sys.exit('bag에 /detection 또는 /tracking_status 가 없음')
    det = [(t - t0, x, z) for t, x, z in det]
    states = [(t - t0, s) for t, s in states]

    rows = []
    run_id = os.path.basename(os.path.normpath(a.run_dir))
    allx = [x for _, x, z in det if z > 0]
    rows.append(dict(run_id=run_id, window='all', start_s='', end_s='', **stats(allx),
                     note='bag 전체의 z>0 프레임'))

    # pre: 첫 SEARCHING 직전의 마지막 TRACKING 진입 ~ 그 SEARCHING 직전 연속 미검출 시작
    s_idx = next((i for i, (_, s) in enumerate(states) if s == 'searching'), None)
    if s_idx is None:
        rows.append(dict(run_id=run_id, window='pre', start_s='', end_s='', n=0, rmse='', mean='', max_abs='',
                         note='SEARCHING 전이 없음 → 가림 구간을 못 찾음'))
    else:
        t_search = states[s_idx][0]
        start = next((t for t, s in reversed(states[:s_idx]) if s == 'tracking'), None)
        # SEARCHING 직전의 연속 z=0 시작
        k = max((i for i, (t, _, _) in enumerate(det) if t <= t_search), default=0)
        while k > 0 and det[k - 1][2] == 0:
            k -= 1
        end = det[k][0]
        if start is None:
            rows.append(dict(run_id=run_id, window='pre', start_s='', end_s=round(end, 2), n=0, rmse='', mean='',
                             max_abs='', note='SEARCHING 이전에 TRACKING 진입이 없음'))
        else:
            xs = [x for t, x, z in det if start <= t < end and z > 0]
            rows.append(dict(run_id=run_id, window='pre', start_s=round(start, 2), end_s=round(end, 2), **stats(xs),
                             note='마지막 TRACKING 진입 ~ 가림 시작'))

    out = os.path.join(a.run_dir, 'ex_rmse.csv')
    with open(out, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=['run_id', 'window', 'start_s', 'end_s', 'n', 'rmse', 'mean', 'max_abs', 'note'])
        w.writeheader(); w.writerows(rows)
    for r in rows:
        print(f"{r['run_id']} {r['window']:>3}: n={r['n']} RMSE={r['rmse']} mean={r['mean']} max|ex|={r['max_abs']} "
              f"[{r['start_s']}~{r['end_s']}s] {r['note']}")
    print('저장:', out)


if __name__ == '__main__':
    main()
