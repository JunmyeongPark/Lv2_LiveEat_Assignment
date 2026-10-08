#!/usr/bin/env python3
"""analyze_run.py — bag → timeseries.csv + 이벤트·지표 (planning 코드 수정 없이 시각 계산)

사용:
  python3 analyze_run.py <run_dir> [--kind r1|c1|generic] [--stop-eps 1e-3]
  run_dir = experiment_logs/<run_id>  (안에 bag/ 있음)

출력(run_dir):
  timeseries.csv  — 모든 토픽을 bag 기록시각(epoch s) 기준으로 합친 표(변화 시 1행)
  events.csv      — state/reason 전이 목록
  summary.json    — 지표(주입·반응·재등장·복구 시각, 정지 확인 등)
시각은 bag 수신 시각이며 planning 내부 시각이 아니다(수신 지연 포함). 기록에 그렇게 적을 것.
"""
import argparse, csv, json, math, os, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(bag_dir):
    opts = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='')
    conv = rosbag2_py.ConverterOptions('', '')
    r = rosbag2_py.SequentialReader(); r.open(opts, conv)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, data, ts = r.read_next()
        yield ts * 1e-9, topic, deserialize_message(data, get_message(types[topic]))


def parse_status(s):
    parts = s.split()
    d = {'state': parts[0].lower()}
    for p in parts[1:]:
        if '=' in p:
            k, v = p.split('=', 1); d[k] = v
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir'); ap.add_argument('--kind', default='generic')
    ap.add_argument('--stop-eps', type=float, default=1e-3)
    a = ap.parse_args()
    bag = os.path.join(a.run_dir, 'bag')
    cur = dict(state='', reason='', heading='', diag='', det_x='', det_y='', det_z='',
               cmd_v='', cmd_w='', arm_pan_cmd='', arm_tilt_cmd='', arm_pan_act='', arm_tilt_act='')
    rows, events = [], []
    for t, topic, m in read_bag(bag):
        ch = True
        if topic.endswith('tracking_status'):
            d = parse_status(m.data)
            if (d['state'], d.get('reason')) != (cur['state'], cur['reason']):
                events.append((t, cur['state'], d['state'], cur['reason'], d.get('reason')))
            cur.update(state=d['state'], reason=d.get('reason', ''), heading=d.get('heading', ''), diag=d.get('diag', ''))
        elif topic.endswith('/detection'):
            cur.update(det_x=m.point.x, det_y=m.point.y, det_z=m.point.z)
        elif topic.endswith('planning/cmd_vel'):
            cur.update(cmd_v=m.linear.x, cmd_w=m.angular.z)
        elif topic.endswith('planning/arm_command'):
            cur.update(arm_pan_cmd=m.data[0], arm_tilt_cmd=m.data[1])
        elif topic.endswith('control/joint_states'):
            cur.update(arm_pan_act=math.degrees(m.position[2]), arm_tilt_act=math.degrees(m.position[3]))
        else:
            ch = False
        if ch:
            rows.append(dict(t=t, **cur))
    if not rows:
        sys.exit('bag에 메시지 없음')
    t0 = rows[0]['t']
    with open(os.path.join(a.run_dir, 'timeseries.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['t', 't_rel'] + list(cur)); w.writeheader()
        for r in rows: w.writerow(dict(r, t_rel=round(r['t'] - t0, 4)))
    with open(os.path.join(a.run_dir, 'events.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 't_rel', 'from_state', 'to_state', 'from_reason', 'to_reason'])
        for e in events: w.writerow([e[0], round(e[0] - t0, 4)] + list(e[1:]))

    S = dict(run_dir=a.run_dir, bag_start_epoch=t0, duration_s=rows[-1]['t'] - t0,
             n_events=len(events), transitions=[f"{e[1]}->{e[2]}@{e[0]-t0:.3f}s({e[4]})" for e in events])
    # 검출 z 이력 (z=0 미검출)
    det = [(r['t'], float(r['det_z'])) for r in rows if r['det_z'] != '']
    # 미검출 구간(주입)과 재등장
    miss_start = next((t for t, z in det if z == 0.0), None)
    S['first_z0_epoch'] = miss_start
    if miss_start:
        reapp = next((t for t, z in det if t > miss_start and z > 0.0), None)
        S['reappear_epoch'] = reapp
        # 반응: 첫 미검출 이후 cmd_v==cmd_w==0 이 확인된 시각, 첫 SEARCHING/FAULT 전이
        S['first_zero_cmd_after_miss_s'] = next((r['t'] - miss_start for r in rows if r['t'] >= miss_start
                                                 and r['cmd_v'] != '' and abs(float(r['cmd_v'])) < a.stop_eps
                                                 and abs(float(r['cmd_w'])) < a.stop_eps), None)
        searching = next((e[0] for e in events if e[2] == 'searching' and e[0] >= miss_start), None)
        S['to_searching_after_miss_s'] = None if searching is None else searching - miss_start
        if reapp:
            rec = next((e[0] for e in events if e[2] == 'tracking' and e[0] >= reapp), None)
            S['recover_time_s'] = None if rec is None else rec - reapp
            S['recover_within_3s'] = (rec is not None) and (rec - reapp <= 3.0)
            # 가림 중 회전 여부: miss_start~reapp 동안 |cmd_w| 최대
            ws = [abs(float(r['cmd_w'])) for r in rows if miss_start <= r['t'] <= reapp and r['cmd_w'] != '']
            S['max_abs_cmd_w_during_occlusion'] = max(ws) if ws else None
            S['occlusion_duration_s'] = reapp - miss_start
    # FAULT 반응(c1): 마지막 신선한 detection 수신 → fault 전이
    fault = next((e for e in events if e[2] == 'fault'), None)
    if fault:
        last_det = max((t for t, _ in det if t <= fault[0]), default=None)
        S['fault_epoch'] = fault[0]; S['fault_reason'] = fault[4]
        S['last_detection_before_fault_epoch'] = last_det
        S['fault_react_s_from_last_detection'] = None if last_det is None else fault[0] - last_det
        S['cmd_zero_at_fault'] = next((r['cmd_v'] == 0 and r['cmd_w'] == 0 for r in rows if r['t'] >= fault[0] and r['cmd_v'] != ''), None)
    with open(os.path.join(a.run_dir, 'summary.json'), 'w') as f:
        json.dump(S, f, ensure_ascii=False, indent=2)
    print(json.dumps(S, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
