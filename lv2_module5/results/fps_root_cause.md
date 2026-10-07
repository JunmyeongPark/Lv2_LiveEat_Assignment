# 인지 FPS 원인 분석 (진행 중)

인지 노드(`perception_master`)의 발행 FPS가 기대보다 낮은 원인을 찾는 기록이다. 가설을 하나씩 세우고 검증한 순서대로 남긴다. 새 측정이 나오면 4절에 가설을 추가하고 1절 요약을 고친다.

- 마지막 갱신: 2026-10-07
- 관련 보고서: [realtime_ncnn_vs_onnx.md](realtime_ncnn_vs_onnx.md) (2.6절 해석 4, 3절 결론 6)

## 1. 현재 결론 요약

| 질문 | 답 | 상태 |
|---|---|---|
| 단독 실행에서 콜백 107 ms인데 왜 6.9 FPS(간격 145 ms)인가? | 추론 스레드(3개)와 카메라 노드의 **CPU 경쟁**. 스레드 2개면 대기 38 → 12.5 ms, 8.0 FPS | **확인** |
| 통합 실행(인지+판단+제어)은 왜 3 FPS인가? | bringup이 `num_threads: 4`로 띄움 → 통합 측정 4.63 FPS까지는 설명됨. 4.6 → 3은 기록 없음 | **부분 확인** |
| ROS 동기화(`ApproximateTime`)가 느리게 하나? | FPS는 안 줄인다. 대신 짝을 한 주기 붙잡아 **지연**을 늘린다 | **확인** (PC 모의) |
| main 통합 실행에서 `camera_diag_error`/`timeout`으로 FAULT가 반복되는 이유는? | 스레드 4라면 처리 간격 0.5초 초과로 인한 `sync fail`(41초에 6번)로 설명된다. 하지만 문제의 테스트는 **스레드 2**였고, 이 조건에서는 드물다. color·depth 시점 어긋남, 촬영 시각 역행은 실측으로 기각. 나머지는 진단 메시지로 확인 필요 | **미확인** (6-2절) |

**당장 할 일**: 통합 실행 중 `/perception/camera_health` 메시지 내용 확인 → `config/perception.yaml`의 `num_threads`를 4 → 2로 바꾸고 다시 잰다 (7절).

## 2. 문제 정의

| 항목 | 값 | 출처 |
|---|---|---|
| 단독 측정 (카메라 + 인지만, NCNN 320x256 스레드 3) | 콜백 106.9 ms, 처리 간격 145.3 ms, **6.88 FPS** | `logs/controlled_320b/ncnn_320x256_t3.csv` |
| 콜백만으로 가능한 FPS | 1000 / 106.9 = **9.4 FPS** | 위 CSV |
| 손실 | 프레임마다 약 **38 ms** 대기 | 간격 − 콜백 |
| 통합 실행 (bringup, 실제 주행) | 약 **3 FPS** (팀 관찰, 로그 없음) | — |

처리 간격은 `header.stamp`(촬영 시각) 차이다. 카메라가 15 FPS(66.7 ms 주기)라서 처리한 프레임 간격은 모두 66.7 ms의 정확한 배수다(배수에서 벗어난 평균 0.1 ms). 스레드 3 단독 실행은 간격의 63%가 2주기(133 ms), 18%가 3주기(200 ms)였다.

## 3. 사용한 자료

| 자료 | 위치 | 내용 |
|---|---|---|
| 단독 통제 측정 | `results/logs/controlled_1005/`, `controlled_320b/` | 카메라 + 인지만. 프레임별 CSV, 온도·클럭, CPU |
| 통합 측정 | `perception_hotfix` 브랜치 `results/logs/fps_cam_planning*`, `fps_pin_*` | 카메라 + planning(+control) 상태에서 스레드 2/3/4, 코어 고정 유무. 스크립트 `tools/benchmark/realtime/integrated_fps.sh` |
| PC 실측 (D435 직결) | `perception_test/yolo/realsense_test.py` | A 키로 정렬 ON/OFF, `--fps 15/30`. 종료 시 정렬 시간·루프·지연 통계 (A 키는 이후 제거됨, 측정 당시 코드는 커밋 5aa0e72의 `realsense_gpt.py`) |
| PC 모의 실험 | 레포 밖 임시 폴더 (재현 방법은 6절) | librealsense 직접 + sleep, ROS `ApproximateTime` 시험 노드 + 가짜 카메라 |

## 4. 가설과 검증 기록

### H1. 카메라 노드의 depth 정렬 연산이 무겁다 → **기각**
- 처음 보고서에 쓴 추정이었다 (`align_depth`가 추론과 CPU를 다퉈 짝이 늦게 온다).
- PC에서 `rs.align` 1회 약 **2 ms**. `realsense_test.py`에서 정렬을 켜고 꺼도 처리 간격이 같았다.
- 다만 PC는 x86이라 Pi보다 훨씬 빠르다. Pi에서의 정렬 시간 자체는 아직 재지 않았다. 정렬은 카메라 노드 CPU(약 40%)의 일부로 H4에 포함된다.

### H2. 15 FPS 프레임 주기 자체 때문에 기다린다 → **기각**
- PC에서 librealsense를 직접 쓰고 추론을 107 ms sleep으로 흉내 냈다.

| 추론(흉내) | 카메라 | 처리 FPS | 대기 | 지연(촬영 → 처리 시작) |
|---|---|---|---|---|
| 107 ms | 15 FPS | 9.0 | 4 ms | 103 ms |
| 107 ms | 30 FPS | 9.0 | 4 ms | 48 ms |
| 30 ms | 15 FPS | 15.0 | 37 ms | 65 ms |
| 30 ms | 30 FPS | 29.2 | 4 ms | 47 ms |

- 추론이 카메라 주기보다 느리면 그사이 다음 프레임이 이미 큐에 있어 대기가 없다. 주기 자체는 원인이 아니다.
- 30 FPS는 처리 FPS는 같고 **지연이 절반**이 된다(103 → 48 ms).

### H3. ROS `ApproximateTime` 동기화가 짝을 늦게 내보낸다 → **FPS 원인 기각, 지연 원인 확인**
- 소스(`message_filters/sync_policies/approximate_time.hpp`, Lyrical): 더 나은 짝이 없다는 것이 증명될 때만 내보낸다. 기본 설정(`inter_message_lower_bounds_` = 0)에서는 **다음 프레임이 와야** 이전 짝을 내보낸다.
- PC 모의 실험: `perception_master`와 같은 구성(SensorDataQoS 구독 2개, `ApproximateTime(10)`, slop 0.02 s)에 sleep을 넣고 가짜 color·depth(stamp 3 ms 차이)를 발행했다.

| 추론(흉내) | 카메라 | `setInterMessageLowerBound` | 처리 FPS | 대기 | 지연 |
|---|---|---|---|---|---|
| 30 ms | 15 Hz | 없음 | 15.0 | 36.7 ms | **67.1 ms** |
| 30 ms | 15 Hz | 60 ms | 15.0 | 36.7 ms | **3.6 ms** |
| 107 ms | 15 Hz | 없음 | 9.47 | ≈0 | 394 ms |
| 107 ms | 15 Hz | 60 ms | 9.44 | ≈0 | 295 ms |
| 107 ms | 30 Hz | 없음 | 9.42 | ≈0 | 241 ms |
| 107 ms | 30 Hz | 30 ms | 9.39 | ≈0 | 149 ms |

- 짝을 **정확히 한 주기** 붙잡는다(67 ms). lower bound를 주면 바로 내보낸다.
- 추론이 107 ms이면 대기는 0이고 9.4 FPS → **FPS는 줄이지 않는다.**
- 추론이 카메라보다 느리면 큐에 프레임이 쌓여 0.15~0.4초 전 장면을 처리하게 된다. Pi에서 실제 지연은 CSV에 없어서 모른다 (6절 남은 질문).

### H4. 추론 스레드와 카메라 노드의 CPU 경쟁 → **확인**
- 통합 측정(`perception_hotfix` 브랜치)에서 스레드 수만 바꿨다. 워밍업 10초 제외.

| 조건 | 스레드 | infer mean | 콜백 | 처리 간격 | 대기 | 발행 FPS |
|---|---|---|---|---|---|---|
| 카메라 + planning + control | 4 | 173.8 ms | 177.3 ms | 216.0 ms | 38.7 ms | 4.63 |
| 카메라 + planning + control | 3 | 109.0 ms | 112.3 ms | 145.2 ms | 32.9 ms | 6.89 |
| 카메라 + planning | 3 | 114.8 ms | 118.1 ms | 156.6 ms | 38.5 ms | 6.39 |
| 카메라 + planning | **2** | **107.9 ms** | 111.9 ms | 124.5 ms | **12.6 ms** | **8.03** |
| 카메라 + planning (2차) | **2** | 110.0 ms | 113.8 ms | 126.3 ms | 12.5 ms | 7.92 |
| 코어 고정: 인지 0-1, 나머지 3 | 2 | 109.2 ms | 112.9 ms | 135.9 ms | 23.0 ms | 7.36 |
| 코어 고정: 인지 0-2, 나머지 3 | 3 | 118.4 ms | 122.5 ms | 157.1 ms | 34.5 ms | 6.37 |

- 320x256 추론은 **2스레드로도 3스레드만큼 빠르다**(108 vs 109~115 ms).
- 스레드 2개면 남는 코어를 카메라 노드(top 기준 약 40%: 정렬 + 약 1.5 MB/프레임 발행)가 쓰면서 프레임이 제때 온다 → 대기 12.5 ms.
- 스레드 4개는 추론 자체도 느려진다(단독 131.6 → 통합 173.8 ms).
- 코어 고정은 카메라와 planning을 한 코어에 몰아 오히려 느리다.
- 발열·클럭: 측정 중 평균 1.77~1.80 GHz, 최고 68 °C. 영향 작음.

### 참고: 검증 중 발견한 도구 버그
- `realsense_test.py`의 정렬 OFF 모드에서 `rs2_project_color_pixel_to_depth_pixel`의 외부 파라미터 인자를 반대로 넣어 depth가 완전히 다르게 나왔다. pyrealsense2 2.58의 순서는 `color_to_depth, depth_to_color`다. 수정 후 정렬 ON과 OFF의 depth 차이는 1 cm 이내(D435 실측).

## 5. 통합 실행 3 FPS 분석

| 단계 | FPS | 근거 |
|---|---|---|
| 단독, 스레드 3 (팀 테스트) | 6.88 | `controlled_320b` |
| bringup은 `pin_cpu:=false`가 기본 → `perception.yaml`의 `num_threads: 4` 사용 | — | `launch/bringup.launch.py`, `config/perception.yaml` (main) |
| 카메라 + planning + control, 스레드 4 | 4.63 | `fps_cam_planning/t4` (H4 표) |
| 실제 통합 주행 | 약 3 | 로그 없음 |

4.63 → 3의 후보 (미확인):
1. 실제 주행 중 control 부하(50 Hz 시리얼·모터)와 planning 대시보드·이벤트 로그
2. PC의 rviz 등이 Wi-Fi로 영상 토픽을 구독 → 카메라 노드가 프레임을 네트워크로 한 번 더 보냄. 측정 스크립트도 이를 의심해 이미지 구독자 수를 기록한다
3. 이전 실행에서 남은 프로세스. 첫 통합 측정 때 레포에 없는 `/target_detector`, `/tracking_controller` 노드가 떠 있었다 (그때 CPU는 거의 안 씀)

## 6. 낮은 FPS가 카메라 FAULT로 이어지는 경로 (main)

main(3e8e04a)으로 통합 실행할 때 planning이 카메라 진단 때문에 FAULT를 반복했다. 함께 본 로그(참고용, 정확한 출력인지 불확실):

```
[realsense2_camera_node-1] ... RealSense Node Is Up!                         (t = 72.89 s)
[planning_master-4] FAULT -> IDLE reason=recovered_from:camera_diag_timeout   (t = 75.61 s)
[planning_master-4] IDLE -> FAULT reason=camera_diag_error                     (t = 75.86 s)
```

인지 코드는 측정 당시 커밋(7aef323)과 main이 같다(바뀐 것은 `perception.launch.py`의 코어 지정 옵션뿐). 그래서 4절 통합 측정을 main에 그대로 적용할 수 있다.

**관련 코드 (main)**
- `perception_master`는 `rclcpp::spin` 하나(단일 실행기)에서 추론 콜백, 원본 영상 수신 콜백, 진단 타이머(10 Hz)를 순서대로 처리한다.
- 진단은 마지막으로 짝이 맞은 시각이 `frame_timeout_s`(0.5 s)보다 오래되면 ERROR `sync fail`을 보낸다. 추론 직후 큐에 밀린 수신 콜백보다 타이머가 먼저 돌면 `no color`, `frames stopped` 등도 나올 수 있다.
- planning(`health_monitor.py`)은 카메라 진단이 ERROR이면 `camera_diag_error`, 0.5 s 동안 안 오면 `camera_diag_timeout`으로 FAULT가 되고, **OK 3번 연속**이어야 복귀한다.

**측정 근거**: 처리 간격이 0.5 s를 넘은 횟수

| 조건 | 발행 FPS | 간격 > 0.5 s | 최대 간격 | 추론 최대 |
|---|---|---|---|---|
| 단독 t3 | 6.88 | 0회 | 400 ms | 274 ms |
| 통합 t4 (**bringup 기본값**) | 4.63 | **6회 / 41초** | **1068 ms** | 616 ms |
| 통합 t3 | 6.89 | 0회 | 467 ms | 255 ms |
| 카메라 + planning t2 | 8.03 | 1회 / 55초 | 534 ms | 229 ms |

**해석**
- bringup 기본값(스레드 4)에서는 처리가 0.5 s 넘게 멈추는 순간이 자주 생긴다. 그때마다 카메라가 정상이어도 `sync fail` ERROR가 나가고 planning이 FAULT가 된다. 진단 타이머도 같은 실행기에서 돌아서, 추론이 길면 진단 발행 자체가 늦어져 `camera_diag_timeout`이 될 수 있다.
- **단, 위 로그의 테스트는 `num_threads: 2`였다** (팀 확인). 스레드 2에서는 0.5 s 초과가 50초에 1번 정도이고, 시작 직후 10초 안에는 한 번도 없었다(첫 프레임부터 추론 약 109 ms, 워밍업 지연 없음). 그래서 **이 로그의 FAULT를 스레드 경쟁만으로 설명하기는 어렵다.** 6-2절에서 다른 후보를 확인 중이다.

### 6-1. 처음 켰을 때 증상과의 연결 (추정)

팀이 본 증상: 처음 켜면 카메라가 잠깐 위아래로 삐죽대다가, 퍽을 못 찾아도 탐색(searching)으로 가지 않는다. 터미널에는 `-3`, `-4`가 번갈아 찍힌다.

| 증상 | 코드상 설명 (main) |
|---|---|
| 카메라가 위아래로 삐죽댐 | 카메라는 팔(pan·tilt)에 달려 있다. FAULT이면 팔을 `fault_arm_pose` **tilt 0°**로, IDLE이면 `nominal_pose` **tilt −15°**로 보낸다(`config/planning.yaml`). 6절처럼 FAULT ↔ IDLE이 반복되면 카메라가 15°씩 끄덕인다 |
| 탐색으로 안 감 | IDLE은 **연속 3번 검출해야** TRACKING으로 가고, SEARCHING은 TRACKING에서 연속 3번 놓쳤을 때만 들어간다. 처음부터 퍽이 안 보이면 IDLE에 머무는 것이 설계다. 또 FAULT가 풀리면 항상 **IDLE로** 돌아가므로, 탐색 중에 FAULT가 끼면 탐색이 끊긴다 |
| `-3`, `-4`가 번갈아 찍힘 | 숫자는 launch가 붙이는 프로세스 번호다. bringup 순서상 `-1` realsense, `-2` perception_master, **`-3` control_master**, **`-4` planning_master**다. control과 planning이 번갈아 로그를 낸다는 뜻이다(내용은 미확인) |

원인이 무엇이든 카메라 진단이 ERROR와 OK를 오가면 FAULT(tilt 0°) → OK 3번 → IDLE(tilt −15°) → 다시 FAULT가 반복되어 위 증상이 된다. planning 진단 우선순위는 MCU → 팔 모터 → 휠 모터 → 카메라라서, reason이 `camera_diag_*`로 찍힌 순간에는 control 쪽 진단은 정상이었다. `-3`(control) 로그 내용은 따로 확인해야 한다.

### 6-2. 스레드 2 조건에서 카메라 ERROR 후보 확인

인지 진단(`publish_health`)이 ERROR를 내는 조건을 하나씩 확인한다. PC 실험은 이 PC에 연결한 D435를 librealsense로 직접 열어 15 FPS, 640x480(color RGB8)으로 측정했다. Pi의 ROS 드라이버 경로와 완전히 같지는 않다.

| 후보 | 진단 메시지 | 확인 결과 |
|---|---|---|
| 처리 간격 0.5 s 초과 (스레드 경쟁·워밍업) | `sync fail` | 스레드 2에서는 드묾 (50초에 1번, 시작 10초 안 0번) → **이 증상의 주원인으로는 약함** |
| 카메라를 켤 때마다 color·depth 촬영 시점이 어긋나 `sync_slop`(20 ms)을 넘음 | `sync fail` | **기각.** 8번 다시 켜도 color − depth 차이가 항상 0.0 ms (같은 FPS면 D435가 두 센서를 맞춤) |
| 켠 직후 촬영 시각이 뒤로 튀어 `frozen` 판정이 계속됨 (코드는 최대 시각과 비교) | `color frozen` / `depth frozen` | **기각.** 6번 켜는 동안 뒤로 간 적 0번, 같은 시각 반복 최대 1번 (판정은 10번 연속) |
| 시작 중 한쪽 스트림만 늦게 들어오거나 잠깐 끊김 | `no color` / `no depth` / `frames stopped` | 미확인 (Pi에서 진단 메시지로 확인) |
| 영상 변환·검출 중 예외 | `exception: ...` | 미확인 |
| 카메라가 USB에서 순간 끊김 (전원·케이블) | `camera USB disconnected` | 미확인 |

남은 후보는 모두 **진단 메시지 내용**으로 바로 구분된다 (7절 첫 항목).

## 7. 다음 할 일

- [ ] **FAULT 원인 확인**: 통합 실행 중 `ros2 topic echo /perception/camera_health --field message`. `sync fail`이면 6절 해석이 맞다. `camera USB disconnected`·`frames stopped`·`color frozen`이면 실제 카메라·드라이버 문제다
- [ ] 시작 직후 로그에서 `[control_master-3]`, `[planning_master-4]` 줄 원문 확보 (6-1절). `event_log:=<파일>`로 띄우면 planning 상태 전이가 파일에 남는다
- [ ] `config/perception.yaml` `num_threads: 4` → `2` (팀 공용 설정이라 합의 후 변경)
- [ ] 바꾼 뒤 **통합 실행**에서 측정: `ros2 topic hz /detection`, `ros2 topic info -v /camera/camera/color/image_raw`(구독자 1이 정상), `timing_csv` 기록
- [ ] 통합 실행에서 control·대시보드를 하나씩 끄며 4.6 → 3 원인 분리
- [ ] 콜백 시작 시각(벽시계)을 `timing_csv`에 추가해 Pi의 실제 지연 측정
- [ ] 지연 개선 시험: `sync_->setInterMessageLowerBound()` 추가, 카메라 30 FPS (`perception.launch.py`의 `640x480x15` → `640x480x30`)
- [ ] Pi에서 `realsense_test.py`로 정렬 1회 시간 측정 (`A` 키 전환, 종료 통계)
- [ ] 진단을 추론과 분리 검토: 진단 타이머와 원본 수신 콜백을 별도 callback group + `MultiThreadedExecutor`로 옮기면 추론이 길어도 진단 발행은 제때 나간다. 단 `sync fail`은 실제로 처리가 멈춘 것이라 FPS를 올리는 것이 우선이다 (planning의 `detection_timeout_s`도 0.5 s)

### 모의 실험 재현 방법
레포에 넣지 않았다. 필요하면 `tools/benchmark/`로 옮긴다.
- **librealsense 직접 (H2)**: `pipeline.wait_for_frames()` → `align.process()` → `time.sleep(W)`를 반복하며 `color_frame.get_timestamp()` 간격과 수신 시각을 기록.
- **ApproximateTime (H3)**: rclcpp 노드에 `message_filters::Subscriber<Image>` 2개(SensorDataQoS) + `Synchronizer<ApproximateTime<Image, Image>>(10)`, `setMaxIntervalDuration(0.02 s)`, 콜백에서 `sleep(W)`. 가짜 카메라는 rclpy로 8x8 Image 두 토픽을 같은 주기로 발행(depth stamp +3 ms). `ROS_DOMAIN_ID`를 따로 둬서 다른 노드와 섞이지 않게 한다.
