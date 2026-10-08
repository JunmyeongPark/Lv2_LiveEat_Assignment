# M-C2 — 제어 통신 중단 (평가 7)

- 날짜: 2026-10-08, 실제 로봇 (pa24), 바퀴를 띄운 상태
- 코드: `code_fingerprint.txt` (sha256 11개. 10개는 `control_fault_inject_Kwonhyeokmu`(238e52b) + `planning_csv_log_Kwonhyeokmu`(94f856b) 병합본, `mock_inputs.py` 는 결과 브랜치와 일치)
- 방법: 바퀴가 도는 중에 control_master 를 **`kill -9` (SIGKILL)** 로 강제 종료
  - Ctrl+C 가 아닌 이유: Ctrl+C 는 control 이 종료하면서 정지 명령을 보내므로 보드 측 정지 시험이 안 됨
  - USB 분리는 기구 구조상 불가
- 보드 측 정지 근거: OpenCR 펌웨어 `firmware/opencr_firmware/watchdog.cpp` `CMD_TIMEOUT_MS = 300` (명령이 300 ms 안 오면 전체 정지)

## 결과

| 항목 | 값 (시각 UTC) |
|---|---|
| 회전 명령 | `/planning/cmd_vel` ω = 0.5 rad/s, 20 Hz, 03:32:45.07 ~ 03:33:00.37 |
| kill 직전 바퀴 실제 속도 | 왼 −2.254, 오 +2.254 rad/s (joint_states 306개, 계속 회전 중) |
| kill 실행 | `date` 출력 03:32:52.191 → 바로 `pkill -9 -f control_master` |
| 마지막 상태 수신 (control 이 죽은 시각) | 03:32:52.233 (kill 명령 후 0.04 s) — 이후 joint_states 없음 |
| kill 후 회전 명령 | kill 명령 이후 164개 (마지막 상태 수신 이후로는 163개, 약 8.1 s) 가 계속 발행됐지만 control 이 없어 OpenCR 로 전달 안 됨 |
| 바퀴 정지 | **눈으로 확인: 즉시 정지**, 마지막 속도로 계속 회전하지 않음 |
| 판정 | ✅ 제어 프로그램이 끊겨도 보드 측에서 정지 |

## 기록과 한계
- bag (드라이브 `bags/M-C2`): 03:32:41 ~ 03:33:00.37. control 상태(`/control/*`) 는 03:32:52.233 에 끊기고, 그 뒤는 회전 명령만 기록됨. `control_20261008_033236.csv`: 마지막 줄 03:32:51.39 — SIGKILL 이라 마지막 약 0.8 s 는 파일에 못 씀.
- 정지까지 걸린 시간은 **측정하지 않음** (영상 없음, kill 뒤 control 재시작으로 바퀴 속도를 읽는 단계 생략). 근거는 육안 확인 + 펌웨어 300 ms 설정.
- 같은 날 03:32:29.56 의 1차 kill 은 bag · 회전 명령을 켜기 전이라 시험 아님 (준비 중 재시작).
