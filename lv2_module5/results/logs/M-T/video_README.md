# SC-01 영상 — M-T01 ~ M-T08 클립

영상 파일은 드라이브 `videos/` (`../../../recordings/README.md`).

원본 (휴대폰, 드라이브에서 받음 — 파일 생성 시각은 받은 시각이라 촬영 시각으로 쓰지 않음)
- `IMG_9507.MOV` (84.2 s) → bag 드라이브 `bags/M-T_SC-01` (M-T1~M-T6). 영상 시각 = bag 시각 **+1.5 s** (회전 멈춤 · 퍽 재노출 장면으로 맞춤)
- `IMG_9508.MOV` (13.6 s) → bag 드라이브 `bags/M-T_SC-01b` (M-T7~M-T8). 영상 시각 = bag 시각 **−6.5 s** (FAULT 때 카메라가 들리는 장면 = 팔 tilt −27°→0° 시각으로 맞춤)

| 클립 | 원본 | 원본 구간 (s) | 전이 | 측정 (bag) |
|---|---|---|---|---|
| M-T01.mp4 | IMG_9507 | 0.0 ~ 11.5 | IDLE → TRACKING | 연속 3프레임, 0.26 s |
| M-T02.mp4 | IMG_9507 | 11.7 ~ 22.5 | SEARCHING → TRACKING (1바퀴째) | 7.87 s, 361° |
| M-T03.mp4 | IMG_9507 | 22.7 ~ 44.5 | SEARCHING → TRACKING (3바퀴째) | 16.53 s, 765°, 팔 tilt +68.9° |
| M-T04.mp4 | IMG_9507 | 44.5 ~ 69.0 | SEARCHING → LOST | 23.47 s, 1081°, 바퀴 0.41 s 감속 정지 |
| M-T05.mp4 | IMG_9507 | 68.0 ~ 71.0 | LOST → IDLE | 0.90 s, 팔 [−0.09°, −14.77°] |
| M-T06.mp4 | IMG_9507 | 69.0 ~ 82.0 | IDLE → TRACKING | 연속 3프레임, 0.28 s |
| M-T07.mp4 | IMG_9508 | 0.0 ~ 7.0 | TRACKING → FAULT | 마지막 검출 0.509 s 뒤 FAULT, 팔 tilt −27.2° → −0.1° |
| M-T08.mp4 | IMG_9508 | 6.5 ~ 13.6 | FAULT → IDLE | IDLE 먼저, 0.77 s 뒤 TRACKING, recovered_from 약 2 s |

- 자막의 수치는 bag 분석값이다 (영상에서 잰 값 아님). 소리는 뺐다.
- M-T7 · M-T8 의 reason 은 진단 ON 이라 `camera_diag_timeout` / `recovered_from:camera_diag_error` 로 나왔다 (노션 표의 `detection_timeout` 과 이름만 다름).
- 영상 시각 맞춤은 사람이 장면을 보고 한 것이라 ±0.5 s 정도 오차가 있을 수 있다.
