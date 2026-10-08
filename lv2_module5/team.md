# team.md — Live.Eat 협업 기록

- 저장소: https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment
- PR 21건(병합 20, 닫힘 1), Issue 2건. 아래 링크와 내용은 2026-10-08 기준 GitHub 기록에서 옮겼다.
- PR 링크: `https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/<번호>`

## 1. 4인 역할·기여

| 이름 / GitHub ID | 역할 | 담당 Issue | 병합된 본인 PR | 다른 PR 리뷰 | 구현·검증 내용 |
|---|---|---|---|---|---|
| 박준명 / [JunmyeongPark](https://github.com/JunmyeongPark) | **팀장 · 테크리드** + 설계 및 통합 | [#9](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/issues/9) planning 새 상태 `wheel_align` | [#1](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/1) 초기 구조, [#13](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/13) wheel align·목표 위치 기반 추적, [#15](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/15) 통합·시뮬레이터 검증, [#17](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/17) [#19](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/19) 시뮬 안정화·SEARCHING, [#21](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/21) 제어·인지 진단 발행·reason 통일, [#23](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/23) 충돌 재해결, [#26](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/26) planning 대시보드 | [#14](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/14) Approve, [#2](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/2) 패킷·토픽·단위 리뷰(엔코더 원시값 추가, health 판단부 이동 요청), [#10](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/10) [#11](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/11) [#20](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/20) [#25](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/25) [#28](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/28) 승인 코멘트, [#3](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/3) [#12](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/12) 승인 | 노드 구조·인터페이스(`/detection`, `/planning/*`, `/control/*`) 설계, planning 좌표 변환·추종·상태 머신, bringup·대시보드, 가상 노드 시뮬레이터, 실험 계획서(M-IN·M-T·M-R1·M-C·M-B0·M-BAG), 10/8 실기 시험·재현 확인 참여 |
| 김민식 / [minsikim-42](https://github.com/minsikim-42) | 인지 | — | [#3](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/3) perception, [#12](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/12) perception build, [#27](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/27) perception chore | [#21](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/21) 리뷰 코멘트·병합 (팀장 PR) | YOLO26n 단일 클래스 학습(v1~v4), NCNN/ONNX export·C++ 검출 노드, depth 추출, 카메라 진단, NCNN vs ONNX·입력 크기 속도 측정(`results/realtime_ncnn_vs_onnx.md`), 10/8 실기 시험·재현 확인 참여 |
| 정수용 / [affluentmind12](https://github.com/affluentmind12) | 판단 | (#6 sensor health — #10 본문 참조) | [#10](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/10) sensor health 기반 상태 전이, [#11](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/11) MCU(OpenCR) 진단 추가, [#14](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/14) imu_driver, [#20](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/20) OpenCR 단절 판정, [#25](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/25) 모터 OFF 진단 | [#15](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/15) [#17](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/17) [#19](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/19) [#23](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/23) 리뷰 (팀장 PR) | `health_monitor`·FAULT 판정·복구 규칙, IMU 드라이버, 진단 시뮬 반영, R1 가림 시험(10/7)·검출률·오검출 육안 판정, 10/8 실기 시험·재현 확인 참여 |
| 권혁무 / [kwonhyeokmu-hub](https://github.com/kwonhyeokmu-hub) | 제어 | [#8](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/issues/8) control 토픽·패킷 확인 | [#2](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/2) 제어 모듈·control_master·OpenCR 펌웨어, [#22](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/22) 회전 미끄러짐 보정, [#28](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/28) IMU 정상 시 yaw 보정·cmd_vel NaN 차단, [#29](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/29) 10/7~8 실험 결과·장애 주입·CSV 로거 | [#1](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/1) [#13](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/13) [#26](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/26) 리뷰 코멘트 (팀장 PR) | OpenCR 펌웨어(watchdog 300 ms·모터·IMU·시리얼), control_master(속도·가속·관절 제한, 명령 timeout, 진단), 바퀴·팔 한계 실측, 장애 주입 도구·bag 스크립트·분석, 실험 결과 정리(`results/README.md`), M-C2 제어 통신 중단 시험, 10/8 실기 시험·재현 확인 참여 |

- 4명 모두 본인 PR이 1건 이상 병합되었고, 다른 사람 PR에 리뷰(Approve·리뷰 코멘트)를 1건 이상 남겼다.
- 팀장 본인 PR(#1·#13·#15·#17·#19·#21·#23·#26)은 모두 다른 팀원(권혁무·김민식·정수용)이 리뷰했다.
- 닫힌 PR: [#16](https://github.com/JunmyeongPark/Lv2_LiveEat_Assignment/pull/16) OpenCR IMU 통합(박준명) — 병합하지 않고 다른 PR로 대체.

## 2. 권한·main 보호

| 항목 | 상태 |
|---|---|
| 팀원 권한 | 팀장 박준명: 저장소 소유자(Admin). 팀원 김민식·정수용·권혁무: **Write** (작업 브랜치 push·PR·리뷰) |
| main 보호 규칙 | Settings → Rules → Rulesets `ruleset1`, Enforcement **Active**, Bypass list **없음**(팀장도 우회 불가). 규칙: **Restrict deletions**(main 삭제 금지) ✅, **Require a pull request before merging** ✅, **Block force pushes** ✅. PR 추가 설정: **Require conversation resolution before merging** ✅, Required approvals **1**, **Dismiss stale pull request approvals when new commits are pushed** ✅. 대상 브랜치: **Default branch (main)** — 작업 브랜치는 대상에서 제외. 캡처: [`1`](docs/main_ruleset_1.png) · [`2`](docs/main_ruleset_2.png) · [`3`](docs/main_ruleset_3.png) · [`4`](docs/main_ruleset_4.png) · [`5`](docs/main_ruleset_5.png) |

## 3. 예외·대행

| 항목 | 내용 |
|---|---|
| 팀장 외 병합 | #13·#26(권혁무), #21(김민식), #15·#17·#19·#23(정수용)은 리뷰한 팀원이 **팀장 승인을 받고** 병합했다. 모두 팀장 본인 PR이며 다른 팀원 리뷰 후 병합. 나머지 13건은 팀장이 병합했다. |
| 팀장 부재 대행 | 없음 |

## 4. 통합 확인

| 날짜 | 확인자 | 내용 |
|---|---|---|
| 2026-10-08 | 권혁무, 정수용, 박준명, 김민식 | 실제 로봇에서 M-IN·M-T·M-C2·M-B0·M-BAG 시험, bag 결과 재분석·입력 재처리로 성능표와 같은 값 확인 (`results/README.md` 7절) |
| 2026-10-08 | 박준명 (팀장) | 최종 main에 `lv2-module5-submit` 태그 생성, 최종 main 기준 실행·정지·재현 확인 완료 |
