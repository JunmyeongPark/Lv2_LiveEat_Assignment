// Watchdog: 명령 끊기면 전체 정지 (매 루프)
//   Pi 가 죽거나 USB 가 빠져서 명령이 CMD_TIMEOUT_MS 동안 안 오면 timed_out
//   → motor_driver 가 바퀴 0, 팔은 그 자리 유지 (= 제어 통신 중단 시 보드 측 정지, 발제 필수)
//   타임아웃이면 USER LED 1 켜짐
#pragma once

#include <Arduino.h>

void watchdog_setup();

// 유효한 명령 패킷을 받을 때마다 호출
void watchdog_feed(uint32_t now_ms);

// 매 루프 호출. 명령을 한 번도 못 받았거나 마지막 명령 후 CMD_TIMEOUT_MS 가 지났으면 true
bool watchdog_check(uint32_t now_ms);
