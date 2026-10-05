// Watchdog: 명령 끊기면 전체 정지 (매 루프)
#include "watchdog.h"

namespace
{
const uint32_t CMD_TIMEOUT_MS = 300;  // 이 시간 동안 명령 없으면 정지 (Pi 의 cmd_timeout_s 와 같게)
uint32_t last_cmd_ms = 0;
bool fed = false;                     // 명령을 한 번이라도 받았는지 (시작은 명령 없음 = 정지)
}  // namespace

void watchdog_setup()
{
  pinMode(BDPIN_LED_USER_1, OUTPUT);
}

void watchdog_feed(uint32_t now_ms)
{
  last_cmd_ms = now_ms;
  fed = true;
}

bool watchdog_check(uint32_t now_ms)
{
  bool timed_out = !fed || (now_ms - last_cmd_ms > CMD_TIMEOUT_MS);
  digitalWrite(BDPIN_LED_USER_1, timed_out ? LOW : HIGH);   // 타임아웃이면 LED 켜짐 (OpenCR LED 는 LOW 가 켜짐)
  return timed_out;
}
