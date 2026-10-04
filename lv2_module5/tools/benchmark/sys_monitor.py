#!/usr/bin/env python3
"""
sys_monitor.py — 벤치마크 중인 프로세스의 CPU·메모리와 라즈베리파이 상태를 주기적으로 기록

기록 항목 (CSV)
  t_s            모니터 시작 후 경과 시간 [s]
  proc_cpu_pct   대상 프로세스 CPU 사용률 [%] (코어 4개 모두 쓰면 최대 400)
  proc_rss_mb    대상 프로세스 메모리 (RSS) [MB]
  sys_cpu_pct    시스템 전체 CPU 사용률 [%] (0~100)
  temp_c         SoC 온도 [°C]
  cpu_mhz        CPU0 현재 클럭 [MHz] (스로틀링 시 내려감)
  throttled      vcgencmd get_throttled 값 (0x0 = 정상, 라즈베리파이 OS/펌웨어에서만)

사용
  python3 sys_monitor.py --pid <PID> --out ncnn_sys.csv [--interval 0.5]
  → 대상 프로세스가 끝나면 자동 종료
"""
import argparse
import csv
import shutil
import subprocess
import sys
import time

try:
    import psutil
except ImportError:
    sys.exit("psutil 필요: pip install psutil")


def read_temp_c():
    for path in ("/sys/class/thermal/thermal_zone0/temp",):
        try:
            with open(path) as f:
                return int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            pass
    return float("nan")


def read_cpu_mhz():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return float("nan")


VCGENCMD = shutil.which("vcgencmd")


def read_throttled():
    if not VCGENCMD:
        return ""
    try:
        out = subprocess.run([VCGENCMD, "get_throttled"], capture_output=True, text=True, timeout=1).stdout
        return out.strip().split("=")[-1]
    except (OSError, subprocess.SubprocessError):
        return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--interval", type=float, default=0.5)
    a = ap.parse_args()

    proc = psutil.Process(a.pid)
    proc.cpu_percent(None)
    psutil.cpu_percent(None)
    t0 = time.monotonic()

    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "proc_cpu_pct", "proc_rss_mb", "sys_cpu_pct", "temp_c", "cpu_mhz", "throttled"])
        while True:
            time.sleep(a.interval)
            try:
                if not proc.is_running() or proc.status() == psutil.STATUS_ZOMBIE:
                    break
                cpu = proc.cpu_percent(None)
                rss = proc.memory_info().rss / 1e6
            except psutil.NoSuchProcess:
                break
            w.writerow([f"{time.monotonic() - t0:.2f}", f"{cpu:.1f}", f"{rss:.1f}",
                        f"{psutil.cpu_percent(None):.1f}", f"{read_temp_c():.1f}",
                        f"{read_cpu_mhz():.0f}", read_throttled()])
            f.flush()


if __name__ == "__main__":
    main()
