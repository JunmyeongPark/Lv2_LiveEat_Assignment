#!/usr/bin/env python3
"""
OpenCR IMU 드라이버: 초기 구간(버퍼 잔여 데이터 + 자이로 보정/필터 수렴)을 건너뛰고
안정화된 값만 반환/발행한다.

수신 형식(10Hz, 한 줄):
  ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,yaw_deg
'#'으로 시작하는 줄은 안내/오류 메시지(화면에만 출력).

초기값 제외 방식:
  1) 포트를 연 직후 입력 버퍼를 비운다 (이전에 쌓여 있던 오래된 줄 제거).
  2) 수렴 판정 전까지 들어오는 값은 버린다.
       - 최근 window_s 초 동안 yaw 변화폭 < yaw_tol_deg
       - 최근 window_s 초 동안 |자이로| < gyro_tol_dps
       - 그 window가 PC 시간 기준 실제로 window_s 초에 걸쳐 도착 (버퍼 덤프 방지)
  3) 펌웨어가 보정 완료 메시지(--ready-msg, 예: "# calib done")를 보내면 그걸 우선 사용.
  4) timeout_s 안에 수렴하지 않으면 TimeoutError.

사용 (라이브러리):
    with IMUDriver("/dev/ttyACM0") as imu:      # 수렴할 때까지 여기서 대기
        while True:
            s = imu.read()                      # dict 또는 None(타임아웃)
            print(s["yaw_deg"])

사용 (단독 실행 / ROS 2 발행):
    python3 imu_driver.py --port /dev/ttyACM0 --ros      # /imu/yaw (Float32) 발행
    python3 imu_driver.py --port /dev/ttyACM0 --out imu.csv
        # 수렴 후 30초 동안 (time_s, yaw_deg)만 CSV로 저장하고 자동 종료
        # --duration N 으로 시간 변경, 0이면 Ctrl+C까지

설치: sudo apt install python3-serial   (또는 pip install pyserial)
주의: 같은 포트를 다른 프로그램이 동시에 열면 안 됩니다. 로봇은 정지 상태로 시작해야 합니다.
"""
import argparse
import csv
import sys
import time
from collections import deque

import serial

FIELDS = ["ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
          "roll_deg", "pitch_deg", "yaw_deg"]


CSV_COLUMNS = ["time_s", "yaw_deg"] 


def parse_line(line: str):
    """정상 데이터 줄이면 dict, 아니면 None."""
    parts = line.split(",")
    if len(parts) != len(FIELDS):
        return None
    try:
        return dict(zip(FIELDS, (float(p) for p in parts)))
    except ValueError:
        return None


class IMUDriver:
    def __init__(self, port="/dev/ttyACM0", baud=115200,
                 window_s=1.0, yaw_tol_deg=0.1, gyro_tol_dps=2.0,
                 timeout_s=20.0, ready_msg=None, verbose=True):
        self.port, self.baud = port, baud
        self.window_s = window_s
        self.yaw_tol = yaw_tol_deg
        self.gyro_tol = gyro_tol_dps
        self.timeout_s = timeout_s
        self.ready_msg = ready_msg
        self.verbose = verbose
        self.ser = None

    # --- 수명 관리 ---
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def start(self):
        """포트를 열고 수렴할 때까지 대기. 이후 read()는 안정된 값만 준다."""
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        self.ser.reset_input_buffer()  # 포트 열기 전에 쌓인 오래된 줄 제거
        self._wait_converged()

    # --- 내부 ---
    def _readline(self):
        raw = self.ser.readline()
        if not raw:
            return None, None
        return raw.decode("utf-8", errors="ignore").strip(), time.time()

    def _log(self, msg):
        if self.verbose:
            print(msg, file=sys.stderr)

    def _wait_converged(self):
        t_start = time.time()
        win = deque()  # (pc_time, sample)
        self._log(f"[imu] 수렴 대기 중... (정지 상태 유지, 최대 {self.timeout_s:.0f}s)")
        while time.time() - t_start < self.timeout_s:
            line, now = self._readline()
            if not line:
                continue
            if line.startswith("#"):
                self._log(f"[imu] {line}")
                if self.ready_msg and self.ready_msg in line:
                    self._log("[imu] 펌웨어 보정 완료 신호 수신")
                    self.ser.reset_input_buffer()
                    return
                continue
            s = parse_line(line)
            if s is None:
                continue
            win.append((now, s))
            while win and now - win[0][0] > self.window_s:
                win.popleft()
            if self.ready_msg:
                continue  # 신호 방식이면 데이터 기반 판정은 하지 않음
            if self._is_stable(win, now):
                self._log("[imu] 수렴 완료, 실제 값 수신 시작")
                return
        raise TimeoutError(f"IMU가 {self.timeout_s}s 안에 수렴하지 않음 "
                           "(로봇이 움직이는 중이거나 펌웨어 보정 미완료)")

    def _is_stable(self, win, now):
        # window가 실제로 window_s 초에 걸쳐 도착했는지 (버퍼 덤프 배제)
        if len(win) < 3 or now - win[0][0] < self.window_s * 0.9:
            return False
        yaws = [s["yaw_deg"] for _, s in win]
        if max(yaws) - min(yaws) > self.yaw_tol:
            return False
        for _, s in win:
            if max(abs(s["gx_dps"]), abs(s["gy_dps"]), abs(s["gz_dps"])) > self.gyro_tol:
                return False
        return True

    # --- 공개 API ---
    def read(self):
        """다음 정상 샘플(dict). 1초 동안 데이터가 없으면 None."""
        while True:
            line, now = self._readline()
            if line is None:
                return None
            if not line:
                continue
            if line.startswith("#"):
                self._log(f"[imu] {line}")
                continue
            s = parse_line(line)
            if s is not None:
                s["pc_time_s"] = now
                return s


def run_ros(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32

    rclpy.init()
    node = Node("imu_driver")
    pub = node.create_publisher(Float32, "/imu/yaw", 10)
    drv = IMUDriver(args.port, args.baud, timeout_s=args.timeout,
                    ready_msg=args.ready_msg)
    try:
        drv.start()  # 수렴 전에는 발행하지 않음
        while rclpy.ok():
            s = drv.read()
            if s is None:
                node.get_logger().warn("IMU 데이터 없음")
                continue
            pub.publish(Float32(data=s["yaw_deg"]))
    except KeyboardInterrupt:
        pass
    finally:
        drv.close()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=20.0, help="수렴 대기 최대 시간(초)")
    ap.add_argument("--ready-msg", default=None,
                    help="펌웨어 보정 완료 '#' 메시지에 포함된 문자열 (예: 'calib done')")
    ap.add_argument("--duration", type=float, default=30,
                    help="CSV 기록 시간(초, 수렴 이후부터 계산, 기본 30). 0이면 Ctrl+C 까지")
    ap.add_argument("--out", default="imu_rot.csv",
                    help="CSV 경로 (기본: imu_rot.csv)")
    ap.add_argument("--ros", action="store_true", help="/imu/yaw (Float32)로 발행")
    args = ap.parse_args()

    if args.ros:
        return run_ros(args)

    out = args.out
    f = writer = None
    n = 0
    try:
        with IMUDriver(args.port, args.baud, timeout_s=args.timeout,
                       ready_msg=args.ready_msg) as imu:
            f = open(out, "w", newline="", encoding="utf-8")
            writer = csv.writer(f)
            writer.writerow(CSV_COLUMNS)
            t0 = time.time()  # 수렴 이후부터 시간 계산
            while not args.duration or time.time() - t0 < args.duration:
                s = imu.read()
                if s is None:
                    continue
                t = round(s["pc_time_s"] - t0, 3)
                writer.writerow([t, s["yaw_deg"]])
                f.flush()
                n += 1
                print(f"t={t:7.2f} s  yaw={s['yaw_deg']:8.2f} deg", end="\r")
    except serial.SerialException as e:
        sys.exit(f"포트 열기 실패: {e}")
    except TimeoutError as e:
        sys.exit(str(e))
    except KeyboardInterrupt:
        pass
    finally:
        if f:
            f.close()
            print(f"\n[saved] {n} rows -> {out}")


if __name__ == "__main__":
    main()
