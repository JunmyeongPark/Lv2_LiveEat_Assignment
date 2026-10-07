"""RealSense + 퍽 검출 — 최신 설정만 (Pi ROS 인지 노드와 같은 조건)

  카메라  color 320x180 · depth 424x240 · 30 FPS · depth를 color에 정렬
          (launch/perception.launch.py와 같음. color 320x180은 15 FPS를 지원하지 않음)
  검출    YOLO v4 (16:9 학습), 입력 320x192 (320x180 + 위아래 6px 패딩), conf 0.5
  depth   bbox 중앙 40% ROI의 유효 깊이 중앙값 (0.2~3.0 m, 유효 50% 미만이면 0)
          bbox가 화면 왼쪽 끝에 닿으면 0 (depth 사각지대에서 배경 깊이를 읽음)
          박스 크기가 depth에 안 맞으면 0 (추정 실제 크기가 1.5~10 cm 밖, perception.yaml size_check)

지난 시험 기능(모델·입력 크기·해상도 전환, HSV, 형상 검사, 크기 표시, 스냅샷 등)은 realsense_test.py

키: D 검출 ON/OFF · R 녹화 (검출 OFF면 학습용 원본 320x180) · Q 종료
"""
import queue
import statistics
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

MODEL = Path(__file__).parent / "runs/target_blue_v4_169/weights/best.pt"  # Pi 배포 모델 target_blue_v4_192의 원본
YOLO_IMGSZ = (192, 320)  # (높이, 너비). 180은 32의 배수가 아니라 192
YOLO_CONF = 0.5
YOLO_DEVICE = "cpu"      # Pi처럼 CPU

WIDTH, HEIGHT = 320, 180
DEPTH_WIDTH, DEPTH_HEIGHT = 424, 240
FPS = 30

# 픽셀 → 각도 변환용 RGB 화각 (deg, D435 color 16:9 실측) — perception.yaml hfov_deg / vfov_deg와 같은 값
RGB_HFOV = 70.1
RGB_VFOV = 43.1

DEPTH_ROI_RATIO = 0.4
MIN_DEPTH = 0.2
MAX_DEPTH = 3.0
MIN_VALID_RATIO = 0.5
LEFT_EDGE_PX = 2  # bbox 왼쪽 x1이 이 px 이하면 화면 왼쪽 끝에 닿은 것으로 봄

# 박스 크기 ↔ depth 검사 (perception.yaml size_min_m / size_max_m와 같은 값)
# 추정 실제 크기 = 박스 긴 변[px] × z / f[px]. 퍽 30x30x60 mm → 보이는 긴 변 30~67 mm, 박스·depth 오차 포함해 넓게
SIZE_MIN_M = 0.015
SIZE_MAX_M = 0.10
FX = (WIDTH / 2) / np.tan(np.radians(RGB_HFOV / 2))   # ≈ 228 (실측 228.0)
FY = (HEIGHT / 2) / np.tan(np.radians(RGB_VFOV / 2))

DISPLAY_SCALE = 2  # 320x180은 글자가 안 들어가서 화면만 2배로 그림
WINDOW_NAME = "Puck Detection"

# 녹화 포맷: WebM(VP8). VP9는 실시간 인코딩에 너무 느림
RECORD_EXT = "webm"
RECORD_FOURCC = "VP80"


def sp(*v):
    """원본 좌표 → 화면 좌표"""
    return tuple(int(x * DISPLAY_SCALE) for x in v)


def detect(model, frame):
    """가장 신뢰도 높은 박스 1개 -> ((x1, y1, x2, y2), conf) 또는 (None, None)"""
    boxes = model.predict(frame, imgsz=YOLO_IMGSZ, conf=YOLO_CONF, device=YOLO_DEVICE, verbose=False)[0].boxes
    if len(boxes) == 0:
        return None, None
    best = int(boxes.conf.argmax())
    return tuple(boxes.xyxy[best].int().tolist()), float(boxes.conf[best])


def estimate_size(x1, y1, x2, y2, z):
    """박스 긴 변의 실제 크기 (m). 퍽이 돌아가도 긴 변은 크게 안 변함"""
    return max((x2 - x1) * z / FX, (y2 - y1) * z / FY)


def size_rejected(box, size):
    """너무 큼 = 뒤 배경 depth / 큰 파란 물체, 너무 작음 = 앞을 가린 물체의 depth. 화면 끝에 닿아 잘린 박스는 상한만"""
    x1, y1, x2, y2 = box
    at_edge = x1 <= 1 or y1 <= 1 or x2 >= WIDTH - 2 or y2 >= HEIGHT - 2
    return size > SIZE_MAX_M or (not at_edge and size < SIZE_MIN_M)


def median_depth(depth_frame, x1, y1, x2, y2):
    """bbox 중앙 ROI의 깊이 중앙값 (m) -> (depth, roi). 범위 밖·유효 픽셀 부족이면 0.0 (perception 노드 z=0과 같음)"""
    depth_image = np.asanyarray(depth_frame.get_data())
    img_h, img_w = depth_image.shape

    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    rw, rh = int((x2 - x1) * DEPTH_ROI_RATIO), int((y2 - y1) * DEPTH_ROI_RATIO)
    rx1, rx2 = max(0, cx - rw // 2), min(img_w, cx + rw // 2)
    ry1, ry2 = max(0, cy - rh // 2), min(img_h, cy + rh // 2)

    roi = depth_image[ry1:ry2, rx1:rx2] * depth_frame.get_units()
    if roi.size == 0:
        return 0.0, None

    valid = roi[(roi >= MIN_DEPTH) & (roi <= MAX_DEPTH)]
    if valid.size < roi.size * MIN_VALID_RATIO:
        return 0.0, (rx1, ry1, rx2, ry2)
    return float(np.median(valid)), (rx1, ry1, rx2, ry2)


class Recorder:
    """VP8 인코딩이 메인 루프를 막지 않도록 별도 스레드에서 저장"""

    def __init__(self, size):
        self.size = size
        filename = f"record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.{RECORD_EXT}"
        self.writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*RECORD_FOURCC), FPS, size)
        self.queue = queue.Queue(maxsize=FPS * 2)  # 인코딩이 밀리면 최대 2초 분량까지 버퍼링
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print(f"[REC START] {filename} {size[0]}x{size[1]}")

    def _run(self):
        while (frame := self.queue.get()) is not None:
            self.writer.write(frame)
        self.writer.release()

    def put(self, frame):
        if (frame.shape[1], frame.shape[0]) != self.size:
            frame = cv2.resize(frame, self.size)
        self.queue.put(frame)

    def stop(self):
        self.queue.put(None)  # 남은 프레임을 모두 저장한 뒤 파일을 닫음
        self.thread.join()
        print("[REC STOP]")


def draw_text(img, text, org, scale=0.5, color=(255, 255, 255), thickness=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)


def main():
    model = YOLO(str(MODEL))
    model.predict(np.zeros((HEIGHT, WIDTH, 3), np.uint8), imgsz=YOLO_IMGSZ, device=YOLO_DEVICE, verbose=False)  # 워밍업

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    detection_enabled = True
    recorder = None
    infer_ms, loop_ms, detected = [], [], 0
    last_t = None

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            t = time.perf_counter()
            if last_t is not None:
                loop_ms.append((t - last_t) * 1000)
            last_t = t

            raw = np.asanyarray(color_frame.get_data())
            view = cv2.resize(raw, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE, interpolation=cv2.INTER_LINEAR)

            if detection_enabled:
                t0 = time.perf_counter()
                box, conf = detect(model, raw)
                infer_ms.append((time.perf_counter() - t0) * 1000)

                if box is not None:
                    detected += 1
                    x1, y1, x2, y2 = box
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    depth, roi = median_depth(depth_frame, *box)

                    # 화면 왼쪽 끝에 닿으면 depth 사각지대로 보고 0 처리
                    left_edge = x1 <= LEFT_EDGE_PX
                    if left_edge:
                        depth = 0.0

                    size = estimate_size(*box, depth) if depth > 0 else None
                    size_bad = size is not None and size_rejected(box, size)
                    if size_bad:
                        depth = 0.0

                    # 픽셀 -> 각도: 왼쪽·위가 + (ex·ey와 부호 반대)
                    angle_x = (RGB_HFOV / 2) * (WIDTH / 2 - cx) / (WIDTH / 2)
                    angle_y = (RGB_VFOV / 2) * (HEIGHT / 2 - cy) / (HEIGHT / 2)

                    cv2.rectangle(view, sp(x1, y1), sp(x2, y2), (0, 255, 0), 2)
                    cv2.circle(view, sp(cx, cy), 4, (0, 0, 255), -1)
                    if roi is not None:
                        cv2.rectangle(view, sp(roi[0], roi[1]), sp(roi[2], roi[3]), (0, 255, 255), 1)

                    if depth > 0:
                        text = f"{depth:.3f} m"
                    elif left_edge:
                        text = "0 (left edge)"
                    elif size_bad:
                        text = f"0 (size {size * 100:.1f} cm)"
                    else:
                        text = "0 (invalid)"
                    draw_text(view, f"{text}  conf {conf:.2f}", (x1 * DISPLAY_SCALE, max(20, y1 * DISPLAY_SCALE - 8)),
                              0.55, (0, 255, 0), 2)
                    size_text = f"  size {size * 100:.1f} cm" if size is not None else ""
                    draw_text(view, f"angle X {angle_x:+.1f}  Y {angle_y:+.1f} deg  box {x2 - x1}x{y2 - y1}{size_text}",
                              (10, 50))

                last = infer_ms[-1]
                status = f"YOLO v4 {YOLO_IMGSZ[1]}x{YOLO_IMGSZ[0]}  {last:.0f} ms ({1000 / last:.1f} FPS)"
                draw_text(view, status, (10, 25), 0.6, (0, 255, 0), 2)
            else:
                draw_text(view, "DETECTION: OFF", (10, 25), 0.6, (0, 0, 255), 2)

            if recorder is not None:
                cv2.circle(view, (view.shape[1] - 60, 20), 7, (0, 0, 255), -1)
                draw_text(view, "REC", (view.shape[1] - 48, 27), 0.6, (0, 0, 255), 2)
                # 검출 OFF면 학습용 깨끗한 원본 (raw는 RealSense 버퍼를 참조하므로 복사)
                recorder.put(view if detection_enabled else raw.copy())

            draw_text(view, f"CAM {WIDTH}x{HEIGHT}@{FPS} | D: Detect | R: Rec | Q: Quit",
                      (10, view.shape[0] - 12), 0.45)
            cv2.imshow(WINDOW_NAME, view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            elif key == ord("d"):
                detection_enabled = not detection_enabled
                print("[DETECTION]", "ON" if detection_enabled else "OFF")
            elif key == ord("r"):
                if recorder is None:
                    # 검출 OFF로 시작하면 원본 해상도, ON이면 화면 크기
                    recorder = Recorder(view.shape[1::-1] if detection_enabled else (WIDTH, HEIGHT))
                else:
                    recorder.stop()
                    recorder = None

    finally:
        if recorder is not None:
            recorder.stop()
        pipeline.stop()
        cv2.destroyAllWindows()

        if infer_ms:
            mean = statistics.mean(infer_ms)
            print(f"\nYOLO v4 {YOLO_IMGSZ[1]}x{YOLO_IMGSZ[0]}: {len(infer_ms)} frames, "
                  f"mean {mean:.1f} ms (p50 {statistics.median(infer_ms):.1f}), {1000 / mean:.1f} FPS, "
                  f"detect {detected / len(infer_ms):.1%}")
        if loop_ms:
            print(f"loop {statistics.mean(loop_ms):.1f} ms ({1000 / statistics.mean(loop_ms):.1f} FPS)")


if __name__ == "__main__":
    main()
