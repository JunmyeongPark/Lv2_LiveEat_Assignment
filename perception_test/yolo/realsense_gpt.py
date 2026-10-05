import queue
import statistics
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from datetime import datetime
from pathlib import Path

from ultralytics import YOLO


# 학습된 YOLO 모델 (prepare_dataset.py -> yolo detect train 결과)
MODEL_PATH = Path(__file__).parent / "runs/target_blue/weights/best.pt"
YOLO_CONF = 0.25

# YOLO 입력 크기 (높이, 너비) — I 키로 전환. Pi4 속도 개선용 크기 비교
# 32의 배수만 가능 (아니면 올림됨: 240 → 256). 4:3에 가까운 크기만 사용
YOLO_IMGSZ = [(480, 640), (256, 320), (192, 256), (128, 160)]

# "cpu": Pi처럼 CPU에서 크기별 속도 비율 비교 · None: GPU 있으면 GPU (GPU에선 크기 차이가 거의 안 보임)
YOLO_DEVICE = "cpu"

WIDTH = 640
HEIGHT = 480
FPS = 30

LOWER_BLUE = np.array([90, 80, 50])
UPPER_BLUE = np.array([130, 255, 255])

MIN_AREA = 300

# 픽셀 → 각도 변환용 RGB 화각 (deg, D435 color) — perception.yaml hfov_deg / vfov_deg와 같은 값
RGB_HFOV = 69.0
RGB_VFOV = 42.0
DEPTH_ROI_RATIO = 0.4

# 유효 깊이 범위 (m) / 최소 유효 픽셀 비율
MIN_DEPTH = 0.2
MAX_DEPTH = 3.0
MIN_VALID_RATIO = 0.5

WINDOW_NAME = "Puck Detection"
MASK_WINDOW_NAME = "Blue Mask"
ROI_WINDOW_NAME = "ROI Depth"

# ROI 깊이 창 크기
ROI_VIEW_SIZE = 400
LEGEND_HEIGHT = 70

# ROI 깊이 색상 (BGR)
COLOR_ZERO = (0, 0, 0)          # 0: 측정 실패
COLOR_TOO_NEAR = (255, 0, 255)  # MIN_DEPTH 미만
COLOR_TOO_FAR = (128, 128, 128) # MAX_DEPTH 초과


# =========================
# RealSense
# =========================

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.depth,
    WIDTH,
    HEIGHT,
    rs.format.z16,
    FPS
)

config.enable_stream(
    rs.stream.color,
    WIDTH,
    HEIGHT,
    rs.format.bgr8,
    FPS
)

pipeline.start(config)

align = rs.align(rs.stream.color)


# =========================
# 상태
# =========================

recording = False
detection_enabled = True
use_yolo = True  # M 키: YOLO <-> HSV 전환
imgsz_index = 0  # I 키: YOLO_IMGSZ 전환

# 입력 크기별 YOLO 통계 (종료 시 출력)
yolo_stats = {s: {"ms": [], "frames": 0, "conf": []} for s in YOLO_IMGSZ}

video_writer = None
record_queue = None
record_thread = None

# 녹화 포맷: WebM(VP8). VP9는 실시간 인코딩에 너무 느림 (프레임당 ~300ms)
RECORD_EXT = "webm"
RECORD_FOURCC = "VP80"


# =========================
# 검출
# =========================

model = YOLO(str(MODEL_PATH))

# 크기별 워밍업 (첫 추론은 느려서 통계가 왜곡됨)
for size in YOLO_IMGSZ:
    model.predict(np.zeros((HEIGHT, WIDTH, 3), np.uint8), imgsz=size, device=YOLO_DEVICE, verbose=False)


def detect_yolo(frame):
    """YOLO로 가장 신뢰도 높은 박스 1개 반환 -> ((x1, y1, x2, y2), conf) 또는 (None, None)"""

    size = YOLO_IMGSZ[imgsz_index]

    t0 = time.perf_counter()
    boxes = model.predict(frame, imgsz=size, conf=YOLO_CONF, device=YOLO_DEVICE, verbose=False)[0].boxes
    elapsed_ms = (time.perf_counter() - t0) * 1000  # 전처리 + 추론 + 후처리

    stats = yolo_stats[size]
    stats["ms"].append(elapsed_ms)
    stats["frames"] += 1

    if len(boxes) == 0:
        return None, None

    best = int(boxes.conf.argmax()) # 가장 높은 확률의 인덱스 가져오기
    x1, y1, x2, y2 = boxes.xyxy[best].int().tolist()
    conf = float(boxes.conf[best])
    stats["conf"].append(conf)

    return (x1, y1, x2, y2), conf


def print_yolo_stats():
    """입력 크기별 처리 시간·검출률 출력 (PC 속도라 Pi와 절대값은 다름, 크기 간 비율을 봄)"""

    print(f"\n{'imgsz':>9} {'frames':>7} {'mean ms':>8} {'p50 ms':>7} {'FPS':>6} {'detect':>7} {'conf':>6}")

    for (h, w), stats in yolo_stats.items():
        if stats["frames"] == 0:
            continue

        mean_ms = statistics.mean(stats["ms"])
        detect_ratio = len(stats["conf"]) / stats["frames"]
        conf = f"{statistics.mean(stats['conf']):.3f}" if stats["conf"] else "-"

        print(f"{w:>4}x{h:<4} {stats['frames']:7d} {mean_ms:8.1f} {statistics.median(stats['ms']):7.1f} "
              f"{1000 / mean_ms:6.1f} {detect_ratio:7.1%} {conf:>6}")


def detect_hsv(frame):
    """HSV 색으로 가장 큰 파란 영역 반환 -> ((x1, y1, x2, y2), contour, mask)"""

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    # 노이즈제거를 위한 커널 생성(HSV검출 안정화)
    kernel = np.ones((5, 5), np.uint8)
    # 오프닝 연산으로 작은 점 노이즈 제거
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # 클로징 연산으로 구멍 노이즈 제거
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 마스크에서 외곽선 찾기
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 최소 영역보다 큰 박스만
    candidates = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]

    if not candidates:
        return None, None, mask

    # 제일 큰 박스만
    target = max(candidates, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(target)

    return (x, y, x + w, y + h), target, mask


# =========================
# 녹화
# =========================

def record_worker(writer, frame_queue):
    """VP8 인코딩(프레임당 ~25ms)이 메인 루프를 막지 않도록 별도 스레드에서 저장"""

    while True:
        frame = frame_queue.get()

        if frame is None:
            break

        writer.write(frame)

    writer.release()


def start_recording():
    global video_writer, record_queue, record_thread

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"record_{timestamp}.{RECORD_EXT}"

    fourcc = cv2.VideoWriter_fourcc(*RECORD_FOURCC)

    video_writer = cv2.VideoWriter(
        filename,
        fourcc,
        FPS,
        (WIDTH, HEIGHT)
    )

    # 인코딩이 밀리면 최대 2초 분량까지 버퍼링
    record_queue = queue.Queue(maxsize=FPS * 2)
    record_thread = threading.Thread(
        target=record_worker,
        args=(video_writer, record_queue),
        daemon=True
    )
    record_thread.start()

    print(f"[REC START] {filename}")


def stop_recording():
    global video_writer, record_queue, record_thread

    if record_thread is not None:
        # 남은 프레임을 모두 저장한 뒤 파일을 닫음
        record_queue.put(None)
        record_thread.join()

    video_writer = None
    record_queue = None
    record_thread = None

    print("[REC STOP]")


# =========================
# Depth
# =========================

def get_median_depth(depth_frame, x1, y1, x2, y2):
    """ROI 깊이 중앙값 (m) -> (depth, stats). 범위 밖·유효 픽셀 부족이면 0.0 (perception 노드 z=0과 같음)"""

    box_width = x2 - x1
    box_height = y2 - y1

    roi_width = int(box_width * DEPTH_ROI_RATIO)
    roi_height = int(box_height * DEPTH_ROI_RATIO)

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2

    rx1 = max(0, cx - roi_width // 2)
    rx2 = min(WIDTH, cx + roi_width // 2)

    ry1 = max(0, cy - roi_height // 2)
    ry2 = min(HEIGHT, cy + roi_height // 2)

    depth_image = np.asanyarray(depth_frame.get_data())

    # raw(z16) -> 미터 단위
    roi = depth_image[ry1:ry2, rx1:rx2] * depth_frame.get_units()

    if roi.size == 0:
        return 0.0, None

    # 0(측정 실패) 및 유효 범위 밖 값 제거
    valid = roi[(roi >= MIN_DEPTH) & (roi <= MAX_DEPTH)]
    nonzero = roi[roi > 0]

    # 디버그용 통계
    stats = {
        "roi": (rx1, ry1, rx2, ry2),
        "roi_depth": roi,
        "roi_size": roi.size,
        "zero_ratio": 1.0 - nonzero.size / roi.size,
        "valid_ratio": valid.size / roi.size,
        # 필터 없이 계산했을 때의 값 (이전 방식)
        "raw_median": float(np.median(nonzero)) if nonzero.size > 0 else None,
    }

    # 유효 픽셀이 너무 적으면 신뢰 불가 (너무 가까움 / 무늬 없는 면 등)
    if valid.size < roi.size * MIN_VALID_RATIO:
        return 0.0, stats

    return float(np.median(valid)), stats


def make_roi_view(roi):
    """ROI 깊이를 픽셀별 색으로 표시 (유효 범위: 가까움=빨강 ~ 멀리=파랑)"""

    # 배경은 0(검정)과 구분되도록 어두운 회색
    view = np.full(
        (ROI_VIEW_SIZE + LEGEND_HEIGHT, ROI_VIEW_SIZE, 3),
        40,
        dtype=np.uint8
    )

    if roi is not None and roi.size > 0:

        # 유효 범위 -> JET 컬러맵 (가까울수록 빨강)
        norm = (roi - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH)
        norm = np.clip(norm, 0.0, 1.0)

        colored = cv2.applyColorMap(
            (255 - norm * 255).astype(np.uint8),
            cv2.COLORMAP_JET
        )

        colored[roi == 0] = COLOR_ZERO
        colored[(roi > 0) & (roi < MIN_DEPTH)] = COLOR_TOO_NEAR
        colored[roi > MAX_DEPTH] = COLOR_TOO_FAR

        # 비율 유지하며 확대 (픽셀 경계가 보이도록 NEAREST)
        h, w = roi.shape
        scale = ROI_VIEW_SIZE / max(h, w)
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))

        colored = cv2.resize(
            colored,
            (new_w, new_h),
            interpolation=cv2.INTER_NEAREST
        )

        ox = (ROI_VIEW_SIZE - new_w) // 2
        oy = (ROI_VIEW_SIZE - new_h) // 2
        view[oy:oy + new_h, ox:ox + new_w] = colored

    else:
        cv2.putText(
            view,
            "No target",
            (ROI_VIEW_SIZE // 2 - 60, ROI_VIEW_SIZE // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

    # =========================
    # 범례
    # =========================

    top = ROI_VIEW_SIZE + 5

    # 컬러바 (MIN_DEPTH ~ MAX_DEPTH)
    bar = np.linspace(255, 0, ROI_VIEW_SIZE - 20).astype(np.uint8)
    bar = cv2.applyColorMap(
        np.tile(bar, (12, 1)),
        cv2.COLORMAP_JET
    )
    view[top:top + 12, 10:ROI_VIEW_SIZE - 10] = bar

    cv2.putText(view, f"{MIN_DEPTH:.1f}m", (10, top + 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(view, f"{MAX_DEPTH:.1f}m", (ROI_VIEW_SIZE - 45, top + 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # 무효 값 색상
    legend = [
        (COLOR_ZERO, "0 (fail)"),
        (COLOR_TOO_NEAR, f"<{MIN_DEPTH:.1f}m"),
        (COLOR_TOO_FAR, f">{MAX_DEPTH:.1f}m"),
    ]

    for i, (color, label) in enumerate(legend):
        lx = 10 + i * 130
        ly = top + 40
        cv2.rectangle(view, (lx, ly), (lx + 14, ly + 14), color, -1)
        cv2.rectangle(view, (lx, ly), (lx + 14, ly + 14), (255, 255, 255), 1)
        cv2.putText(view, label, (lx + 20, ly + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    return view


# =========================
# Window
# =========================

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.namedWindow(MASK_WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.namedWindow(ROI_WINDOW_NAME, cv2.WINDOW_NORMAL)

# ROI 깊이 창을 메인 창 오른쪽에 배치
cv2.moveWindow(WINDOW_NAME, 0, 0)
cv2.moveWindow(ROI_WINDOW_NAME, WIDTH + 20, 0)


try:

    while True:

        frames = pipeline.wait_for_frames()
        # 뎁스카메라와 RGB카메라 시점 정렬
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # 원본 RGB 프레임
        raw_frame = np.asanyarray(
            color_frame.get_data()
        )

        # 화면 표시용 복사본
        display_frame = raw_frame.copy()

        mask = np.zeros(
            (HEIGHT, WIDTH),
            dtype=np.uint8
        )

        roi_depth = None

        # =========================
        # 객체 검출 ON
        # =========================

        if detection_enabled:

            target = None  # HSV일 때만 contour 존재
            conf = None    # YOLO일 때만 신뢰도 존재

            if use_yolo:
                box, conf = detect_yolo(raw_frame)
            else:
                box, target, mask = detect_hsv(raw_frame)

            if box is not None:

                x1, y1, x2, y2 = box

                w = x2 - x1
                h = y2 - y1

                cx = x1 + w // 2
                cy = y1 + h // 2

                depth, stats = get_median_depth(
                    depth_frame,
                    x1,
                    y1,
                    x2,
                    y2
                )

                # Bounding Box
                cv2.rectangle(
                    display_frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                # Contour (HSV 모드)
                if target is not None:
                    cv2.drawContours(
                        display_frame,
                        [target],
                        -1,
                        (255, 0, 0),
                        2
                    )

                # 중심점
                cv2.circle(
                    display_frame,
                    (cx, cy),
                    5,
                    (0, 0, 255),
                    -1
                )

                # 화면 중앙선
                cv2.line(
                    display_frame,
                    (WIDTH // 2, 0),
                    (WIDTH // 2, HEIGHT),
                    (0, 255, 255),
                    1
                )

                if depth > 0:
                    text = f"Depth: {depth:.3f} m"
                else:
                    text = f"Depth: 0 (invalid, {MIN_DEPTH:.1f}~{MAX_DEPTH:.1f} m)"

                cv2.putText(
                    display_frame,
                    text,
                    (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                # =========================
                # 깊이 디버그 정보
                # =========================

                # 픽셀 -> 각도: 좌우 = (HFOV/2)*(w/2 - x)/(w/2), 상하 = (VFOV/2)*(h/2 - y)/(h/2)
                # 왼쪽·위가 + (ex·ey와 부호 반대)
                angle_x = (RGB_HFOV / 2) * (WIDTH / 2 - cx) / (WIDTH / 2)
                angle_y = (RGB_VFOV / 2) * (HEIGHT / 2 - cy) / (HEIGHT / 2)

                debug_lines = [
                    f"Box: {w}x{h}",
                    f"Angle X: {angle_x:+.1f} deg (L+)",
                    f"Angle Y: {angle_y:+.1f} deg (U+)",
                ]

                if conf is not None:
                    debug_lines.append(f"Conf: {conf:.2f}")

                if stats is not None:

                    rx1, ry1, rx2, ry2 = stats["roi"]

                    # 깊이 계산에 사용한 ROI
                    cv2.rectangle(
                        display_frame,
                        (rx1, ry1),
                        (rx2, ry2),
                        (0, 255, 255),
                        1
                    )

                    roi_depth = stats["roi_depth"]
                    raw_median = stats["raw_median"]

                    debug_lines += [
                        f"ROI: {rx2 - rx1}x{ry2 - ry1} ({stats['roi_size']} px)",
                        f"Zero: {stats['zero_ratio']:.0%}",
                        f"Valid: {stats['valid_ratio']:.0%} (min {MIN_VALID_RATIO:.0%})",
                        "Raw median: "
                        + (f"{raw_median:.3f} m" if raw_median is not None else "N/A"),
                    ]

                # 우측 상단 정보 패널
                panel_x = WIDTH - 230

                cv2.rectangle(
                    display_frame,
                    (panel_x - 5, 10),
                    (WIDTH - 5, 15 + 20 * len(debug_lines)),
                    (0, 0, 0),
                    -1
                )

                for i, line in enumerate(debug_lines):
                    cv2.putText(
                        display_frame,
                        line,
                        (panel_x, 28 + 20 * i),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (255, 255, 255),
                        1
                    )

        # =========================
        # 상태 표시
        # =========================

        if detection_enabled and use_yolo:
            h_in, w_in = YOLO_IMGSZ[imgsz_index]
            last_ms = yolo_stats[(h_in, w_in)]["ms"][-1]
            detection_text = f"DETECTION: ON (YOLO {w_in}x{h_in}) {last_ms:.0f} ms ({1000 / last_ms:.1f} FPS)"
        elif detection_enabled:
            detection_text = "DETECTION: ON (HSV)"
        else:
            detection_text = "DETECTION: OFF"

        cv2.putText(
            display_frame,
            detection_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0)
            if detection_enabled
            else (0, 0, 255),
            2
        )

        if recording:

            cv2.circle(
                display_frame,
                (25, 55),
                8,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                display_frame,
                "REC",
                (40, 62),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.putText(
            display_frame,
            "D: Detection | M: YOLO/HSV | I: YOLO size | R: Record | Q: Quit",
            (10, HEIGHT - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1
        )


        # =========================
        # 녹화
        # =========================

        if recording and record_queue is not None:

            # 객체 검출 OFF이면
            # YOLO 학습용 깨끗한 원본 영상 저장
            # raw_frame은 RealSense 버퍼를 참조하므로 복사해서 넘김
            if detection_enabled:
                record_queue.put(display_frame)
            else:
                record_queue.put(raw_frame.copy())


        # =========================
        # 출력
        # =========================

        cv2.imshow(
            WINDOW_NAME,
            display_frame
        )

        cv2.imshow(
            MASK_WINDOW_NAME,
            mask
        )

        cv2.imshow(
            ROI_WINDOW_NAME,
            make_roi_view(roi_depth)
        )

        key = cv2.waitKey(1) & 0xFF


        # =========================
        # 키 입력
        # =========================

        if key == ord("q"):
            break

        elif key == ord("r"):

            recording = not recording

            if recording:
                start_recording()
            else:
                stop_recording()

        elif key == ord("m"):

            use_yolo = not use_yolo

            print("[MODE]", "YOLO" if use_yolo else "HSV")

        elif key == ord("i"):

            imgsz_index = (imgsz_index + 1) % len(YOLO_IMGSZ)

            h_in, w_in = YOLO_IMGSZ[imgsz_index]
            print(f"[YOLO SIZE] {w_in}x{h_in}")

        elif key == ord("d"):

            detection_enabled = not detection_enabled

            print(
                "[DETECTION]",
                "ON"
                if detection_enabled
                else "OFF"
            )


        # =========================
        # X 버튼
        # =========================

        if cv2.getWindowProperty(
            WINDOW_NAME,
            cv2.WND_PROP_VISIBLE
        ) < 1:
            break

        if cv2.getWindowProperty(
            MASK_WINDOW_NAME,
            cv2.WND_PROP_VISIBLE
        ) < 1:
            break

        if cv2.getWindowProperty(
            ROI_WINDOW_NAME,
            cv2.WND_PROP_VISIBLE
        ) < 1:
            break


finally:

    if recording:
        stop_recording()

    pipeline.stop()

    cv2.destroyAllWindows()

    print_yolo_stats()