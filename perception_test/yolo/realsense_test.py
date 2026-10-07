"""RealSense + 퍽 검출 시험 도구 — 지금까지 시험한 기능 전부

최신 설정(Pi와 같은 320x180 · v4 · 왼쪽 끝 depth 거부)만 쓰려면 realsense.py.
depth는 항상 color에 정렬(rs.align, Pi와 같음).
여기는 비교·분석용: 해상도(--res), 모델(V), YOLO 입력 크기(I), YOLO/HSV(M), 박스 크기 검사(Z),
depth 거부 방식(E: 왼쪽 끝 / 형상 검사 / 끔), 형상 검사 설정(F, 1~9), 스냅샷(S), 녹화(R)
"""
import argparse
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


# 학습된 YOLO 모델 (prepare_dataset.py -> yolo detect train 결과) — V 키 또는 --model 로 전환
MODELS = {
    "v1": Path(__file__).parent / "runs/target_blue/weights/best.pt",
    "v2": Path(__file__).parent / "runs/target_blue_v2/weights/best.pt",
    "v3": Path(__file__).parent / "runs/target_blue_v3/weights/best.pt",
    "v4": Path(__file__).parent / "runs/target_blue_v4_169/weights/best.pt",  # v3 데이터를 16:9로 변환, imgsz=320 (Pi 배포 모델)
}

# 입력 크기 전용 모델: (모델, (높이, 너비)) -> 그 크기(imgsz=너비)로 학습한 가중치
# 여기 없는 조합은 MODELS의 640 학습 모델을 그대로 사용
# 키의 크기는 4:3·16:9 입력 크기 둘 다 적어 둠 (같은 가중치, imgsz=너비로 학습)
_V3_320 = Path(__file__).parent / "runs/target_blue_v3_imgsz320/weights/best.pt"  # 4:3 Pi 배포 모델(v3)과 같은 가중치
_V3_256 = Path(__file__).parent / "runs/target_blue_v3_imgsz256/weights/best.pt"
_V3_160 = Path(__file__).parent / "runs/target_blue_v3_imgsz160/weights/best.pt"
SIZE_MODELS = {
    ("v3", (256, 320)): _V3_320, ("v3", (192, 320)): _V3_320,
    ("v3", (192, 256)): _V3_256, ("v3", (160, 256)): _V3_256,
    ("v3", (128, 160)): _V3_160, ("v3", (96, 160)): _V3_160,
}
YOLO_CONF = 0.5

# YOLO 입력 크기 (높이, 너비) — I 키로 전환. Pi4 속도 개선용 크기 비교
# 32의 배수만 가능 (아니면 올림됨: 240 → 256, 180 → 192). 카메라 비율에 가까운 크기만 사용, 카메라보다 큰 크기는 뺌
YOLO_IMGSZ_BY_ASPECT = {
    "16:9": [(384, 640), (192, 320), (160, 256), (96, 160)],  # 320x180 → 192 (Pi 배포 v4: 320x192)
    "4:3": [(480, 640), (256, 320), (192, 256), (128, 160)],
}

# "cpu": Pi처럼 CPU에서 크기별 속도 비율 비교 · None: GPU 있으면 GPU (GPU에선 크기 차이가 거의 안 보임)
YOLO_DEVICE = "cpu"

# 카메라 해상도·화각 (--res). Pi ROS 설정(launch/perception.launch.py)은 320x180
# D435 실측: color 640x480은 16:9 센서 가운데를 자른 것(fx 608, HFOV 55.5°), 16:9 해상도는 센서 전체(HFOV 70.1°)
# depth 해상도: 낮을수록 최소 측정 거리(min-Z)가 짧아짐 (640x480 ≈ 17~20 cm → 424x240 ≈ 10 cm)
# rs.align(color)가 color 해상도(WIDTH x HEIGHT)로 맞춰 주므로 ROI 계산은 그대로
# res: (WIDTH, HEIGHT, DEPTH_WIDTH, DEPTH_HEIGHT, RGB_HFOV, RGB_VFOV, 비율, color 지원 FPS)
CAMERA_BY_RES = {
    "320x180": (320, 180, 424, 240, 70.1, 43.1, "16:9", (6, 30, 60)),  # Pi 설정. color 320x180은 15 FPS 없음
    "640x360": (640, 360, 640, 360, 70.1, 43.1, "16:9", (6, 15, 30, 60)),
    "640x480": (640, 480, 640, 480, 55.5, 43.1, "4:3", (6, 15, 30, 60)),  # v1~v3 학습 데이터
}
FPS = 30

LOWER_BLUE = np.array([90, 80, 50])
UPPER_BLUE = np.array([130, 255, 255])

MIN_AREA = 300  # [px²] 640 너비 기준 (다른 해상도면 면적 비율대로 줄임)

DEPTH_ROI_RATIO = 0.4

# 유효 깊이 범위 (m) / 최소 유효 픽셀 비율
MIN_DEPTH = 0.2
MAX_DEPTH = 3.0
MIN_VALID_RATIO = 0.5

# 퍽이 너무 가까우면 depth 사각지대(왼쪽 무효 띠)에 걸려 align 시 뒤 배경 깊이가 bbox를 채움
# → bbox가 화면 왼쪽 끝에 닿으면 depth를 믿지 않고 0 처리
LEFT_EDGE_PX = 2        # bbox 왼쪽 x1이 이 px 이하면 화면 왼쪽 끝에 닿은 것으로 봄

# depth 거부 방식 (E 키로 전환). "edge"가 최종 채택 (realsense.py · Pi와 같음)
#   edge:  bbox가 화면 왼쪽 끝에 닿으면 0
#   shape: 깊이 형상 검사 (이전 시도, 아래 SHAPE_*) — 점수가 낮으면 0
#   off:   거부 안 함
DEPTH_REJECT_MODES = ["edge", "shape", "off"]

# 박스 크기 ↔ depth 검사 (Z 키로 ON/OFF, 위 거부 방식과 따로 적용) — perception.yaml size_min_m / size_max_m와 같은 값
# 추정 실제 크기 = 박스 긴 변[px] × z / f[px]. 퍽 30x30x60 mm → 보이는 긴 변 30~67 mm, 박스·depth 오차 포함해 넓게
# 너무 큼 = 뒤 배경 depth / 큰 파란 물체, 너무 작음 = 앞을 가린 물체의 depth. 화면 끝에 닿아 잘린 박스는 상한만
SIZE_MIN_M = 0.015
SIZE_MAX_M = 0.10

# 기대 크기: 퍽 30x30x60 mm가 화면에 보이는 긴 변 — 정면(30 mm) ~ 대각선(√(30²+60²) ≈ 67 mm)
# 화면에 "Size 실측 cm (기대 3.0~6.7)", "Box px (기대 px)", 박스 중심에 기대 크기 사각형(보라)으로 표시
PUCK_MIN_M = 0.030
PUCK_MAX_M = 0.067

# 깊이 형상 검사: 기준 깊이(bbox 최빈값) ±tol 픽셀이 bbox 주위에 얼마나 없는지(empty)
# F 키로 bbox 안쪽이 얼마나 차 있는지(fill)를 점수에 곱할지 전환
# 퍽이 사각지대에 들어오면 align 시 뒤 배경 깊이가 bbox 안을 채움 → 같은 깊이가 주위까지 퍼져 empty가 낮아짐
# px 값은 640 너비 기준 (다른 해상도면 비율대로 줄임)
SHAPE_X_RATIO = 0.25    # bbox 중심이 화면 왼쪽 이 비율 안에 있고
SHAPE_EDGE_PX = 2       # bbox 외곽이 화면 끝(상하좌우)에서 이 px 안에 하나라도 붙어 있을 때만 검사
SHAPE_BIN = 0.02        # [m] 기준 깊이(bbox 깊이 최빈값) 히스토그램 구간 폭
SHAPE_MARGIN_PX = 150   # [px] bbox를 각 방향으로 이만큼 넓혀 주위 영역으로 봄
SHAPE_TOL = 0.02        # [m] 기준 깊이 ± 허용오차 초기값 (실행 중 1~9 키로 1~9 cm 변경)
SHAPE_MIN_SCORE = 0.5   # score = empty 또는 fill x empty (0~1). 이보다 낮으면 depth 거부 (0 처리)

WINDOW_NAME = "Puck Detection"
MASK_WINDOW_NAME = "Blue Mask"
ROI_WINDOW_NAME = "ROI Depth"
SHAPE_WINDOW_NAME = "Depth Shape"

# ROI 깊이 창 크기
ROI_VIEW_SIZE = 400
LEGEND_HEIGHT = 70

# ROI 깊이 색상 (BGR)
COLOR_ZERO = (0, 0, 0)          # 0: 측정 실패
COLOR_TOO_NEAR = (255, 0, 255)  # MIN_DEPTH 미만
COLOR_TOO_FAR = (128, 128, 128) # MAX_DEPTH 초과


parser = argparse.ArgumentParser()
parser.add_argument("--model", choices=list(MODELS), default="v4", help="시작 모델 (실행 중 V 키로 전환)")
parser.add_argument("--res", choices=list(CAMERA_BY_RES), default="320x180",
                    help="color 해상도: 320x180 (Pi 설정) · 640x360 (같은 화각, 2배 화질) · 640x480 (4:3, v1~v3 학습 데이터)")
parser.add_argument("--fps", type=int, choices=[6, 15, 30, 60], default=FPS,
                    help="카메라 FPS (Pi ROS 설정은 30, 320x180은 15 불가). 추론이 느리면 처리 FPS는 같고 지연만 달라짐")
parser.add_argument("--reject", choices=DEPTH_REJECT_MODES, default="edge",
                    help="depth 거부 방식 시작값 (실행 중 E 키로 전환)")
parser.add_argument("--no-size-check", action="store_true",
                    help="박스 크기 ↔ depth 검사 끄고 시작 (실행 중 Z 키로 전환)")
args = parser.parse_args()
FPS = args.fps
# 픽셀 → 각도 변환용 RGB 화각 (deg, D435 color 실측) — perception.yaml hfov_deg / vfov_deg와 같은 값 (16:9)
WIDTH, HEIGHT, DEPTH_WIDTH, DEPTH_HEIGHT, RGB_HFOV, RGB_VFOV, ASPECT, COLOR_FPS = CAMERA_BY_RES[args.res]
if FPS not in COLOR_FPS:
    parser.error(f"color {args.res}는 {COLOR_FPS} FPS만 지원")
YOLO_IMGSZ = [s for s in YOLO_IMGSZ_BY_ASPECT[ASPECT] if s[1] <= WIDTH]

# 화면 표시는 640 너비로 키워서 그림 (320x180이면 글자가 안 들어감). 검출·depth 좌표는 원본 해상도 그대로
FX = (WIDTH / 2) / np.tan(np.radians(RGB_HFOV / 2))   # 크기 검사용 초점거리 [px] (color 기준)
FY = (HEIGHT / 2) / np.tan(np.radians(RGB_VFOV / 2))

DISPLAY_SCALE = max(1, 640 // WIDTH)
DISP_W, DISP_H = WIDTH * DISPLAY_SCALE, HEIGHT * DISPLAY_SCALE
SHAPE_MARGIN = SHAPE_MARGIN_PX * WIDTH // 640


def sp(*v):
    """원본 좌표 → 화면 좌표"""
    return tuple(int(x * DISPLAY_SCALE) for x in v)


def upscale(img):
    return img if DISPLAY_SCALE == 1 else cv2.resize(img, (DISP_W, DISP_H), interpolation=cv2.INTER_LINEAR)


# =========================
# RealSense
# =========================

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.depth,
    DEPTH_WIDTH,
    DEPTH_HEIGHT,
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
model_name = args.model  # V 키: MODELS 전환
depth_reject = args.reject  # E 키: depth 거부 방식
shape_tol = SHAPE_TOL  # 1~9 키: 형상 검사 허용오차 1~9 cm
shape_use_fill = False  # F 키: 형상 점수 empty <-> fill x empty
size_check = not args.no_size_check  # Z 키: 박스 크기 ↔ depth 검사

# 모델·입력 크기별 YOLO 통계 (종료 시 출력)
yolo_stats = {(m, s): {"ms": [], "frames": 0, "conf": []} for m in MODELS for s in YOLO_IMGSZ}

# 카메라 통계 (종료 시 출력)
#   align_ms: rs.align (depth 전체 프레임을 color 좌표로 변환)
#   latency_ms: 촬영 시각 → 이 프레임을 받은 시각. 추론이 카메라 주기보다 느리면 큐에 쌓인 지난 프레임을 받아 커짐
cam_stats = {"align_ms": [], "loop_ms": [], "latency_ms": []}

video_writer = None
record_queue = None
record_thread = None
record_size = None

# 녹화 포맷: WebM(VP8). VP9는 실시간 인코딩에 너무 느림 (프레임당 ~300ms)
RECORD_EXT = "webm"
RECORD_FOURCC = "VP80"


# =========================
# 검출
# =========================

# 모두 미리 로드 (전환 시 끊김 없도록)
models = {name: YOLO(str(path)) for name, path in MODELS.items()}
models.update({key: YOLO(str(path)) for key, path in SIZE_MODELS.items()})


def get_model(name, size):
    """그 크기 전용 모델이 있으면 그것, 없으면 640 학습 모델"""
    return models.get((name, size), models[name])


def model_label(name, size):
    """화면·통계 표시용 이름: 전용 모델이면 학습 크기 표시 (예: v3@256)"""
    return f"{name}@{size[1]}" if (name, size) in SIZE_MODELS else name


# 모델·크기별 워밍업 (첫 추론은 느려서 통계가 왜곡됨)
for name in MODELS:
    for size in YOLO_IMGSZ:
        get_model(name, size).predict(np.zeros((HEIGHT, WIDTH, 3), np.uint8), imgsz=size, device=YOLO_DEVICE, verbose=False)


def detect_yolo(frame):
    """YOLO로 가장 신뢰도 높은 박스 1개 반환 -> ((x1, y1, x2, y2), conf) 또는 (None, None)"""

    size = YOLO_IMGSZ[imgsz_index]

    t0 = time.perf_counter()
    boxes = get_model(model_name, size).predict(frame, imgsz=size, conf=YOLO_CONF, device=YOLO_DEVICE, verbose=False)[0].boxes
    elapsed_ms = (time.perf_counter() - t0) * 1000  # 전처리 + 추론 + 후처리

    stats = yolo_stats[(model_name, size)]
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
    """모델·입력 크기별 처리 시간·검출률 출력 (PC 속도라 Pi와 절대값은 다름, 크기 간 비율을 봄)"""

    print(f"\n{'model':>7} {'imgsz':>9} {'frames':>7} {'mean ms':>8} {'p50 ms':>7} {'FPS':>6} {'detect':>7} {'conf':>6}")

    for (name, (h, w)), stats in yolo_stats.items():
        if stats["frames"] == 0:
            continue

        mean_ms = statistics.mean(stats["ms"])
        detect_ratio = len(stats["conf"]) / stats["frames"]
        conf = f"{statistics.mean(stats['conf']):.3f}" if stats["conf"] else "-"

        print(f"{model_label(name, (h, w)):>7} {w:>4}x{h:<4} {stats['frames']:7d} {mean_ms:8.1f} {statistics.median(stats['ms']):7.1f} "
              f"{1000 / mean_ms:6.1f} {detect_ratio:7.1%} {conf:>6}")


def print_cam_stats():
    """정렬 시간과 프레임 간격·지연"""

    if not cam_stats["loop_ms"] or not cam_stats["align_ms"]:
        return

    align_ms = cam_stats["align_ms"]
    p95 = sorted(align_ms)[int(len(align_ms) * 0.95)]
    loop_ms = statistics.mean(cam_stats["loop_ms"])

    print(f"\n[camera {WIDTH}x{HEIGHT} {FPS} FPS]")
    print(f"{'frames':>7} {'align ms':>9} {'p95 ms':>7} {'loop ms':>8} {'FPS':>6} {'latency p50':>12}")
    print(f"{len(cam_stats['loop_ms']):7d} {statistics.mean(align_ms):9.2f} {p95:7.2f} "
          f"{loop_ms:8.1f} {1000 / loop_ms:6.1f} {statistics.median(cam_stats['latency_ms']):9.1f} ms")


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
    candidates = [c for c in contours if cv2.contourArea(c) >= MIN_AREA * (WIDTH / 640) ** 2]

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
    global video_writer, record_queue, record_thread, record_size

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"record_{timestamp}.{RECORD_EXT}"

    fourcc = cv2.VideoWriter_fourcc(*RECORD_FOURCC)

    # 검출 OFF로 시작하면 학습용 원본 해상도, ON이면 화면(확대) 크기. 녹화 중 D로 바꾸면 이 크기로 맞춰 저장
    record_size = (DISP_W, DISP_H) if detection_enabled else (WIDTH, HEIGHT)

    video_writer = cv2.VideoWriter(
        filename,
        fourcc,
        FPS,
        record_size
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
    """ROI 깊이 중앙값 (m) -> (depth, stats). 범위 밖·유효 픽셀 부족이면 0.0 (perception 노드 z=0과 같음)

    depth는 color에 정렬되어 있어 bbox 좌표를 그대로 씀
    """

    depth_image = np.asanyarray(depth_frame.get_data())
    img_h, img_w = depth_image.shape

    box_width = x2 - x1
    box_height = y2 - y1

    roi_width = int(box_width * DEPTH_ROI_RATIO)
    roi_height = int(box_height * DEPTH_ROI_RATIO)

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2

    rx1 = max(0, cx - roi_width // 2)
    rx2 = min(img_w, cx + roi_width // 2)

    ry1 = max(0, cy - roi_height // 2)
    ry2 = min(img_h, cy + roi_height // 2)

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


def colorize_depth(depth_m):
    """깊이(m) -> BGR (유효 범위: 가까움=빨강 ~ 멀리=파랑, 0=검정, 너무 가까움=마젠타, 너무 멂=회색)"""

    norm = np.clip((depth_m - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH), 0.0, 1.0)
    colored = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_JET)

    colored[depth_m == 0] = COLOR_ZERO
    colored[(depth_m > 0) & (depth_m < MIN_DEPTH)] = COLOR_TOO_NEAR
    colored[depth_m > MAX_DEPTH] = COLOR_TOO_FAR
    return colored


def full_depth_view(depth_frame, message):
    """Depth Shape 창 기본 화면 (검사 안 함): 전체 depth 맵 + 사유"""
    depth_m = np.asanyarray(depth_frame.get_data()) * depth_frame.get_units()
    view = colorize_depth(depth_m)
    view = upscale(view)
    cv2.rectangle(view, (5, 10), (DISP_W - 5, 37), (0, 0, 0), -1)
    cv2.putText(view, message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return view


def touches_edge(box):
    """bbox 외곽이 화면 끝(상하좌우) 중 하나라도 SHAPE_EDGE_PX 안에 붙어 있는지"""
    x1, y1, x2, y2 = box
    return (x1 <= SHAPE_EDGE_PX or y1 <= SHAPE_EDGE_PX
            or x2 >= WIDTH - 1 - SHAPE_EDGE_PX or y2 >= HEIGHT - 1 - SHAPE_EDGE_PX)


def check_depth_shape(depth_frame, box, tol, use_fill):
    """bbox 깊이 최빈값 ±tol 픽셀로 형상 점수 계산 -> (score, view)
    fill  = bbox 안 ±tol 픽셀 비율 (차 있을수록 1)
    empty = bbox 주위(각 방향 SHAPE_MARGIN_PX, 화면 밖 제외) ±tol 아닌 픽셀 비율 (비어 있을수록 1)
    score = fill x empty (use_fill) 또는 empty (bbox 안쪽은 기준 깊이 구할 때만 씀). 기준 깊이를 못 구하면 score 0
    view: 탐색 영역 밖은 depth 컬러맵. 영역 안은 주위의 ±tol 픽셀(빨강), 나머지(흰색 50% + 컬러맵 50%), bbox(노랑), 주위 영역(회색).
          bbox 안쪽은 use_fill이면 ±tol 픽셀 초록·나머지 흰색 50%, 아니면 depth 컬러맵 그대로"""

    x1, y1, x2, y2 = box

    # 주위 영역: bbox를 SHAPE_MARGIN(SHAPE_MARGIN_PX를 해상도에 맞춘 값)만큼 넓힘 (배경이면 같은 깊이가 이 영역까지 퍼짐)
    sx1 = max(0, x1 - SHAPE_MARGIN)
    sy1 = max(0, y1 - SHAPE_MARGIN)
    sx2 = min(WIDTH, x2 + SHAPE_MARGIN)
    sy2 = min(HEIGHT, y2 + SHAPE_MARGIN)

    depth_image = np.asanyarray(depth_frame.get_data())
    region = depth_image[sy1:sy2, sx1:sx2] * depth_frame.get_units()

    # 기준 깊이: YOLO bbox 전체 유효 깊이의 최빈값 (SHAPE_BIN 구간 히스토그램에서 가장 많은 구간의 중앙값)
    # 평균은 퍽·배경이 섞이면 둘 사이 값이 되어 어느 쪽에도 안 걸림
    box_depth = region[y1 - sy1:y2 - sy1, x1 - sx1:x2 - sx1]
    box_valid = box_depth[(box_depth >= MIN_DEPTH) & (box_depth <= MAX_DEPTH)]
    if box_valid.size == 0:
        return 0.0, full_depth_view(depth_frame, "No valid depth in bbox")
    bins = ((box_valid - MIN_DEPTH) / SHAPE_BIN).astype(np.int32)
    mode_bin = int(np.argmax(np.bincount(bins)))
    depth = float(np.median(box_valid[bins == mode_bin]))

    mask = np.abs(region - depth) <= tol

    # bbox 안 / 주위 구분
    inside = np.zeros(mask.shape, dtype=bool)
    inside[y1 - sy1:y2 - sy1, x1 - sx1:x2 - sx1] = True

    ring_area = (~inside).sum()
    # 주위 영역이 없으면(bbox가 화면 전체) 퍼짐을 판단할 근거가 없으므로 비어 있다고 봄
    empty = float(1.0 - mask[~inside].sum() / ring_area) if ring_area > 0 else 1.0
    fill = float(mask[inside].sum() / inside.sum())
    score = fill * empty if use_fill else empty

    # 시각화: 탐색 영역 밖은 depth 컬러맵 그대로
    # 탐색 영역 안: 주위의 ±tol 픽셀은 빨강, 나머지는 흰색 50% + depth 컬러맵 50%
    # bbox 안쪽: fill을 쓰면 ±tol 픽셀 초록·나머지 흰색 50%, 안 쓰면 depth 컬러맵 그대로
    view = colorize_depth(depth_image * depth_frame.get_units())
    sub = view[sy1:sy2, sx1:sx2]
    faded = ((sub.astype(np.uint16) + 255) // 2).astype(np.uint8)
    blend = ~inside | use_fill
    sub[blend] = faded[blend]
    sub[mask & ~inside] = (0, 0, 255)
    if use_fill:
        sub[mask & inside] = (0, 255, 0)
    view = upscale(view)
    cv2.rectangle(view, sp(sx1, sy1), sp(sx2 - 1, sy2 - 1), (128, 128, 128), 1)
    cv2.rectangle(view, sp(x1, y1), sp(x2, y2), (0, 255, 255), 2)

    lines = [
        (f"bbox mode {depth:.3f} m +- {tol * 100:.1f} cm", (255, 255, 255)),
        ((f"fill {fill:.2f} x empty {empty:.2f} = {score:.2f}" if use_fill else f"empty {score:.2f}")
         + f" (min {SHAPE_MIN_SCORE}) [F]",
         (0, 255, 0) if score >= SHAPE_MIN_SCORE else (0, 0, 255)),
        (f"bbox valid {box_valid.size}/{box_depth.size} px, "
         f"range {box_valid.min():.2f}~{box_valid.max():.2f} m", (255, 255, 255)),
    ]
    # 글자가 초록·빨강 픽셀 위에서도 보이도록 검은 배경
    cv2.rectangle(view, (5, 10), (DISP_W - 5, 15 + 22 * len(lines)), (0, 0, 0), -1)
    for i, (line, color) in enumerate(lines):
        cv2.putText(view, line, (10, 28 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

    return score, view


def make_roi_view(roi):
    """ROI 깊이를 픽셀별 색으로 표시 (유효 범위: 가까움=빨강 ~ 멀리=파랑)"""

    # 배경은 0(검정)과 구분되도록 어두운 회색
    view = np.full(
        (ROI_VIEW_SIZE + LEGEND_HEIGHT, ROI_VIEW_SIZE, 3),
        40,
        dtype=np.uint8
    )

    if roi is not None and roi.size > 0:

        colored = colorize_depth(roi)

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
cv2.namedWindow(SHAPE_WINDOW_NAME, cv2.WINDOW_NORMAL)

# ROI 깊이 창은 메인 창 오른쪽, 형상 검사 창은 아래
cv2.moveWindow(WINDOW_NAME, 0, 0)
cv2.moveWindow(ROI_WINDOW_NAME, DISP_W + 20, 0)
cv2.moveWindow(SHAPE_WINDOW_NAME, 0, DISP_H + 60)


last_loop_t = None

try:

    while True:

        frames = pipeline.wait_for_frames()

        loop_t = time.perf_counter()
        if last_loop_t is not None:
            cam_stats["loop_ms"].append((loop_t - last_loop_t) * 1000)
        last_loop_t = loop_t

        # 뎁스카메라와 RGB카메라 시점 정렬 (depth 전체 프레임을 color 좌표로 변환)
        t0 = time.perf_counter()
        frames = align.process(frames)
        align_ms = (time.perf_counter() - t0) * 1000
        cam_stats["align_ms"].append(align_ms)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # 촬영 시각(global time, ms) → 받은 시각
        cam_stats["latency_ms"].append(time.time() * 1000 - color_frame.get_timestamp())

        # 원본 RGB 프레임
        raw_frame = np.asanyarray(
            color_frame.get_data()
        )

        # 화면 표시용 복사본 (DISPLAY_SCALE배 확대, 그릴 때 좌표는 sp()로 변환)
        display_frame = upscale(raw_frame.copy())

        mask = np.zeros(
            (HEIGHT, WIDTH),
            dtype=np.uint8
        )

        roi_depth = None
        snap_box = None
        shape_view = full_depth_view(depth_frame, f"Depth reject: {depth_reject} [E]" + ("" if depth_reject == "shape" else " (shape check off)"))

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
                snap_box = box

                w = x2 - x1
                h = y2 - y1

                cx = x1 + w // 2
                cy = y1 + h // 2

                depth, stats = get_median_depth(depth_frame, *box)
                measured_depth = depth  # 거부 전 값 (크기 표시용)

                left_edge = False
                shape_score = None
                size = None
                size_bad = False

                if depth_reject == "edge":
                    # 화면 왼쪽 끝에 닿으면 depth 사각지대로 보고 0 처리 (최종 채택, Pi와 같음)
                    left_edge = x1 <= LEFT_EDGE_PX
                    if left_edge:
                        depth = 0.0

                elif depth_reject == "shape":
                    # 깊이 형상 검사 (이전 시도): bbox 안은 차 있고 주위는 비어 있어야 함. 아니면 배경 깊이로 보고 거부
                    if cx >= WIDTH * SHAPE_X_RATIO:
                        shape_view = full_depth_view(
                            depth_frame,
                            f"Center x {cx} >= {WIDTH * SHAPE_X_RATIO:.0f} (shape check skipped)"
                        )
                    elif not touches_edge(box):
                        shape_view = full_depth_view(depth_frame, "bbox not touching image edge (shape check skipped)")
                    elif depth > 0:
                        shape_score, shape_view = check_depth_shape(
                            depth_frame, box, shape_tol, shape_use_fill
                        )
                        if shape_score < SHAPE_MIN_SCORE:
                            depth = 0.0
                    else:
                        shape_view = full_depth_view(depth_frame, "No valid depth (shape check skipped)")

                # 박스 크기 ↔ depth: 긴 변 실제 크기 = px × z / f, 기대 박스 긴 변 = f × 퍽 크기 / z
                # 표시는 거부 전 depth로 계산 (다른 검사로 0이 돼도 크기는 보여 줌)
                expect_px = None
                if measured_depth > 0:
                    size = max(w * measured_depth / FX, h * measured_depth / FY)
                    expect_px = (FX * PUCK_MIN_M / measured_depth, FX * PUCK_MAX_M / measured_depth)
                if depth > 0:
                    at_edge = x1 <= 1 or y1 <= 1 or x2 >= WIDTH - 2 or y2 >= HEIGHT - 2
                    size_bad = size_check and (size > SIZE_MAX_M or (not at_edge and size < SIZE_MIN_M))
                    if size_bad:
                        depth = 0.0

                # 기대 크기 사각형 (보라): 박스 중심에 기대 긴 변 최소·최대 정사각형. 실제 박스가 둘 사이에 있으면 정상
                if expect_px is not None:
                    for side in expect_px:
                        half = side / 2
                        cv2.rectangle(display_frame, sp(cx - half, cy - half), sp(cx + half, cy + half),
                                      (255, 0, 255), 1)

                # Bounding Box
                cv2.rectangle(
                    display_frame,
                    sp(x1, y1),
                    sp(x2, y2),
                    (0, 255, 0),
                    2
                )

                # Contour (HSV 모드)
                if target is not None:
                    cv2.drawContours(
                        display_frame,
                        [target * DISPLAY_SCALE],
                        -1,
                        (255, 0, 0),
                        2
                    )

                # 중심점
                cv2.circle(
                    display_frame,
                    sp(cx, cy),
                    5,
                    (0, 0, 255),
                    -1
                )

                if depth > 0:
                    text = f"Depth: {depth:.3f} m"
                elif left_edge:
                    text = "Depth: 0 (touching left edge)"
                elif shape_score is not None and shape_score < SHAPE_MIN_SCORE:
                    text = f"Depth: 0 (shape score {shape_score:.2f} < {SHAPE_MIN_SCORE})"
                elif size_bad:
                    text = f"Depth: 0 (size {size * 100:.1f} cm, {SIZE_MIN_M * 100:g}~{SIZE_MAX_M * 100:.0f})"
                else:
                    text = f"Depth: 0 (invalid, {MIN_DEPTH:.1f}~{MAX_DEPTH:.1f} m)"

                cv2.putText(
                    display_frame,
                    text,
                    (x1 * DISPLAY_SCALE, max(20, y1 * DISPLAY_SCALE - 10)),
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
                    f"Box: {w}x{h} px" + (f" (expect {expect_px[0]:.0f}~{expect_px[1]:.0f})" if expect_px else ""),
                    "Size: " + (f"{size * 100:.1f} cm (expect {PUCK_MIN_M * 100:.1f}~{PUCK_MAX_M * 100:.1f})"
                                if size is not None else "N/A (no depth)"),
                    f"Size check: {SIZE_MIN_M * 100:g}~{SIZE_MAX_M * 100:g} cm "
                    + (("REJECT" if size_bad else "ok") if size_check else "OFF [Z]"),
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
                        sp(rx1, ry1),
                        sp(rx2, ry2),
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

                if shape_score is not None:
                    debug_lines.append(
                        f"Shape {'fill x empty' if shape_use_fill else 'empty'}: "
                        f"{shape_score:.2f} (min {SHAPE_MIN_SCORE})"
                    )
                    if conf is not None:
                        debug_lines.append(f"Conf x Score: {conf * shape_score:.2f}")

                # 우측 정보 패널 (위쪽 상태 표시 줄 아래부터)
                panel_x = DISP_W - 250
                panel_top = 115

                cv2.rectangle(
                    display_frame,
                    (panel_x - 5, panel_top - 18),
                    (DISP_W - 5, panel_top - 13 + 20 * len(debug_lines)),
                    (0, 0, 0),
                    -1
                )

                for i, line in enumerate(debug_lines):
                    cv2.putText(
                        display_frame,
                        line,
                        (panel_x, panel_top + 20 * i),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (255, 255, 255),
                        1
                    )


        # =========================
        # 상태 표시
        # =========================

        cv2.putText(
            display_frame,
            f"CAM {WIDTH}x{HEIGHT}@{FPS} | align {align_ms:.1f} ms"
            f" | latency {cam_stats['latency_ms'][-1]:.0f} ms | REJECT: {depth_reject}"
            f"{' +size' if size_check else ''}",
            (10, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2
        )

        if detection_enabled and use_yolo:
            h_in, w_in = YOLO_IMGSZ[imgsz_index]
            last_ms = yolo_stats[(model_name, (h_in, w_in))]["ms"][-1]
            detection_text = f"DETECTION: ON (YOLO {model_label(model_name, (h_in, w_in))} {w_in}x{h_in}) {last_ms:.0f} ms ({1000 / last_ms:.1f} FPS)"
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

        for i, line in enumerate([
            "D: Detect | M: YOLO/HSV | V: model | I: size | R: Rec | Q: Quit",
            "E: Depth reject | Z: Size check | S: Snap | F: Fill | 1~9: Tol cm",
        ]):
            cv2.putText(
                display_frame,
                line,
                (10, DISP_H - 35 + 20 * i),
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
            frame = display_frame if detection_enabled else raw_frame.copy()
            if (frame.shape[1], frame.shape[0]) != record_size:
                frame = cv2.resize(frame, record_size)
            record_queue.put(frame)


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

        cv2.imshow(
            SHAPE_WINDOW_NAME,
            shape_view
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

        elif key == ord("v"):

            names = list(MODELS)
            model_name = names[(names.index(model_name) + 1) % len(names)]
            print(f"[YOLO MODEL] {model_label(model_name, YOLO_IMGSZ[imgsz_index])}")

        elif key == ord("i"):

            imgsz_index = (imgsz_index + 1) % len(YOLO_IMGSZ)

            h_in, w_in = YOLO_IMGSZ[imgsz_index]
            print(f"[YOLO SIZE] {w_in}x{h_in} ({model_label(model_name, (h_in, w_in))})")

        elif key == ord("s"):

            # 분석용 스냅샷: 원본 color + depth(raw z16, color에 정렬) + bbox
            snap_name = f"snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"
            np.savez_compressed(
                snap_name,
                color=raw_frame,
                depth=np.asanyarray(depth_frame.get_data()),
                depth_units=depth_frame.get_units(),
                box=np.array(snap_box if snap_box is not None else [], dtype=np.int32),
            )
            print(f"[SNAP] {snap_name} box={snap_box}")

        elif ord("1") <= key <= ord("9"):

            shape_tol = (key - ord("0")) / 100
            print(f"[SHAPE TOL] +-{shape_tol * 100:.0f} cm")

        elif key == ord("f"):

            shape_use_fill = not shape_use_fill
            print("[SHAPE SCORE]", "fill x empty" if shape_use_fill else "empty")

        elif key == ord("z"):

            size_check = not size_check
            print("[SIZE CHECK]", "ON" if size_check else "OFF")

        elif key == ord("e"):

            depth_reject = DEPTH_REJECT_MODES[(DEPTH_REJECT_MODES.index(depth_reject) + 1) % len(DEPTH_REJECT_MODES)]
            print("[DEPTH REJECT]", depth_reject)

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

        if cv2.getWindowProperty(
            SHAPE_WINDOW_NAME,
            cv2.WND_PROP_VISIBLE
        ) < 1:
            break


finally:

    if recording:
        stop_recording()

    pipeline.stop()

    cv2.destroyAllWindows()

    print_yolo_stats()
    print_cam_stats()