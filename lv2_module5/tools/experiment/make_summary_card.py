#!/usr/bin/env python3
"""make_summary_card.py — results/plots/summary_1008.png (실험 결과 한 장 요약) 생성

값은 results/README.md · metrics.csv 와 같다 (그 값이 바뀌면 여기 TILES 도 같이 고친다).
  python3 tools/experiment/make_summary_card.py
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, '..', '..', 'results', 'plots', 'summary_1008.png'))
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONT_B = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'

SURFACE, CARD, BORDER = '#f7f7f5', '#ffffff', '#dddcd6'
INK, INK2, MUTED = '#1f1f1d', '#4a4a46', '#7a7a74'
STATUS = {   # (점 색, 글자)  — 색만으로 뜻을 전하지 않게 글자를 같이 쓴다
    'pass': ('#0ca30c', '통과'),
    'note': ('#8a8a85', '기록'),
    'warn': ('#fab219', '확인 필요'),
    'doc': ('#8a8a85', '문서 작업'),
    'none': ('#d03b3b', '미수행'),
}

# (평가, 시험, 핵심 값, 보조 설명, 상태, 근거 폴더)
TILES = [
    ('평가 4', '모의 입력 5종 (모터 OFF)', '5 / 5', 'x=0 · ±0.4 · 미검출 · 발행 중단', 'pass', 'logs/IN-02'),
    ('평가 6', '상태 전이 M-T1 ~ M-T8', '8 / 8', 'IDLE·TRACKING·SEARCHING·LOST·FAULT', 'pass', 'logs/M-T'),
    ('평가 6', '2초 가림 × 5 (R1)', '반응 0.30–0.36 s', '시야 안 복구 0/5 (탐색 후 재발견)', 'note', 'logs/R1'),
    ('평가 7', '인지 토픽 중단', '0.51 s → 정지', '마지막 검출 후 FAULT, 팔 [0, 0]', 'pass', 'logs/M-T'),
    ('평가 7', '제어 통신 중단 (kill -9)', '보드가 정지', 'OpenCR watchdog 300 ms, 육안 확인', 'pass', 'logs/M-C2'),
    ('평가 8', '정상 추적 41.3 s', 'TRACKING 100%', 'FAULT · SEARCHING 0회', 'pass', 'logs/B0-01'),
    ('평가 8', '처리 FPS', '8.64 Hz', '/detection 357개 ÷ 41.2 s', 'pass', 'logs/B0-01'),
    ('평가 8', '수평 RMSE', '0.0003', '정지 목표 · 움직이는 목표 0.216', 'pass', 'logs/B0-01'),
    ('평가 8', '검출률 (사람 대조)', '30 / 30', '권혁무 · 정수용 육안 판정', 'pass', 'images/detection_30'),
    ('평가 8', '배경 오검출', '0 / 19', '목표 없는 프레임에서 잘못 검출', 'pass', 'images/no_target_19'),
    ('평가 9', '성공 · 소실 bag', '24.1 s · 30.1 s', '재등장 → TRACKING 0.36–0.39 s', 'pass', 'logs/BAG-OK · BAG-LOST'),
    ('평가 9', '재현 (재분석 · 재처리)', '172 / 172 일치', '모터 없이 bag 재생, 값 차이 0', 'pass', 'logs/RAW-01 · RE2-01'),
]

# 평가표 1 ~ 11 전체 상태
TABLE = [
    ('1', '조건 정의', 'doc'), ('2', '실행 환경', 'doc'), ('3', 'HSV 검출', 'warn'), ('4', '인터페이스', 'pass'),
    ('5', 'Kp 비교', 'none'), ('6', '소실·복구', 'pass'), ('7', '통신 중단', 'pass'), ('8', '성능 측정', 'pass'),
    ('9', 'bag 재현', 'pass'), ('10', '협업·PR', 'doc'), ('11', '제출·시연', 'doc'),
]


def main():
    fp, fb = font_manager.FontProperties(fname=FONT), font_manager.FontProperties(fname=FONT_B)
    W, H = 16.0, 11.0
    fig = plt.figure(figsize=(W, H), dpi=110)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis('off')
    fig.patch.set_facecolor(SURFACE)

    ax.text(0.5, 0.75, '실험 결과 한눈에 보기 — 실제 로봇 (2026-10-07 · 08)', fontproperties=fb, fontsize=24, color=INK, va='center')
    ax.text(0.5, 1.3, '카드 한 장 = 시험 하나.  큰 숫자가 결과,  맨 아래 회색 글씨가 근거 폴더 (results/ 기준)',
            fontproperties=fp, fontsize=13, color=INK2, va='center')

    cols, cw, ch, gx, gy, x0, y0 = 4, 3.6, 2.05, 0.2, 0.22, 0.5, 1.75
    for i, (grade, name, value, sub, st, where) in enumerate(TILES):
        x, y = x0 + (i % cols) * (cw + gx), y0 + (i // cols) * (ch + gy)
        ax.add_patch(FancyBboxPatch((x, y), cw, ch, boxstyle='round,pad=0,rounding_size=0.12',
                                    fc=CARD, ec=BORDER, lw=1.2))
        ax.text(x + 0.22, y + 0.33, grade, fontproperties=fb, fontsize=11, color=MUTED, va='center')
        color, label = STATUS[st]
        ax.add_patch(Circle((x + cw - 1.05, y + 0.33), 0.07, color=color))
        ax.text(x + cw - 0.9, y + 0.33, label, fontproperties=fb, fontsize=11, color=INK2, va='center')
        ax.text(x + 0.22, y + 0.72, name, fontproperties=fb, fontsize=13.5, color=INK, va='center')
        ax.text(x + 0.22, y + 1.22, value, fontproperties=fb, fontsize=22, color=INK, va='center')
        ax.text(x + 0.22, y + 1.62, sub, fontproperties=fp, fontsize=10.5, color=INK2, va='center')
        ax.text(x + 0.22, y + 1.88, where, fontproperties=fp, fontsize=10, color=MUTED, va='center')

    yb = y0 + 3 * (ch + gy) + 0.35
    ax.text(0.5, yb, '발제 평가표 1 ~ 11 전체 상태', fontproperties=fb, fontsize=15, color=INK, va='center')
    bw, bg = (W - 1.0 - 10 * 0.12) / 11, 0.12
    for j, (no, name, st) in enumerate(TABLE):
        x = 0.5 + j * (bw + bg)
        ax.add_patch(FancyBboxPatch((x, yb + 0.35), bw, 1.05, boxstyle='round,pad=0,rounding_size=0.08',
                                    fc=CARD, ec=BORDER, lw=1.0))
        color, label = STATUS[st]
        ax.text(x + bw / 2, yb + 0.6, f'{no}. {name}', fontproperties=fb, fontsize=10.5, color=INK, ha='center', va='center')
        ax.add_patch(Circle((x + 0.2, yb + 1.05), 0.06, color=color))
        ax.text(x + 0.33, yb + 1.05, label, fontproperties=fp, fontsize=9.5, color=INK2, va='center')
    ax.text(0.5, yb + 1.75, '3: HSV 대신 YOLO (튜터 확인 필요)   5: 단일 Kp 제어가 아니라 미수행 (report.md 에 사유 기록 필요)   '
            '1·2·10·11: report · README · team · presentation 문서',
            fontproperties=fp, fontsize=10.5, color=INK2, va='center')

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, facecolor=SURFACE)
    print('저장', OUT)


if __name__ == '__main__':
    main()
