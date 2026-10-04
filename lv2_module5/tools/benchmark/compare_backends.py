#!/usr/bin/env python3
"""
compare_backends.py — run_benchmark.sh 결과로 NCNN / ONNX 비교 지표·그래프 생성

입력 폴더 구조 (run_benchmark.sh 출력)
  <backend>_r<i>.csv            프레임별 시간·검출 (detector_bench)
  <backend>_r<i>.csv.meta.json  실행 조건
  <backend>_r<i>_sys.csv        CPU·메모리·온도 (sys_monitor.py)

출력 (같은 폴더)
  summary.md       보고서용 표 (지표 정의 포함)
  summary.csv      백엔드별 지표 (반복 평균 ± 표준편차)
  per_run.csv      실행(반복)별 지표
  agreement.csv    두 백엔드 출력 일치도
  plots/*.png      latency 분포, 프레임별 latency, 온도·CPU, e_x 일치도

사용
  python3 compare_backends.py <out_dir> [--exclude-first N]
의존성: numpy, pandas, matplotlib
"""
import argparse
import glob
import json
import os
import re
import sys

import numpy as np

try:
    import pandas as pd
except ImportError:
    sys.exit("pandas 필요: pip install pandas")

RUN_RE = re.compile(r"^(?P<backend>[a-z0-9]+)_r(?P<run>\d+)\.csv$")


def load_runs(d):
    runs = []
    for path in sorted(glob.glob(os.path.join(d, "*_r*.csv"))):
        m = RUN_RE.match(os.path.basename(path))
        if not m:
            continue
        b, r = m["backend"], int(m["run"])
        frames = pd.read_csv(path)
        meta = {}
        if os.path.exists(path + ".meta.json"):
            with open(path + ".meta.json") as f:
                meta = json.load(f)
        sys_path = os.path.join(d, f"{b}_r{r}_sys.csv")
        sysdf = pd.read_csv(sys_path) if os.path.exists(sys_path) else None
        runs.append(dict(backend=b, run=r, frames=frames, meta=meta, sys=sysdf))
    return runs


def throttled_any(col):
    """vcgencmd get_throttled 값 중 0x0이 아닌 게 있으면 True, 값이 없으면(라즈베리파이 아님) None."""
    vals = [str(v).strip() for v in col.dropna().tolist() if str(v).strip() not in ("", "nan")]
    if not vals:
        return None
    return any(v != "0x0" for v in vals)


def run_metrics(run, exclude_first):
    f = run["frames"].iloc[exclude_first:]
    tot, inf = f["total_ms"].to_numpy(), f["infer_ms"].to_numpy()
    m = {
        "backend": run["backend"], "run": run["run"], "frames": len(f),
        # 속도
        "pre_ms_mean": f["pre_ms"].mean(),
        "infer_ms_mean": inf.mean(), "infer_ms_p50": np.percentile(inf, 50),
        "infer_ms_p95": np.percentile(inf, 95), "infer_ms_max": inf.max(),
        "post_ms_mean": f["post_ms"].mean(),
        "total_ms_mean": tot.mean(), "total_ms_p95": np.percentile(tot, 95), "total_ms_std": tot.std(),
        "fps": 1000.0 / tot.mean() if tot.mean() > 0 else np.nan,
        # 출력 (정답 대조가 아닌 모델 출력 기준)
        "detected_ratio": f["detected"].mean(),
        "score_mean": f.loc[f["detected"] == 1, "score"].mean(),
        # 준비 비용
        "load_ms": run["meta"].get("load_ms", np.nan),
        "model_mb": run["meta"].get("model_bytes", np.nan) / 1e6 if run["meta"] else np.nan,
    }
    s = run["sys"]
    if s is not None and len(s):
        m.update({
            "proc_cpu_pct_mean": s["proc_cpu_pct"].mean(),
            "proc_rss_mb_max": s["proc_rss_mb"].max(),
            "temp_c_max": s["temp_c"].max(),
            "cpu_mhz_min": s["cpu_mhz"].min(),
            "throttled_any": throttled_any(s["throttled"]),
        })
    return m


def iou(a, b):
    ix1, iy1 = np.maximum(a[:, 0], b[:, 0]), np.maximum(a[:, 1], b[:, 1])
    ix2, iy2 = np.minimum(a[:, 2], b[:, 2]), np.minimum(a[:, 3], b[:, 3])
    inter = np.clip(ix2 - ix1, 0, None) * np.clip(iy2 - iy1, 0, None)
    area = lambda r: (r[:, 2] - r[:, 0]) * (r[:, 3] - r[:, 1])
    union = area(a) + area(b) - inter
    return np.where(union > 0, inter / union, 0.0)


def agreement(fa, fb, name_a, name_b):
    j = fa.merge(fb, on="frame_idx", suffixes=("_a", "_b"))
    da, db = j["detected_a"] == 1, j["detected_b"] == 1
    both = j[da & db]
    cols = ["x1", "y1", "x2", "y2"]
    ious = iou(both[[c + "_a" for c in cols]].to_numpy(float), both[[c + "_b" for c in cols]].to_numpy(float))
    return {
        "pair": f"{name_a} vs {name_b}", "frames": len(j),
        "both_detected": int((da & db).sum()), f"only_{name_a}": int((da & ~db).sum()),
        f"only_{name_b}": int((~da & db).sum()), "neither": int((~da & ~db).sum()),
        "detect_agree_pct": 100.0 * float((da == db).mean()) if len(j) else np.nan,
        "iou_mean": float(ious.mean()) if len(ious) else np.nan,
        "iou_p5": float(np.percentile(ious, 5)) if len(ious) else np.nan,
        "abs_dex_mean": float((both["ex_a"] - both["ex_b"]).abs().mean()) if len(both) else np.nan,
        "abs_dex_max": float((both["ex_a"] - both["ex_b"]).abs().max()) if len(both) else np.nan,
        "abs_dey_mean": float((both["ey_a"] - both["ey_b"]).abs().mean()) if len(both) else np.nan,
        "abs_dscore_mean": float((both["score_a"] - both["score_b"]).abs().mean()) if len(both) else np.nan,
    }, j


def make_plots(d, runs, joined, name_a, name_b, exclude_first):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 없음 → 그래프 생략")
        return
    pdir = os.path.join(d, "plots")
    os.makedirs(pdir, exist_ok=True)
    backends = sorted({r["backend"] for r in runs})
    first = {b: next(r for r in runs if r["backend"] == b and r["run"] == min(x["run"] for x in runs if x["backend"] == b))
             for b in backends}

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for ax, col, title in zip(axes, ["infer_ms", "total_ms"], ["Inference latency", "Total latency (pre+infer+post)"]):
        data = [pd.concat([r["frames"][col].iloc[exclude_first:] for r in runs if r["backend"] == b]) for b in backends]
        try:
            ax.boxplot(data, tick_labels=backends, showfliers=False)   # matplotlib >= 3.9
        except TypeError:
            ax.boxplot(data, labels=backends, showfliers=False)
        ax.set_title(title); ax.set_ylabel("ms"); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(pdir, "latency_box.png"), dpi=120); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 3.5))
    for b in backends:
        f = first[b]["frames"]
        ax.plot(f["frame_idx"], f["total_ms"], lw=.8, label=b)
    ax.set_xlabel("frame"); ax.set_ylabel("total ms"); ax.set_title("Per-frame latency (run 1)")
    ax.legend(); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(pdir, "latency_series.png"), dpi=120); plt.close(fig)

    if all(first[b]["sys"] is not None for b in backends):
        fig, axes = plt.subplots(1, 2, figsize=(10, 3.5))
        for b in backends:
            s = first[b]["sys"]
            axes[0].plot(s["t_s"], s["temp_c"], label=b)
            axes[1].plot(s["t_s"], s["proc_cpu_pct"], label=b)
        axes[0].set_title("SoC temperature (run 1)"); axes[0].set_ylabel("°C")
        axes[1].set_title("Process CPU (run 1)"); axes[1].set_ylabel("%")
        for ax in axes:
            ax.set_xlabel("s"); ax.legend(); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(pdir, "system.png"), dpi=120); plt.close(fig)

    if joined is not None:
        both = joined[(joined["detected_a"] == 1) & (joined["detected_b"] == 1)]
        fig, ax = plt.subplots(figsize=(4.5, 4.5))
        ax.scatter(both["ex_a"], both["ex_b"], s=6)
        ax.plot([-1, 1], [-1, 1], "k--", lw=.8)
        ax.set_xlabel(f"e_x ({name_a})"); ax.set_ylabel(f"e_x ({name_b})"); ax.set_title("e_x agreement (run 1)")
        ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(pdir, "ex_agreement.png"), dpi=120); plt.close(fig)


METRIC_DOC = """
## 지표 정의
| 지표 | 정의 |
|---|---|
| infer_ms (mean/p50/p95/max) | 엔진 추론 시간 (입력 복사 포함). 전·후처리는 두 백엔드가 같은 코드 |
| total_ms | 전처리 + 추론 + 후처리 (프레임 디코딩 제외) |
| fps | 1000 / mean(total_ms) — 검출기 단독 처리 FPS (카메라 FPS·ROS 오버헤드 제외) |
| total_ms_std | 프레임별 처리 시간 흔들림 (지터) |
| proc_cpu_pct_mean | 벤치마크 프로세스 CPU 평균 (4코어 최대 400%) |
| proc_rss_mb_max | 최대 메모리 사용량 |
| temp_c_max / cpu_mhz_min / throttled_any | 발열·클럭 저하 여부 (스로틀링 시 결과 해석 주의) |
| load_ms | 모델 로드 시간 |
| detected_ratio | 모델이 '검출'이라고 낸 프레임 비율 — **정답 대조 검출률 아님** |
| agreement | 같은 프레임에서 두 백엔드 출력 비교: 검출 일치율, IoU, \\|Δe_x\\| |

워밍업 프레임은 기록 전에 별도로 실행했고, 분석 시 앞 N 프레임을 추가로 제외할 수 있다 (--exclude-first).
"""


def fmt(v):
    if isinstance(v, (bool, np.bool_)):
        return str(bool(v))
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    try:
        return f"{float(v):.3g}" if abs(float(v)) < 1000 else f"{float(v):.0f}"
    except (TypeError, ValueError):
        return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--exclude-first", type=int, default=0)
    a = ap.parse_args()

    runs = load_runs(a.dir)
    if not runs:
        sys.exit(f"no <backend>_r<i>.csv in {a.dir}")

    per_run = pd.DataFrame([run_metrics(r, a.exclude_first) for r in runs])
    per_run.to_csv(os.path.join(a.dir, "per_run.csv"), index=False)

    num = per_run.drop(columns=["run"]).groupby("backend")
    mean, std = num.mean(numeric_only=True), num.std(numeric_only=True)
    summary = mean.add_suffix("").copy()
    summary.to_csv(os.path.join(a.dir, "summary.csv"))

    backends = sorted(per_run["backend"].unique())
    agree_rows, joined, (na, nb) = [], None, (None, None)
    if len(backends) >= 2:
        na, nb = backends[0], backends[1]
        fa = next(r for r in runs if r["backend"] == na)["frames"]
        fb = next(r for r in runs if r["backend"] == nb)["frames"]
        row, joined = agreement(fa, fb, na, nb)
        agree_rows.append(row)
        pd.DataFrame(agree_rows).to_csv(os.path.join(a.dir, "agreement.csv"), index=False)

    key = ["frames", "infer_ms_mean", "infer_ms_p95", "total_ms_mean", "total_ms_p95", "total_ms_std", "fps",
           "proc_cpu_pct_mean", "proc_rss_mb_max", "temp_c_max", "load_ms", "model_mb", "detected_ratio"]
    key = [k for k in key if k in mean.columns]
    lines = ["# NCNN vs ONNX 벤치마크 결과", "",
             f"- 입력·조건: `{a.dir}/env.txt` 참고", f"- 반복: 백엔드별 {per_run.groupby('backend').size().to_dict()}회",
             "", "| 지표 | " + " | ".join(backends) + " |", "|---|" + "---|" * len(backends)]
    for k in key:
        cells = []
        for b in backends:
            s = std.loc[b, k] if b in std.index and k in std.columns else np.nan
            cells.append(fmt(mean.loc[b, k]) + (f" ± {fmt(s)}" if pd.notna(s) and s != 0 else ""))
        lines.append(f"| {k} | " + " | ".join(cells) + " |")
    if "throttled_any" in per_run.columns:
        def thr(b):
            v = [x for x in per_run.loc[per_run.backend == b, "throttled_any"] if x is not None and x == x]
            return "n/a" if not v else str(any(v))
        lines.append("| throttled_any | " + " | ".join(thr(b) for b in backends) + " |")
    if agree_rows:
        r = agree_rows[0]
        lines += ["", f"## 출력 일치도 ({r['pair']}, run 1)", "", "| 항목 | 값 |", "|---|---|"]
        lines += [f"| {k} | {fmt(v)} |" for k, v in r.items() if k != "pair"]
    lines.append(METRIC_DOC)
    with open(os.path.join(a.dir, "summary.md"), "w") as f:
        f.write("\n".join(lines))

    make_plots(a.dir, runs, joined, na, nb, a.exclude_first)
    print("\n".join(lines[:len(key) + 8]))
    print(f"\n-> {a.dir}/summary.md, summary.csv, per_run.csv, agreement.csv, plots/")


if __name__ == "__main__":
    main()
