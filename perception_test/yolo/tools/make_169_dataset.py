"""Convert the 4:3 (640x480) YOLO dataset into a 16:9 (640x360) one with the D435 geometry.

D435 color 640x480 is the centre crop of the 16:9 sensor (fx 608), and 640x360 is the whole
16:9 sensor scaled by 0.75 (fx 456, HFOV 70°). So a 640x480 frame shrunk to 480x360 is exactly
the centre of a 640x360 frame (ppx 327.5*0.75+80 = 325.6, measured 325.6). The 80 px side
strips the 4:3 frame never saw are filled with letterbox grey (114).

Labels: x' = 0.75x + 0.125, w' = 0.75w, y and h unchanged.

usage: python tools/make_169_dataset.py [--src dataset] [--out dataset_169]
"""
import argparse
import shutil
from pathlib import Path

import cv2

YOLO_DIR = Path(__file__).resolve().parent.parent
OUT_W, OUT_H = 640, 360
PAD = 114


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--src", default=str(YOLO_DIR / "dataset"))
    p.add_argument("--out", default=str(YOLO_DIR / "dataset_169"))
    args = p.parse_args()
    src, out = Path(args.src), Path(args.out)
    if out.exists():
        shutil.rmtree(out)

    for img_path in sorted((src / "images").rglob("*.jpg")):
        rel = img_path.relative_to(src / "images")
        img = cv2.imread(str(img_path))
        h, w = img.shape[:2]
        if (w, h) != (640, 480):
            raise SystemExit(f"{img_path}: expected 640x480, got {w}x{h}")
        s = OUT_H / h                      # 0.75
        nw = round(w * s)                  # 480
        x0 = (OUT_W - nw) // 2             # 80
        small = cv2.resize(img, (nw, OUT_H), interpolation=cv2.INTER_AREA)
        canvas = cv2.copyMakeBorder(small, 0, 0, x0, OUT_W - nw - x0, cv2.BORDER_CONSTANT, value=(PAD,) * 3)
        (out / "images" / rel.parent).mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / "images" / rel), canvas, [cv2.IMWRITE_JPEG_QUALITY, 95])

        lines = []
        lbl = src / "labels" / rel.with_suffix(".txt")
        for line in (lbl.read_text().split("\n") if lbl.exists() else []):
            if not line.strip():
                continue
            c, x, y, bw, bh = line.split()
            x = (float(x) * nw + x0) / OUT_W
            bw = float(bw) * nw / OUT_W
            lines.append(f"{c} {x:.6f} {y} {bw:.6f} {bh}")
        (out / "labels" / rel.parent).mkdir(parents=True, exist_ok=True)
        (out / "labels" / rel.with_suffix(".txt")).write_text("\n".join(lines) + ("\n" if lines else ""))

    names = (src / "data.yaml").read_text().split("names:", 1)[1]
    (out / "data.yaml").write_text(f"path: {out.resolve()}\ntrain: images/train\nval: images/val\nnames:{names}")
    print(f"-> {out / 'data.yaml'}")


if __name__ == "__main__":
    main()
