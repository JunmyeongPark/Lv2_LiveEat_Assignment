"""Build a YOLO dataset from extracted frames and a makesense YOLO export zip.

Every frame is kept: frames without a label file in the zip become negatives (empty label).
The split is by contiguous time (the last --val-frac of each video goes to val) so that
near-identical neighbouring frames do not leak between train and val.

usage: python prepare_dataset.py <labels.zip> <frames_dir> [--out dataset]
"""
import argparse
import shutil
import zipfile
from collections import defaultdict
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("labels_zip")
    p.add_argument("frames", help="folder of .jpg frames (searched recursively)")
    p.add_argument("--out", default="dataset")
    p.add_argument("--val-frac", type=float, default=0.2)
    p.add_argument("--names", nargs="+", default=["target_blue"])
    args = p.parse_args()

    with zipfile.ZipFile(args.labels_zip) as z:
        labels = {Path(n).stem: z.read(n).decode().strip() for n in z.namelist() if n.endswith(".txt")}

    # group frames by video: "<video>_<frame idx>.jpg"
    videos = defaultdict(list)
    for img in Path(args.frames).rglob("*.jpg"):
        videos[img.stem.rsplit("_", 1)[0]].append(img)

    out = Path(args.out)
    if out.exists():
        shutil.rmtree(out)
    for kind in ("images", "labels"):
        for split in ("train", "val"):
            (out / kind / split).mkdir(parents=True)

    stats = {s: {"images": 0, "boxes": 0, "negatives": 0} for s in ("train", "val")}
    for _, imgs in sorted(videos.items()):
        imgs.sort()
        n_val = round(len(imgs) * args.val_frac)
        for i, img in enumerate(imgs):
            split = "val" if i >= len(imgs) - n_val else "train"
            text = labels.pop(img.stem, "")
            shutil.copy(img, out / "images" / split / img.name)
            (out / "labels" / split / f"{img.stem}.txt").write_text(text + "\n" if text else "")
            n_box = len(text.splitlines()) if text else 0
            stats[split]["images"] += 1
            stats[split]["boxes"] += n_box
            stats[split]["negatives"] += n_box == 0
    if labels:
        print(f"warning: {len(labels)} labels have no matching frame, e.g. {next(iter(labels))}")

    names = "\n".join(f"  {i}: {n}" for i, n in enumerate(args.names))
    (out / "data.yaml").write_text(f"path: {out.resolve()}\ntrain: images/train\nval: images/val\nnames:\n{names}\n")
    for split, s in stats.items():
        print(f"{split}: {s['images']} images, {s['boxes']} boxes, {s['negatives']} negatives")
    print(f"-> {out / 'data.yaml'}")


if __name__ == "__main__":
    main()
