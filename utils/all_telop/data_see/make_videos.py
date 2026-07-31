#!/usr/bin/env python3
"""Convert all_telop episode image folders into preview videos.

Reads:
  ~/pika_ros/data/{episode}/color|depth|fisheye

Writes:
  ~/pika_ros/utils/all_telop/data_see/{episode}/color.mp4
  ~/pika_ros/utils/all_telop/data_see/{episode}/depth.mp4   (false-color)
  ~/pika_ros/utils/all_telop/data_see/{episode}/fisheye.mp4

Depth colormap (shallow -> deep): blue -> green -> yellow -> red.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}

# Control points in BGR for near->far: blue, green, yellow, red.
DEPTH_GRADIENT_BGR = np.array(
    [
        [255, 0, 0],    # blue   (shallow)
        [0, 255, 0],    # green
        [0, 255, 255],  # yellow
        [0, 0, 255],    # red    (deep)
    ],
    dtype=np.float32,
)


def list_images(folder: Path) -> list[Path]:
    if not folder.is_dir():
        return []
    files = [p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXTS]
    return sorted(files, key=lambda p: p.name)


def build_depth_lut() -> np.ndarray:
    """(256, 3) BGR LUT: index 0 = black (invalid), 1..255 = blue..red."""
    lut = np.zeros((256, 3), dtype=np.uint8)
    stops = DEPTH_GRADIENT_BGR
    n_seg = len(stops) - 1
    for i in range(1, 256):
        t = (i - 1) / 254.0
        x = t * n_seg
        i0 = int(np.floor(x))
        i1 = min(i0 + 1, n_seg)
        a = x - i0
        color = (1.0 - a) * stops[i0] + a * stops[i1]
        lut[i] = np.clip(np.round(color), 0, 255).astype(np.uint8)
    return lut


DEPTH_LUT = build_depth_lut()


def estimate_depth_range(paths: list[Path], sample: int = 40) -> tuple[float, float]:
    """Robust mm range from non-zero pixels across a subsample of frames."""
    if not paths:
        return 1.0, 1.0
    step = max(1, len(paths) // sample)
    vals: list[np.ndarray] = []
    for p in paths[::step]:
        img = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        nz = img[img > 0]
        if nz.size:
            vals.append(nz.astype(np.float32).ravel())
    if not vals:
        return 1.0, 1.0
    all_v = np.concatenate(vals)
    lo = float(np.percentile(all_v, 1.0))
    hi = float(np.percentile(all_v, 99.0))
    if hi <= lo:
        hi = lo + 1.0
    return lo, hi


def colorize_depth(depth: np.ndarray, lo: float, hi: float) -> np.ndarray:
    """Map 16-bit depth to continuous blue-green-yellow-red BGR image."""
    if depth is None:
        return np.zeros((480, 640, 3), dtype=np.uint8)
    if depth.ndim == 3:
        depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)

    out_idx = np.zeros(depth.shape, dtype=np.uint8)
    valid = depth > 0
    if valid.any():
        d = depth.astype(np.float32)
        # 0 invalid; 1..255 scale shallow(lo)->deep(hi)
        norm = np.clip((d - lo) / (hi - lo), 0.0, 1.0)
        out_idx[valid] = (1.0 + norm[valid] * 254.0).astype(np.uint8)

    return DEPTH_LUT[out_idx]


def open_writer(path: Path, fps: float, size: tuple[int, int]) -> cv2.VideoWriter:
    path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, fps, size)
    if not writer.isOpened():
        raise RuntimeError(f"failed to open VideoWriter: {path}")
    return writer


def write_color_or_fisheye(src_dir: Path, out_path: Path, fps: float, label: str) -> int:
    paths = list_images(src_dir)
    if not paths:
        print(f"[skip] {label}: no images in {src_dir}")
        return 0

    first = cv2.imread(str(paths[0]), cv2.IMREAD_COLOR)
    if first is None:
        raise RuntimeError(f"failed to read {paths[0]}")
    h, w = first.shape[:2]
    writer = open_writer(out_path, fps, (w, h))
    try:
        for i, p in enumerate(paths):
            frame = cv2.imread(str(p), cv2.IMREAD_COLOR)
            if frame is None:
                print(f"[warn] skip unreadable {p}")
                continue
            if frame.shape[0] != h or frame.shape[1] != w:
                frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
            writer.write(frame)
            if (i + 1) % 200 == 0 or i + 1 == len(paths):
                print(f"[{label}] {i + 1}/{len(paths)} -> {out_path}")
    finally:
        writer.release()
    return len(paths)


def write_depth(src_dir: Path, out_path: Path, fps: float,
                lo: float | None, hi: float | None) -> int:
    paths = list_images(src_dir)
    if not paths:
        print(f"[skip] depth: no images in {src_dir}")
        return 0

    if lo is None or hi is None:
        est_lo, est_hi = estimate_depth_range(paths)
        lo = est_lo if lo is None else lo
        hi = est_hi if hi is None else hi
    if hi <= lo:
        hi = lo + 1.0
    print(f"[depth] colormap range mm: [{lo:.1f}, {hi:.1f}] (shallow=blue .. deep=red)")

    first = cv2.imread(str(paths[0]), cv2.IMREAD_UNCHANGED)
    if first is None:
        raise RuntimeError(f"failed to read {paths[0]}")
    h, w = first.shape[:2]
    writer = open_writer(out_path, fps, (w, h))
    try:
        for i, p in enumerate(paths):
            depth = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
            if depth is None:
                print(f"[warn] skip unreadable {p}")
                continue
            if depth.shape[0] != h or depth.shape[1] != w:
                depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
            frame = colorize_depth(depth, lo, hi)
            writer.write(frame)
            if (i + 1) % 200 == 0 or i + 1 == len(paths):
                print(f"[depth] {i + 1}/{len(paths)} -> {out_path}")
    finally:
        writer.release()
    return len(paths)


def parse_args() -> argparse.Namespace:
    default_data = Path.home() / "pika_ros" / "data"
    default_see = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Export color/depth/fisheye folders of an episode to MP4 videos.")
    parser.add_argument(
        "episode",
        nargs="?",
        default="001",
        help="episode folder name under data/ (default: 001)",
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        default=default_data,
        help=f"data root containing episodes (default: {default_data})",
    )
    parser.add_argument(
        "--see-root",
        type=Path,
        default=default_see,
        help=f"output root (default: {default_see})",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="output FPS (default: 30)")
    parser.add_argument(
        "--depth-min-mm",
        type=float,
        default=None,
        help="shallow end of colormap in mm (default: 1%% of valid depths)",
    )
    parser.add_argument(
        "--depth-max-mm",
        type=float,
        default=None,
        help="deep end of colormap in mm (default: 99%% of valid depths)",
    )
    parser.add_argument(
        "--only",
        choices=("color", "depth", "fisheye", "all"),
        default="all",
        help="which video(s) to generate",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data_root = args.data_root.expanduser().resolve()
    see_root = args.see_root.expanduser().resolve()
    episode_dir = data_root / args.episode
    out_dir = see_root / args.episode

    if not episode_dir.is_dir():
        print(f"episode not found: {episode_dir}", file=sys.stderr)
        return 1

    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"episode : {episode_dir}")
    print(f"output  : {out_dir}")
    print(f"fps     : {args.fps}")

    n = 0
    if args.only in ("color", "all"):
        n += write_color_or_fisheye(
            episode_dir / "color", out_dir / "color.mp4", args.fps, "color")
    if args.only in ("depth", "all"):
        n += write_depth(
            episode_dir / "depth",
            out_dir / "depth.mp4",
            args.fps,
            args.depth_min_mm,
            args.depth_max_mm,
        )
    if args.only in ("fisheye", "all"):
        n += write_color_or_fisheye(
            episode_dir / "fisheye", out_dir / "fisheye.mp4", args.fps, "fisheye")

    if n == 0:
        print("no frames written", file=sys.stderr)
        return 1
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
