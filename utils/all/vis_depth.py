#!/usr/bin/env python3
"""临时脚本：将 16 位灰度深度图可视化为伪彩色（0~1000mm 线性渐变）。

颜色映射：0mm → 蓝，1000mm → 红，中间平滑过渡（JET 色带）。
有效像素>0 映射，0 值保持黑色。
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def depth_to_jet_index(depth_mm: np.ndarray, max_mm: float = 1000.0) -> np.ndarray:
    """将 0~max_mm 线性映射到 JET 色带的 0-255 索引，0 值保持 0（黑）。"""
    d = depth_mm.astype(np.float32)
    idx = np.zeros_like(d, dtype=np.float32)

    valid = d > 0
    if np.any(valid):
        # 截断到 max_mm，避免超出
        clipped = np.clip(d[valid], 0, max_mm)
        idx[valid] = (clipped / max_mm) * 255.0

    return np.clip(idx, 0, 255).astype(np.uint8)


def visualize_depth(path: Path, out: Path | None = None, show: bool = True, max_mm: float = 1000.0):
    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if depth is None:
        print(f"无法读取: {path}", file=sys.stderr)
        return 1

    print(f"file: {path}")
    print(f"shape={depth.shape} dtype={depth.dtype} min={depth.min()} max={depth.max()} mean={depth.mean():.1f}")
    nz = depth[depth > 0]
    if nz.size:
        print(f"nonzero: min={nz.min()} median={np.median(nz):.0f} p95={np.percentile(nz, 95):.0f} max={nz.max()}")
        print(f"nonzero%={100.0 * nz.size / depth.size:.1f}%")
    print(f"colormap: 0-{max_mm:.0f}mm 线性渐变 (蓝→红)，0=黑")

    vis8 = depth_to_jet_index(depth, max_mm)
    color = cv2.applyColorMap(vis8, cv2.COLORMAP_JET)
    color[depth == 0] = 0  # 无效像素保持黑

    if out is None:
        out = path.with_name(path.stem + "_vis.jpg")
    cv2.imwrite(str(out), color)
    print(f"saved: {out}")

    if show:
        cv2.namedWindow("depth_vis", cv2.WINDOW_NORMAL)
        cv2.imshow("depth_vis", color)
        print("按任意键关闭窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="可视化 16 位深度 PNG（0~1000mm 线性渐变，蓝→红）"
    )
    ap.add_argument(
        "image",
        nargs="?",
        default="/home/vector/pika_ros/data/012/depth/depth_20260727_170005_568671.png",
        help="深度图路径",
    )
    ap.add_argument("--out", type=Path, default=None, help="输出伪彩色图路径")
    ap.add_argument("--no-show", action="store_true", help="只保存不弹窗")
    ap.add_argument("--max-mm", type=float, default=1000.0, help="最大距离(毫米)，默认1000")
    args = ap.parse_args()
    return visualize_depth(Path(args.image), out=args.out, show=not args.no_show, max_mm=args.max_mm)


if __name__ == "__main__":
    raise SystemExit(main())