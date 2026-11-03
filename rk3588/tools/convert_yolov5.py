#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from rknn.api import RKNN


PROJECT_DIR = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert the official YOLOv5 ONNX model for RK3588"
    )
    parser.add_argument(
        "--onnx",
        type=Path,
        default=PROJECT_DIR / "models" / "yolov5s_relu.onnx",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_DIR / "models" / "yolov5s_relu_rk3588.rknn",
    )
    return parser.parse_args()


def require_ok(result: int, operation: str) -> None:
    if result != 0:
        raise RuntimeError(f"{operation} failed: {result}")


def main() -> int:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rknn = RKNN(verbose=True)
    try:
        require_ok(
            rknn.config(
                mean_values=[[0, 0, 0]],
                std_values=[[255, 255, 255]],
                target_platform="rk3588",
            ),
            "config",
        )
        require_ok(rknn.load_onnx(model=str(args.onnx)), "load_onnx")
        require_ok(
            rknn.build(do_quantization=False),
            "build_fp16",
        )
        require_ok(rknn.export_rknn(str(args.output)), "export_rknn")
    finally:
        rknn.release()
    print(f"RKNN_MODEL_READY {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
