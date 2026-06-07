#!/usr/bin/env python3
"""Export trained Ultralytics YOLO weights for deployment."""

import argparse

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", required=True, help="Path to best.pt")
    parser.add_argument(
        "--format",
        default="onnx",
        choices=["onnx", "engine", "torchscript"],
        help="Export format. Build TensorRT engine on the deployment GPU.",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--half", action="store_true")
    parser.add_argument("--dynamic", action="store_true")
    parser.add_argument("--opset", type=int, default=12)
    return parser.parse_args()


def main():
    args = parse_args()
    model = YOLO(args.weights)
    model.export(
        format=args.format,
        imgsz=args.imgsz,
        device=args.device,
        half=args.half,
        dynamic=args.dynamic,
        opset=args.opset,
        simplify=True,
    )


if __name__ == "__main__":
    main()
