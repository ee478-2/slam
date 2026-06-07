#!/usr/bin/env python3
"""Train an Ultralytics YOLO storefront detector."""

import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, help="Ultralytics data.yaml")
    parser.add_argument("--model", default="yolo11s.pt", help="Initial weights")
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--project", default="runs/storefront_yolo")
    parser.add_argument("--name", default="yolo11s_storefront")
    parser.add_argument("--seed", type=int, default=478)
    parser.add_argument("--patience", type=int, default=40)
    parser.add_argument("--cache", default="false", choices=["false", "ram", "disk"])
    parser.add_argument("--exist-ok", action="store_true")
    parser.add_argument("--export-onnx", action="store_true")
    parser.add_argument("--export-opset", type=int, default=12)
    return parser.parse_args()


def main():
    args = parse_args()
    cache = False if args.cache == "false" else args.cache
    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=args.project,
        name=args.name,
        seed=args.seed,
        patience=args.patience,
        cache=cache,
        cos_lr=True,
        close_mosaic=10,
        exist_ok=args.exist_ok,
    )

    save_dir = Path(results.save_dir)
    best = save_dir / "weights" / "best.pt"
    print("best weights:", best)

    if args.export_onnx:
        export_model = YOLO(str(best))
        export_model.export(
            format="onnx",
            imgsz=args.imgsz,
            opset=args.export_opset,
            simplify=True,
            dynamic=True,
        )


if __name__ == "__main__":
    main()
