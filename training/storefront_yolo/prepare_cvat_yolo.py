#!/usr/bin/env python3
"""
Convert a CVAT YOLO export into an Ultralytics YOLO dataset directory.

Expected output:
  dataset/
    data.yaml
    images/train/*.jpg
    images/val/*.jpg
    labels/train/*.txt
    labels/val/*.txt
"""

import argparse
import os
import random
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="CVAT YOLO zip or extracted directory")
    parser.add_argument("--output", required=True, help="Output Ultralytics dataset directory")
    parser.add_argument(
        "--classes",
        default="",
        help="Comma-separated class names. Overrides obj.names if provided.",
    )
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=478)
    parser.add_argument(
        "--copy-empty-labels",
        action="store_true",
        help="Create empty .txt files for unlabeled negative images.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove the output directory before writing.",
    )
    return parser.parse_args()


def prepare_input(path):
    src = Path(path).expanduser().resolve()
    if src.is_dir():
        return src, None
    if src.suffix.lower() != ".zip":
        raise ValueError("input must be a directory or .zip file: %s" % src)
    tmp = tempfile.TemporaryDirectory(prefix="cvat_yolo_")
    with zipfile.ZipFile(src) as archive:
        archive.extractall(tmp.name)
    return Path(tmp.name), tmp


def read_classes(root, override):
    if override:
        names = [name.strip() for name in override.split(",") if name.strip()]
        if names:
            return names
    for candidate in root.rglob("obj.names"):
        lines = [line.strip() for line in candidate.read_text().splitlines()]
        names = [line for line in lines if line]
        if names:
            return names
    for candidate in root.rglob("classes.txt"):
        lines = [line.strip() for line in candidate.read_text().splitlines()]
        names = [line for line in lines if line]
        if names:
            return names
    raise ValueError("no classes found; pass --classes storefront")


def load_subset_lists(root):
    subsets = {}
    for subset in ("train", "valid", "val"):
        normalized = set()
        for list_path in root.rglob("%s.txt" % subset):
            for line in list_path.read_text().splitlines():
                line = line.strip()
                if not line:
                    continue
                normalized.add(Path(line).as_posix())
                normalized.add(Path(line).name)
        if normalized:
            subsets["val" if subset == "valid" else subset] = normalized
    return subsets


def image_subset(image, root, subset_lists):
    rel = image.relative_to(root).as_posix()
    name = image.name
    if rel in subset_lists.get("val", set()) or name in subset_lists.get("val", set()):
        return "val"
    if rel in subset_lists.get("train", set()) or name in subset_lists.get("train", set()):
        return "train"
    lowered = {part.lower() for part in image.parts}
    if "obj_valid_data" in lowered or "valid" in lowered or "val" in lowered:
        return "val"
    if "obj_train_data" in lowered or "train" in lowered:
        return "train"
    return ""


def collect_images(root):
    return sorted(path for path in root.rglob("*") if path.suffix.lower() in IMAGE_EXTS)


def split_images(images, root, val_fraction, seed, subset_lists):
    explicit = {"train": [], "val": [], "": []}
    for image in images:
        explicit[image_subset(image, root, subset_lists)].append(image)

    if explicit[""] or not explicit["val"]:
        rng = random.Random(seed)
        leftovers = list(explicit[""])
        rng.shuffle(leftovers)
        val_count = max(1, int(round(len(leftovers) * val_fraction))) if leftovers else 0
        explicit["val"].extend(leftovers[:val_count])
        explicit["train"].extend(leftovers[val_count:])

    if not explicit["train"] and explicit["val"]:
        explicit["train"].append(explicit["val"].pop())

    return {"train": sorted(explicit["train"]), "val": sorted(explicit["val"])}


def unique_name(image, root):
    rel = image.relative_to(root).with_suffix("").as_posix()
    safe = rel.replace("/", "__").replace(" ", "_")
    return safe + image.suffix.lower()


def copy_split(split, root, output, copy_empty_labels):
    for subset, images in split.items():
        image_out = output / "images" / subset
        label_out = output / "labels" / subset
        image_out.mkdir(parents=True, exist_ok=True)
        label_out.mkdir(parents=True, exist_ok=True)
        for image in images:
            name = unique_name(image, root)
            dst_image = image_out / name
            shutil.copy2(image, dst_image)

            src_label = image.with_suffix(".txt")
            dst_label = label_out / (Path(name).stem + ".txt")
            if src_label.exists():
                shutil.copy2(src_label, dst_label)
            elif copy_empty_labels:
                dst_label.write_text("")


def write_data_yaml(output, classes):
    def quote_yaml(value):
        return "'" + value.replace("'", "''") + "'"

    lines = [
        "path: %s" % output.resolve().as_posix(),
        "train: images/train",
        "val: images/val",
        "names:",
    ]
    for idx, name in enumerate(classes):
        lines.append("  %d: %s" % (idx, quote_yaml(name)))
    (output / "data.yaml").write_text("\n".join(lines) + "\n")


def main():
    args = parse_args()
    root, tmp = prepare_input(args.input)
    try:
        classes = read_classes(root, args.classes)
        images = collect_images(root)
        if not images:
            raise ValueError("no images found in %s" % root)

        output = Path(args.output).expanduser().resolve()
        if output.exists() and args.overwrite:
            shutil.rmtree(output)
        if output.exists() and any(output.iterdir()):
            raise ValueError("output exists and is not empty; pass --overwrite")
        output.mkdir(parents=True, exist_ok=True)

        subsets = load_subset_lists(root)
        split = split_images(images, root, args.val_fraction, args.seed, subsets)
        copy_split(split, root, output, args.copy_empty_labels)
        write_data_yaml(output, classes)

        print("classes:", ", ".join(classes))
        print("train images:", len(split["train"]))
        print("val images:", len(split["val"]))
        print("wrote:", output / "data.yaml")
    finally:
        if tmp is not None:
            tmp.cleanup()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("error: %s" % exc, file=sys.stderr)
        sys.exit(1)
