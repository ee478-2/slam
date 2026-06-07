# Storefront YOLO Dataset And Training

This directory contains only training utilities. Do not commit collected images,
labels, runs, exported engines, or weights; `.gitignore` excludes the generated
paths and model artifacts.

## Recommendation

Use **CVAT** for labeling and export as YOLO bounding boxes. It is the best fit
here because it is self-hostable, works well for many camera frames, and exports
YOLO box annotations directly.

Start with one class:

```text
storefront
```

Do not start with one class per store unless you already have enough examples
per store. For SLAM, the detector should first answer "is there a storefront
structure in this image region?" Landmark association can then use geometry,
global-map priors, camera bearing, and gating. Add per-store or per-category
classes later only if the generic detector is stable.

Model choice:

- Train first with `yolo11s.pt` at `imgsz=640`.
- If Jetson inference is too slow, retrain or distill to `yolo11n.pt`.
- Try `yolo11m.pt` only after the dataset is large and latency is acceptable.
- Use ordinary detection boxes first. OBB/segmentation adds labeling cost and is
  not needed until box extent is the limiting error.

## Collect Images On The Robot

On the Jetson:

```bash
source ~/catkin_ws/src/slam/scripts/slam_aliases.sh
slam collect-cam
```

In another real terminal:

```bash
slam teleop
```

The collector writes by default to:

```text
~/catkin_ws/src/slam/data/storefront_yolo/raw/<session>/
  images/
  classes.txt
  metadata.csv
```

Useful overrides:

```bash
SLAM_STORE_YOLO_HZ=1.0 slam collect
SLAM_STORE_YOLO_SESSION=aisle_slow_01 slam collect
SLAM_STORE_YOLO_RAW=/media/usb/storefront_raw slam collect
```

`slam collect-cam` starts only ROS environment setup, the D435 camera, and the
image collector. It does not start RTAB-Map, AprilTag detection, wheel odometry,
or the localization manager. If `/odom` is not running, the collector still
saves frames and leaves the odometry metadata columns blank.

Capture guidance:

- Drive slowly; avoid fast yaw because the camera is 640x480 at 15 fps.
- Capture each storefront from straight-on, oblique, near, far, left-to-right,
  and right-to-left passes.
- Include negative frames: blank walls, shelves, floor, robot arm, people, and
  partial storefront views.
- Keep validation images from a different driving pass than training images.
- For an initial detector, a few hundred labeled frames is enough to test the
  pipeline; expect to expand after seeing false positives/negatives.

## Label In CVAT

Create an image task with label `storefront`. Draw one tight bounding box around
the visible storefront/signboard structure that should become a semantic
landmark. Export the task as YOLO annotations.

If a frame has no storefront, keep it in the task without boxes. When preparing
the dataset, pass `--copy-empty-labels` so those frames become YOLO negative
examples.

## Prepare Dataset On The RTX 4080 Server

Install dependencies in a Python environment:

```bash
pip install -r training/storefront_yolo/requirements.txt
```

Convert the CVAT YOLO export:

```bash
python training/storefront_yolo/prepare_cvat_yolo.py \
  --input /data/cvat/storefront_yolo_export.zip \
  --output /data/storefront_yolo/dataset \
  --classes storefront \
  --copy-empty-labels
```

The output will contain `/data/storefront_yolo/dataset/data.yaml`.

## Train

```bash
python training/storefront_yolo/train_yolo.py \
  --data /data/storefront_yolo/dataset/data.yaml \
  --model yolo11s.pt \
  --epochs 150 \
  --imgsz 640 \
  --batch 16 \
  --device 0 \
  --project /data/storefront_yolo/runs \
  --name yolo11s_storefront
```

If memory allows on the RTX 4080, increase `--batch` to 24 or 32. Keep the first
run conservative so failures are data-quality issues, not OOM.

## Export

ONNX is portable:

```bash
python training/storefront_yolo/export_yolo.py \
  --weights /data/storefront_yolo/runs/yolo11s_storefront/weights/best.pt \
  --format onnx \
  --dynamic
```

TensorRT engines should be built on the deployment GPU/Jetson, not copied from
the 4080 server:

```bash
python training/storefront_yolo/export_yolo.py \
  --weights best.pt \
  --format engine \
  --half
```
