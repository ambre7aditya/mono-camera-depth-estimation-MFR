# Monocular Cone Localization for Formula Student Driverless

This project explores a lightweight monocular cone localization pipeline for Formula Student Driverless.

The goal is to estimate cone positions using a single camera image, 2D cone bounding boxes, and an image-to-ground homography. Instead of relying directly on stereo depth or LiDAR for every cone position, this project investigates how far a monocular ground-plane projection approach can go on real vehicle data.

The current implementation runs both offline on a real ROS bag and live as a ROS node.

---

## Overview

The core pipeline is:

```text
ZED left camera image
+ cone bounding boxes
+ image-to-ground homography
→ cone X/Y ground positions
```

For each detected cone, the bottom-center of the bounding box is used as an approximation of the cone's ground contact point. This image pixel is then projected into ground-plane coordinates using a homography.

Coordinate convention used by the monocular pipeline:

```text
X = forward
Y = left
Z = 0
```

---

## Current status

The project currently supports:

- Reading real Formula Student Driverless ROS bag data
- Extracting ZED left camera images and camera calibration info
- Parsing cone bounding boxes from `/stereo_cone_perception/bounding_boxes`
- Computing bbox bottom-center points
- Projecting image points into ground coordinates using a homography
- Exporting monocular cone estimates to CSV
- Exporting stereo cone reference positions to CSV
- Comparing monocular estimates against stereo reference cones
- Running a live ROS node on rosbag playback
- Publishing monocular cone positions as a ROS `PointCloud`
- Publishing a debug image with bounding boxes, bottom-center points, ROI, and projected X/Y labels
- Generating and detecting ArUco markers
- Computing an ArUco-based homography from a real calibration image

The current manual homography is a working baseline, not final calibration. An ArUco-based homography has been computed, but the transform from the ArUco layout frame to the vehicle/camera frame still needs to be confirmed before final accuracy comparison.

---

## Example pipeline

```text
rosbag play
    ↓
/zed2/zed_node/left/image_rect_color
/stereo_cone_perception/bounding_boxes
    ↓
17_mono_cone_ros_node.py
    ↓
/mono_cone_perception/cones
/mono_cone_perception/debug_image
```

---

## ROS topics

The development ROS bag contains real vehicle data, including:

```text
/zed2/zed_node/left/image_rect_color
/zed2/zed_node/left/camera_info
/stereo_cone_perception/bounding_boxes
/stereo_cone_perception/cones
/lidar/cone_position_cloud
```

The current monocular pipeline uses:

```text
Input image:
  /zed2/zed_node/left/image_rect_color

Input 2D cone boxes:
  /stereo_cone_perception/bounding_boxes

Reference for validation:
  /stereo_cone_perception/cones
```

Important note: the localization part is monocular, but the current 2D bounding boxes are still taken from the existing stereo cone perception output. A future step is replacing this dependency with an independent monocular cone detector.

---

## Repository structure

```text
mono_cone_perception/
├── config/
│   ├── camera.json
│   ├── homography_manual.yaml
│   ├── homography_team.yaml
│   ├── homography_aruco.yaml
│   └── aruco_markers.yaml
│
├── data/
│   ├── sample_images/
│   └── calibration_images/
│
├── docs/
│   ├── project_plan.md
│   ├── data_inventory.md
│   ├── geometry_notes.md
│   └── session_log.md
│
├── outputs/
│   ├── aruco_markers/
│   ├── test_manual/
│   └── test_manual_no_roi/
│
└── scripts/
    ├── 01_extract_sample_from_bag.py
    ├── 05_save_manual_homography.py
    ├── 06_birdseye_view_demo.py
    ├── 10_real_bbox_to_bev_synced.py
    ├── 11_export_projected_cones_csv.py
    ├── 12_export_stereo_cones_csv.py
    ├── 13_compare_mono_vs_stereo.py
    ├── 14_generate_aruco_markers.py
    ├── 15_detect_aruco_in_image.py
    ├── 16_compute_homography_from_aruco.py
    └── 17_mono_cone_ros_node.py
```

---

## Why bbox bottom-center?

A homography maps points from the image plane to a ground plane. It is only valid for points that actually lie on that plane.

The center of a cone bounding box is not on the ground; it is somewhere on the cone body. The bottom-center of the bounding box is a better approximation of the cone's contact point with the ground.

For each bounding box:

```text
u = (x_min + x_max) / 2
v = y_max
```

This point is then projected using OpenCV's perspective transform.

---

## Calibration

### Manual homography baseline

The first working calibration is:

```text
config/homography_manual.yaml
```

This was created from manually selected image points and approximate world coordinates. It is useful for testing the full software pipeline, but it is not final physical calibration.

### ArUco homography

A real ArUco calibration image has been processed using:

```text
scripts/15_detect_aruco_in_image.py
scripts/16_compute_homography_from_aruco.py
```

This produced:

```text
config/homography_aruco.yaml
outputs/aruco_homography_debug.png
```

Current ArUco calibration result:

```text
Detected markers: 0, 1, 2, 3
Correspondences: 16
Mean reprojection error: 0.043411 m
Mean inlier reprojection error: 0.017045 m
Max reprojection error: 0.088356 m
```

The ArUco homography currently outputs coordinates in the team ArUco layout frame. Before using it for final stereo comparison, the transform from ArUco layout coordinates to vehicle/camera ground coordinates must be confirmed.

---

## Offline evaluation

Export monocular cone projections:

```bash
python3 scripts/11_export_projected_cones_csv.py \
  --homography config/homography_manual.yaml \
  --output outputs/test_manual/projected_cones.csv
```

Export stereo reference cones:

```bash
python3 scripts/12_export_stereo_cones_csv.py
```

Compare monocular output against stereo reference:

```bash
python3 scripts/13_compare_mono_vs_stereo.py \
  --mono outputs/test_manual/projected_cones.csv \
  --stereo outputs/stereo_cones_reference.csv \
  --output outputs/test_manual/mono_vs_stereo_comparison.csv
```

---

## Current baseline results

Using the manually initialized homography with ROI filtering:

```text
Mono cones:                4741
Matched cones:             3999
Match rate:                84.3%

Mean 2D error:             1.001 m
Median 2D error:           0.997 m
95th percentile 2D error:  1.804 m
RMSE 2D error:             1.115 m

Mean absolute X error:     0.802 m
Mean absolute Y error:     0.394 m
```

Without ROI filtering:

```text
Mono cones:                15150
Matched cones:             7011
Match rate:                46.3%

Mean 2D error:             1.048 m
RMSE 2D error:             1.154 m
Mean absolute X error:     0.789 m
Mean absolute Y error:     0.498 m
```

The ROI-filtered version is more reliable because it restricts projection to the image region where the current homography is meaningful.

---

## Live ROS node

Start ROS core:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Run the monocular cone node:

```bash
source /opt/ros/noetic/setup.bash
cd ~/mono_cone_perception

python3 scripts/17_mono_cone_ros_node.py \
  --homography config/homography_manual.yaml
```

Play the bag:

```bash
source /opt/ros/noetic/setup.bash
cd ~/mono_cone_data/bags

rosbag play --clock mono_cone_dev.bag
```

View the debug image:

```bash
source /opt/ros/noetic/setup.bash
rqt_image_view /mono_cone_perception/debug_image
```

Check published cone positions:

```bash
rostopic echo /mono_cone_perception/cones -n 1
```

The node publishes:

```text
/mono_cone_perception/debug_image
/mono_cone_perception/cones
```

Run without ROI filtering:

```bash
python3 scripts/17_mono_cone_ros_node.py \
  --homography config/homography_manual.yaml \
  --disable-roi
```

This is useful for debugging, but it may produce unstable coordinates outside the calibrated ground region.

---

## Requirements

Tested with:

```text
Ubuntu 20.04
ROS Noetic
Python 3.8
OpenCV
NumPy
PyYAML
rosbag
cv_bridge
foxglove_msgs
```

Basic ROS setup:

```bash
source /opt/ros/noetic/setup.bash
```

Install Python dependencies if needed:

```bash
pip3 install numpy pyyaml
```

Most ROS dependencies are expected to come from the ROS Noetic installation.

---

## Limitations

This project is still under development.

Current limitations:

- The manual homography is not final physical calibration
- The ArUco homography currently needs a confirmed transform into the vehicle/camera frame
- Accuracy is limited by calibration quality
- 2D bounding boxes currently come from an existing perception topic
- The system is not yet a full standalone monocular detector
- Matching against stereo reference is currently nearest-neighbor based
- No temporal tracking is implemented yet
- No final vehicle integration or Jetson optimization yet

---

## Next steps

Planned next milestones:

1. Confirm vehicle/camera origin and axis direction in the ArUco layout frame
2. Add an ArUco-layout-to-vehicle-frame transform
3. Re-run accuracy comparison using `homography_aruco.yaml`
4. Add a live bird's-eye-view debug visualization
5. Improve validation with per-frame matching and error plots
6. Convert the scripts into a proper ROS package
7. Replace existing bounding boxes with an independent monocular cone detector
8. Add cone tracking using Kalman filtering and Hungarian assignment
9. Test real-time performance on the target vehicle computer

---

## Project motivation

This project is part of a Formula Student Driverless perception exploration. The aim is not only to make a script run, but to understand the geometry, validate it on real data, and document the tradeoffs clearly.

The current version proves that a monocular image-to-ground projection pipeline can run live on real rosbag data and produce measurable cone position estimates. The next major step is completing the ArUco-to-vehicle calibration so the accuracy becomes physically meaningful in the vehicle frame.