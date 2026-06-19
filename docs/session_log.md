## Environment Check

Python: 3.8.10 (default, Mar 18 2025, 20:04:55) 
[GCC 9.4.0]
OpenCV: 4.2.0
NumPy: 1.17.4
Has ArUco: True
ArUco module OK

## Extracted Sample Data

- Sample image: `data/sample_images/left_sample_001.png`
- Image shape: 720 x 1280 x 3
- Camera config: `config/camera.json`
- Source bag: `~/mono_cone_data/bags/mono_cone_dev.bag`
- Image topic: `/zed2/zed_node/left/image_rect_color`
- Camera info topic: `/zed2/zed_node/left/camera_info`
- Camera frame: `zed2_left_camera_optical_frame`
- fx: 521.0756225585938
- fy: 521.0756225585938
- cx: 645.6155395507812
- cy: 355.7669982910156

## Real Bounding Box Projection Demo

Created `scripts/09_real_bbox_to_bev_from_bag.py`.

Result:
- Loaded real image and real bounding box markers from `mono_cone_dev.bag`.
- Parsed Foxglove `ImageMarkerArray` rectangle points into bounding boxes.
- Used bbox bottom-center pixels as cone ground-contact estimates.
- Projected bottom-center pixels through `config/homography_manual.yaml`.
- Generated camera + BEV debug output.

Output:
- `outputs/real_bbox_to_bev_from_bag.png`

Known limitations:
- Image and bounding boxes are not timestamp-synchronized yet.
- Manual homography uses approximate/fake world coordinates.
- Far detections clutter the visualization.
- Class/color decoding is not implemented yet.==-

## Mono vs Stereo Validation Sanity Check

Created `scripts/13_compare_mono_vs_stereo.py`.

Inputs:
- `outputs/projected_cones_manual_homography.csv`
- `outputs/stereo_cones_reference.csv`

The script matches monocular homography projections to nearest stereo cone detections at nearby timestamps and computes 2D position error.

Results using temporary manual homography:
- Mean 2D error: 0.818 m
- Median 2D error: 0.687 m
- 95th percentile 2D error: 1.454 m
- RMSE 2D error: 0.910 m
- Mean absolute X error: 0.809 m
- Mean absolute Y error: 0.051 m
- Mean absolute range error: 0.735 m

Interpretation:
The lateral position estimate is consistent, but forward distance has significant scale error due to approximate manual calibration. These results are a pipeline sanity check, not final accuracy.

## Accuracy baseline: manual homography

The live/offline monocular cone projection pipeline was validated against the existing stereo cone perception output.

Configuration:
- Homography: config/homography_manual.yaml
- ROI: enlarged manual ROI
- Reference: /stereo_cone_perception/cones
- Input detections: /stereo_cone_perception/bounding_boxes

Results:
- Projected mono cones: 4743
- Matched mono/stereo cones: 4592
- Match rate: 96.8%
- Mean 2D error: 0.894 m
- Median 2D error: 0.872 m
- 95th percentile 2D error: 1.684 m
- RMSE 2D error: 1.027 m
- Mean absolute X error: 0.814 m
- Mean absolute Y error: 0.185 m

Interpretation:
The full monocular pipeline works on the real rosbag. The main error source is forward distance X, which is expected because the current homography was created from manually guessed world coordinates.

## Homography script refactor and ROI comparison

Refactored the monocular projection and comparison scripts to support configurable input/output paths.

Updated:
- `scripts/11_export_projected_cones_csv.py`
  - added `--homography`
  - added `--output`
  - added `--disable-roi`

- `scripts/13_compare_mono_vs_stereo.py`
  - added `--mono`
  - added `--stereo`
  - added `--output`
  - added invalid NaN/inf filtering

- `scripts/17_mono_cone_ros_node.py`
  - added `--homography`
  - added `--disable-roi`

Manual homography ROI comparison:

ROI ON:
- Mono cones: 4741
- Matched cones: 3999
- Match rate: 84.3%
- Mean 2D error: 1.001 m
- RMSE 2D error: 1.115 m
- Mean absolute X error: 0.802 m
- Mean absolute Y error: 0.394 m

ROI OFF:
- Mono cones: 15150
- Matched cones: 7011
- Match rate: 46.3%
- Mean 2D error: 1.048 m
- RMSE 2D error: 1.154 m
- Mean absolute X error: 0.789 m
- Mean absolute Y error: 0.498 m

Conclusion:
ROI filtering improves robustness with the current manual homography. Without ROI filtering, more detections are projected, but many are outside the calibrated ground region and matching quality decreases.

## ArUco Calibration Progress

Started the real ArUco-based calibration path to replace the temporary manual homography.

Input calibration image:
- `data/calibration_images/team_aruco_calib.png`

Marker detection:
- Script used: `scripts/15_detect_aruco_in_image.py`
- Detected 4 ArUco markers successfully:
  - ID 0
  - ID 1
  - ID 2
  - ID 3

Detected marker corner examples:
- ID 2 around image region `u=1025–1115`, `v=518–549`
- ID 1 around image region `u=228–306`, `v=512–539`
- ID 3 around image region `u=1128–1263`, `v=607–660`
- ID 0 around image region `u=99–212`, `v=592–636`

Team marker layout:
- Marker size: `0.20 m`
- Original layout was provided in millimeters.
- Converted to meters and stored in `config/aruco_markers.yaml`.

Current marker center configuration:
- ID 1 center: `[0.10, 0.10]`
- ID 2 center: `[2.85, 0.10]`
- ID 0 center: `[0.10, 0.81]`
- ID 3 center: `[2.85, 0.81]`

Created:
- `scripts/16_compute_homography_from_aruco.py`

The script detects ArUco markers, builds image-to-world correspondences from marker corners, computes a homography, and saves:

- `config/homography_aruco.yaml`
- `outputs/aruco_homography_debug.png`

ArUco homography result:
- Number of correspondences: 16
- Mean reprojection error over all points: `0.043411 m`
- Mean reprojection error over inliers: `0.017045 m`
- Max reprojection error: `0.088356 m`
- Inliers: `8 / 16`

Interpretation:
The ArUco homography computation works and produces a metric image-to-ground mapping in the team ArUco layout frame. However, this frame is not yet confirmed to be the same as the vehicle/camera frame used by the stereo cone reference.

Current blocker:
To compare ArUco-based monocular cone positions directly against `/stereo_cone_perception/cones`, we need the transform from the ArUco layout frame to the vehicle/camera ground frame.

Missing information:
- Vehicle/camera origin relative to ID0 or the ArUco layout
- Which ArUco axis corresponds to vehicle forward
- Which ArUco axis/sign corresponds to vehicle left

Next planned step:
Add a clean ArUco-layout-to-vehicle-frame transform layer once the vehicle origin and axis convention are confirmed.