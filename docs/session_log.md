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