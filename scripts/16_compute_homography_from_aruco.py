#!/usr/bin/env python3

import os
import math
import cv2
import yaml
import numpy as np


CALIBRATION_IMAGE_PATH = os.path.expanduser(
    "~/mono_cone_perception/data/calibration_images/team_aruco_calib.png"
)

ARUCO_CONFIG_PATH = os.path.expanduser(
    "~/mono_cone_perception/config/aruco_markers.yaml"
)

OUTPUT_HOMOGRAPHY_PATH = os.path.expanduser(
    "~/mono_cone_perception/config/homography_aruco.yaml"
)

OUTPUT_DEBUG_IMAGE_PATH = os.path.expanduser(
    "~/mono_cone_perception/outputs/aruco_homography_debug.png"
)

ARUCO_DICT_NAME = cv2.aruco.DICT_4X4_50


def load_aruco_config(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def compute_marker_world_corners(center, marker_size_m, yaw_deg):
    """
    Compute the 4 world corners of a square ArUco marker.

    The detected OpenCV ArUco corner order is:
      0: top-left
      1: top-right
      2: bottom-right
      3: bottom-left

    This function returns world corners in the same order.

    Coordinate convention for current config:
      X/Y are in the team ArUco layout frame, meters.
    """
    cx, cy = center
    half = marker_size_m / 2.0

    # Local corners in OpenCV ArUco order:
    # top-left, top-right, bottom-right, bottom-left
    local_corners = np.array(
        [
            [-half, -half],
            [ half, -half],
            [ half,  half],
            [-half,  half],
        ],
        dtype=np.float32,
    )

    yaw = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    R = np.array(
        [
            [cos_yaw, -sin_yaw],
            [sin_yaw,  cos_yaw],
        ],
        dtype=np.float32,
    )

    rotated = local_corners @ R.T
    world_corners = rotated + np.array([cx, cy], dtype=np.float32)

    return world_corners


def detect_aruco_markers(image):
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT_NAME)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected = cv2.aruco.detectMarkers(
        image,
        aruco_dict,
        parameters=parameters,
    )

    return corners, ids, rejected


def draw_world_corner_labels(debug_image, img_corners, world_corners, marker_id):
    for idx in range(4):
        u, v = img_corners[idx]
        X, Y = world_corners[idx]

        cv2.circle(debug_image, (int(u), int(v)), 5, (0, 0, 255), -1)

        label = f"{marker_id}:{idx} ({X:.2f},{Y:.2f})"
        cv2.putText(
            debug_image,
            label,
            (int(u) + 6, int(v) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
        )


def main():
    image = cv2.imread(CALIBRATION_IMAGE_PATH)

    if image is None:
        raise FileNotFoundError(
            f"Could not load calibration image: {CALIBRATION_IMAGE_PATH}"
        )

    config = load_aruco_config(ARUCO_CONFIG_PATH)

    marker_size_m = float(config["marker_size_m"])
    marker_config = config["markers"]

    corners, ids, rejected = detect_aruco_markers(image)

    debug_image = image.copy()

    print("ArUco homography calibration")
    print("----------------------------")
    print(f"Calibration image: {CALIBRATION_IMAGE_PATH}")
    print(f"ArUco config:      {ARUCO_CONFIG_PATH}")
    print(f"Marker size:       {marker_size_m} m")

    if ids is None or len(ids) == 0:
        print("\nNo ArUco markers detected.")
        os.makedirs(os.path.dirname(OUTPUT_DEBUG_IMAGE_PATH), exist_ok=True)
        cv2.imwrite(OUTPUT_DEBUG_IMAGE_PATH, debug_image)
        print(f"Saved debug image to: {OUTPUT_DEBUG_IMAGE_PATH}")
        return

    cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

    image_points = []
    world_points = []
    used_marker_ids = []

    print(f"\nDetected {len(ids)} marker(s):")

    for marker_corners, marker_id_array in zip(corners, ids):
        marker_id = int(marker_id_array[0])
        print(f"  marker ID: {marker_id}")

        if marker_id not in marker_config:
            print(f"    skipped: marker ID {marker_id} not in config")
            continue

        marker_info = marker_config[marker_id]
        center = marker_info["center"]
        yaw_deg = float(marker_info.get("yaw_deg", 0.0))

        img_corners = marker_corners[0].astype(np.float32)

        world_corners = compute_marker_world_corners(
            center=center,
            marker_size_m=marker_size_m,
            yaw_deg=yaw_deg,
        )

        print(f"    center: {center}, yaw: {yaw_deg}")
        for i in range(4):
            u, v = img_corners[i]
            X, Y = world_corners[i]
            print(
                f"    corner {i}: image=({u:.1f}, {v:.1f}) "
                f"world=({X:.3f}, {Y:.3f})"
            )

            image_points.append([u, v])
            world_points.append([X, Y])

        draw_world_corner_labels(debug_image, img_corners, world_corners, marker_id)
        used_marker_ids.append(marker_id)

    image_points = np.array(image_points, dtype=np.float32)
    world_points = np.array(world_points, dtype=np.float32)

    print(f"\nUsed marker IDs: {used_marker_ids}")
    print(f"Number of correspondences: {len(image_points)}")

    if len(image_points) < 4:
        raise RuntimeError("Need at least 4 point correspondences.")

    H_img_to_world, mask = cv2.findHomography(
        image_points,
        world_points,
        method=cv2.RANSAC,
        ransacReprojThreshold=0.05,
    )

    if H_img_to_world is None:
        raise RuntimeError("cv2.findHomography failed.")

    projected_world = cv2.perspectiveTransform(
        image_points.reshape(-1, 1, 2),
        H_img_to_world,
    ).reshape(-1, 2)

    errors = np.linalg.norm(projected_world - world_points, axis=1)

    if mask is not None:
        inlier_mask = mask.ravel().astype(bool)
    else:
        inlier_mask = np.ones(len(errors), dtype=bool)

    inlier_errors = errors[inlier_mask]

    print("\nComputed H_img_to_world:")
    print(H_img_to_world)

    print("\nReprojection error:")
    print(f"  mean all points:     {np.mean(errors):.6f} m")
    print(f"  mean inlier points:  {np.mean(inlier_errors):.6f} m")
    print(f"  max all points:      {np.max(errors):.6f} m")
    print(f"  inliers:             {np.sum(inlier_mask)} / {len(errors)}")

    # Use all marker corner image points as ROI polygon approximation.
    # For this rectangular layout, create a simple outer polygon from known marker IDs.
    # This is only for filtering/debugging, not the homography math.
    image_points_for_roi = image_points.tolist()

    output_data = {
        "description": "ArUco-based image-to-ground homography calibration using team marker layout.",
        "calibration_image": CALIBRATION_IMAGE_PATH,
        "aruco_config": ARUCO_CONFIG_PATH,
        "coordinate_frame": {
            "unit": "meters",
            "note": "Current output is in the team ArUco layout frame, not necessarily vehicle frame.",
            "x": "team layout X axis",
            "y": "team layout Y axis",
        },
        "used_marker_ids": used_marker_ids,
        "marker_size_m": marker_size_m,
        "image_points": image_points_for_roi,
        "world_points": world_points.tolist(),
        "H_img_to_world": H_img_to_world.tolist(),
        "reprojection_error_m": {
            "mean_all": float(np.mean(errors)),
            "mean_inliers": float(np.mean(inlier_errors)),
            "max_all": float(np.max(errors)),
            "num_inliers": int(np.sum(inlier_mask)),
            "num_points": int(len(errors)),
        },
    }

    os.makedirs(os.path.dirname(OUTPUT_HOMOGRAPHY_PATH), exist_ok=True)
    with open(OUTPUT_HOMOGRAPHY_PATH, "w") as f:
        yaml.dump(output_data, f, default_flow_style=False)

    os.makedirs(os.path.dirname(OUTPUT_DEBUG_IMAGE_PATH), exist_ok=True)
    cv2.imwrite(OUTPUT_DEBUG_IMAGE_PATH, debug_image)

    print(f"\nSaved ArUco homography to: {OUTPUT_HOMOGRAPHY_PATH}")
    print(f"Saved debug image to:      {OUTPUT_DEBUG_IMAGE_PATH}")

    cv2.namedWindow("ArUco Homography Debug", cv2.WINDOW_NORMAL)
    cv2.imshow("ArUco Homography Debug", debug_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()