#!/usr/bin/env python3

import os
import csv
import argparse

import cv2
import yaml
import rosbag
import numpy as np


DEFAULT_BAG_PATH = os.path.expanduser(
    "~/mono_cone_data/bags/mono_cone_dev.bag"
)

DEFAULT_HOMOGRAPHY_PATH = os.path.expanduser(
    "~/mono_cone_perception/config/homography_manual.yaml"
)

DEFAULT_OUTPUT_CSV = os.path.expanduser(
    "~/mono_cone_perception/outputs/projected_cones_manual_homography.csv"
)

BBOX_TOPIC = "/stereo_cone_perception/bounding_boxes"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export monocular cone projections from rosbag bounding boxes."
    )

    parser.add_argument(
        "--bag",
        default=DEFAULT_BAG_PATH,
        help="Path to input rosbag.",
    )

    parser.add_argument(
        "--homography",
        default=DEFAULT_HOMOGRAPHY_PATH,
        help="Path to homography YAML file.",
    )

    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_CSV,
        help="Path to output CSV file.",
    )

    parser.add_argument(
        "--disable-roi",
        action="store_true",
        help="Disable ROI filtering and project all bbox bottom-centers.",
    )

    return parser.parse_args()


def load_homography_and_roi(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    H = np.array(data["H_img_to_world"], dtype=np.float64)

    roi_polygon = None
    if "image_points" in data and data["image_points"]:
        roi_polygon = np.array(data["image_points"], dtype=np.float32)

    return H, roi_polygon


def point_inside_image_polygon(u, v, polygon_points):
    if polygon_points is None:
        return True

    result = cv2.pointPolygonTest(
        polygon_points,
        (float(u), float(v)),
        False,
    )

    return result >= 0


def bbox_from_marker(marker):
    if len(marker.points) < 4:
        return None

    xs = [p.x for p in marker.points]
    ys = [p.y for p in marker.points]

    x_min = min(xs)
    x_max = max(xs)
    y_min = min(ys)
    y_max = max(ys)

    u = 0.5 * (x_min + x_max)
    v = y_max

    return x_min, y_min, x_max, y_max, u, v


def image_point_to_world(u, v, H):
    point = np.array([[[u, v]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(point, H)
    X, Y = transformed[0, 0]
    return float(X), float(Y)


def main():
    args = parse_args()

    bag_path = os.path.expanduser(args.bag)
    homography_path = os.path.expanduser(args.homography)
    output_csv = os.path.expanduser(args.output)

    H_img_to_world, roi_polygon = load_homography_and_roi(homography_path)

    output_dir = os.path.dirname(output_csv)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    total_bbox_messages = 0
    total_markers_seen = 0
    exported_count = 0
    skipped_outside_roi = 0
    skipped_invalid_markers = 0

    with open(output_csv, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)

        writer.writerow(
            [
                "bbox_stamp",
                "detection_id",
                "x_min",
                "y_min",
                "x_max",
                "y_max",
                "u_bottom_center",
                "v_bottom_center",
                "X_mono",
                "Y_mono",
                "inside_roi",
                "homography_path",
            ]
        )

        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=[BBOX_TOPIC]):
                total_bbox_messages += 1
                bbox_stamp = t.to_sec()

                for detection_id, marker in enumerate(msg.markers):
                    total_markers_seen += 1

                    bbox = bbox_from_marker(marker)
                    if bbox is None:
                        skipped_invalid_markers += 1
                        continue

                    x_min, y_min, x_max, y_max, u, v = bbox

                    inside_roi = True
                    if not args.disable_roi:
                        inside_roi = point_inside_image_polygon(u, v, roi_polygon)

                    if not inside_roi:
                        skipped_outside_roi += 1
                        continue

                    X_mono, Y_mono = image_point_to_world(u, v, H_img_to_world)

                    writer.writerow(
                        [
                            bbox_stamp,
                            detection_id,
                            x_min,
                            y_min,
                            x_max,
                            y_max,
                            u,
                            v,
                            X_mono,
                            Y_mono,
                            inside_roi,
                            homography_path,
                        ]
                    )

                    exported_count += 1

    print("Export projected cones")
    print("----------------------")
    print(f"Bag path:                 {bag_path}")
    print(f"Bounding box topic:       {BBOX_TOPIC}")
    print(f"Homography path:          {homography_path}")
    print(f"Output CSV:               {output_csv}")
    print(f"ROI filtering disabled:   {args.disable_roi}")
    print()
    print(f"Total bbox messages:      {total_bbox_messages}")
    print(f"Total markers seen:       {total_markers_seen}")
    print(f"Exported projected cones: {exported_count}")
    print(f"Skipped outside ROI:      {skipped_outside_roi}")
    print(f"Skipped invalid markers:  {skipped_invalid_markers}")


if __name__ == "__main__":
    main()
