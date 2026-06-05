#!/usr/bin/env python3

import os
import cv2
import yaml
import numpy as np


MANUAL_H_PATH = os.path.expanduser(
    "~/mono_cone_perception/config/homography_manual.yaml"
)

TEAM_H_PATH = os.path.expanduser(
    "~/mono_cone_perception/config/homography_team.yaml"
)


# Some real bbox bottom-center pixels from our rosbag output
# Format: detection_name, u, v
TEST_POINTS = [
    ("near_left_1", 383, 407),
    ("near_right_1", 948, 442),
    ("near_left_2", 309, 461),
    ("near_left_3", 353, 428),
    ("far_left", 522, 309),
    ("far_right", 768, 305),
    ("right_mid_1", 875, 394),
    ("right_mid_2", 908, 412),
]


def load_homography(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    H = np.array(data["H_img_to_world"], dtype=np.float64)
    return H


def transform_point(u, v, H):
    point = np.array([[[u, v]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(point, H)
    x, y = transformed[0, 0]
    return float(x), float(y)


def main():
    print("Loading homographies...")
    H_manual = load_homography(MANUAL_H_PATH)
    H_team = load_homography(TEAM_H_PATH)

    print("\nManual H:")
    print(H_manual)

    print("\nTeam H:")
    print(H_team)

    print("\nTesting same image points with both homographies")
    print("--------------------------------------------------------------------------")
    print(
        f"{'name':15s} {'u':>5s} {'v':>5s} | "
        f"{'manual_X':>10s} {'manual_Y':>10s} | "
        f"{'team_X':>10s} {'team_Y':>10s}"
    )
    print("--------------------------------------------------------------------------")

    for name, u, v in TEST_POINTS:
        mx, my = transform_point(u, v, H_manual)
        tx, ty = transform_point(u, v, H_team)

        print(
            f"{name:15s} {u:5d} {v:5d} | "
            f"{mx:10.3f} {my:10.3f} | "
            f"{tx:10.3f} {ty:10.3f}"
        )

    print("--------------------------------------------------------------------------")
    print("\nHow to interpret:")
    print("- If team_X/team_Y are around 0–20 and -10–10, it may be metric.")
    print("- If team_X/team_Y are hundreds/thousands, it is likely BEV image pixels.")
    print("- If signs/axes look swapped, we need to understand the team coordinate convention.")


if __name__ == "__main__":
    main()