#!/usr/bin/env python3

import os
import cv2


OUTPUT_DIR = os.path.expanduser(
    "~/mono_cone_perception/outputs/aruco_markers"
)

ARUCO_DICT_NAME = cv2.aruco.DICT_4X4_50
MARKER_IDS = [0, 1, 2, 3]
MARKER_SIZE_PX = 600


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT_NAME)

    for marker_id in MARKER_IDS:
        marker_img = cv2.aruco.drawMarker(
            aruco_dict,
            marker_id,
            MARKER_SIZE_PX
        )

        output_path = os.path.join(
            OUTPUT_DIR,
            f"aruco_4x4_id_{marker_id}.png"
        )

        cv2.imwrite(output_path, marker_img)
        print(f"Saved marker ID {marker_id} to {output_path}")

    print("\nDone.")
    print("Print these four markers for calibration.")
    print("Keep the physical printed marker size known, for example 20 cm x 20 cm.")


if __name__ == "__main__":
    main()