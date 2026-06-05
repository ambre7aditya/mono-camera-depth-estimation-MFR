#!/usr/bin/env python3

import os
import cv2
import numpy as np


# For now, test on the sample ZED image.
# Later, replace this with a calibration image containing printed ArUco markers.
IMAGE_PATH = os.path.expanduser(
    "~/mono_cone_perception/data/sample_images/left_sample_001.png"
)

OUTPUT_PATH = os.path.expanduser(
    "~/mono_cone_perception/outputs/aruco_detection_debug.png"
)

ARUCO_DICT_NAME = cv2.aruco.DICT_4X4_50


def get_aruco_detector():
    """
    OpenCV 4.2 uses the older ArUco API:
      Dictionary_get()
      DetectorParameters_create()
      detectMarkers()
    """
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT_NAME)
    parameters = cv2.aruco.DetectorParameters_create()
    return aruco_dict, parameters


def main():
    image = cv2.imread(IMAGE_PATH)

    if image is None:
        raise FileNotFoundError(f"Could not load image: {IMAGE_PATH}")

    aruco_dict, parameters = get_aruco_detector()

    corners, ids, rejected = cv2.aruco.detectMarkers(
        image,
        aruco_dict,
        parameters=parameters,
    )

    debug_image = image.copy()

    print("ArUco detection result")
    print("----------------------")

    if ids is None:
        print("No ArUco markers detected.")
        print("This is expected if the current sample image has no printed markers.")
    else:
        print(f"Detected {len(ids)} marker(s).")

        cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            print(f"\nMarker ID: {marker_id}")

            # marker_corners shape is (1, 4, 2)
            pts = marker_corners[0]

            for i, (u, v) in enumerate(pts):
                print(f"  corner {i}: u={u:.1f}, v={v:.1f}")

                cv2.circle(debug_image, (int(u), int(v)), 5, (0, 0, 255), -1)
                cv2.putText(
                    debug_image,
                    f"{marker_id}:{i}",
                    (int(u) + 6, int(v) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                )

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    cv2.imwrite(OUTPUT_PATH, debug_image)
    print(f"\nSaved debug image to: {OUTPUT_PATH}")

    cv2.namedWindow("ArUco Detection Debug", cv2.WINDOW_NORMAL)
    cv2.imshow("ArUco Detection Debug", debug_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()