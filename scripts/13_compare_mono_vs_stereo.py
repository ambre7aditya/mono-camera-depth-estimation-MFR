#!/usr/bin/env python3

import os
import csv
import math
import argparse
from collections import defaultdict

import numpy as np


DEFAULT_MONO_CSV = os.path.expanduser(
    "~/mono_cone_perception/outputs/projected_cones_manual_homography.csv"
)

DEFAULT_STEREO_CSV = os.path.expanduser(
    "~/mono_cone_perception/outputs/stereo_cones_reference.csv"
)

DEFAULT_OUTPUT_CSV = os.path.expanduser(
    "~/mono_cone_perception/outputs/mono_vs_stereo_comparison.csv"
)

MAX_TIME_DIFF = 0.05
MAX_MATCH_DIST = 2.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare monocular projected cones against stereo reference cones."
    )

    parser.add_argument(
        "--mono",
        default=DEFAULT_MONO_CSV,
        help="Path to monocular projected cones CSV.",
    )

    parser.add_argument(
        "--stereo",
        default=DEFAULT_STEREO_CSV,
        help="Path to stereo reference cones CSV.",
    )

    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_CSV,
        help="Path to output comparison CSV.",
    )

    parser.add_argument(
        "--max-time-diff",
        type=float,
        default=MAX_TIME_DIFF,
        help="Maximum allowed time difference between mono and stereo messages in seconds.",
    )

    parser.add_argument(
        "--max-match-dist",
        type=float,
        default=MAX_MATCH_DIST,
        help="Maximum allowed nearest-neighbor match distance in meters.",
    )

    return parser.parse_args()


def read_mono_csv(path):
    rows = []
    skipped_invalid = 0

    with open(path, "r") as f:
        reader = csv.DictReader(f)

        for row in reader:
            x_mono = float(row["X_mono"])
            y_mono = float(row["Y_mono"])

            if not math.isfinite(x_mono) or not math.isfinite(y_mono):
                skipped_invalid += 1
                continue

            rows.append(
                {
                    "bbox_stamp": float(row["bbox_stamp"]),
                    "detection_id": int(float(row["detection_id"])),
                    "X_mono": x_mono,
                    "Y_mono": y_mono,
                    "u_bottom_center": float(row["u_bottom_center"]),
                    "v_bottom_center": float(row["v_bottom_center"]),
                }
            )

    if skipped_invalid > 0:
        print(f"Skipped invalid mono rows: {skipped_invalid}")

    return rows


def read_stereo_csv(path):
    rows = []
    skipped_invalid = 0

    with open(path, "r") as f:
        reader = csv.DictReader(f)

        for row in reader:
            x_stereo = float(row["X_stereo"])
            y_stereo = float(row["Y_stereo"])
            z_stereo = float(row["Z_stereo"])

            if (
                not math.isfinite(x_stereo)
                or not math.isfinite(y_stereo)
                or not math.isfinite(z_stereo)
            ):
                skipped_invalid += 1
                continue

            rows.append(
                {
                    "stamp": float(row["stamp"]),
                    "frame_id": row.get("frame_id", ""),
                    "cone_id": int(float(row["cone_id"])),
                    "X_stereo": x_stereo,
                    "Y_stereo": y_stereo,
                    "Z_stereo": z_stereo,
                    "color": float(row["color"]) if row.get("color", "") != "" else math.nan,
                    "confidence": float(row["confidence"]) if row.get("confidence", "") != "" else math.nan,
                }
            )

    if skipped_invalid > 0:
        print(f"Skipped invalid stereo rows: {skipped_invalid}")

    return rows

def group_stereo_by_stamp(stereo_rows):
    grouped = defaultdict(list)

    for row in stereo_rows:
        grouped[row["stamp"]].append(row)

    sorted_stamps = sorted(grouped.keys())

    return grouped, sorted_stamps


def find_closest_stamp(target_stamp, sorted_stamps, max_time_diff):
    if not sorted_stamps:
        return None, None

    # Simple linear search is okay for this dataset size.
    closest_stamp = min(sorted_stamps, key=lambda s: abs(s - target_stamp))
    dt = abs(closest_stamp - target_stamp)

    if dt > max_time_diff:
        return None, dt

    return closest_stamp, dt


def find_nearest_stereo_cone(mono_row, stereo_candidates, used_keys, max_match_dist):
    best = None
    best_dist = None

    mx = mono_row["X_mono"]
    my = mono_row["Y_mono"]

    for stereo_row in stereo_candidates:
        key = (stereo_row["stamp"], stereo_row["cone_id"])

        if key in used_keys:
            continue

        sx = stereo_row["X_stereo"]
        sy = stereo_row["Y_stereo"]

        dist = math.sqrt((mx - sx) ** 2 + (my - sy) ** 2)

        if best is None or dist < best_dist:
            best = stereo_row
            best_dist = dist

    if best is None:
        return None, None

    if best_dist > max_match_dist:
        return None, best_dist

    return best, best_dist


def compute_metrics(matches):
    if not matches:
        return None

    error_2d = np.array([m["error_2d"] for m in matches], dtype=np.float64)
    abs_x = np.array([abs(m["error_x"]) for m in matches], dtype=np.float64)
    abs_y = np.array([abs(m["error_y"]) for m in matches], dtype=np.float64)
    range_error = np.array([m["range_error"] for m in matches], dtype=np.float64)
    abs_range_error = np.abs(range_error)

    return {
        "mean_2d": float(np.mean(error_2d)),
        "median_2d": float(np.median(error_2d)),
        "p95_2d": float(np.percentile(error_2d, 95)),
        "rmse_2d": float(math.sqrt(np.mean(error_2d ** 2))),
        "mean_abs_x": float(np.mean(abs_x)),
        "mean_abs_y": float(np.mean(abs_y)),
        "mean_range_error": float(np.mean(range_error)),
        "mean_abs_range_error": float(np.mean(abs_range_error)),
    }


def write_matches_csv(path, matches):
    output_dir = os.path.dirname(path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    fieldnames = [
        "bbox_stamp",
        "closest_stereo_stamp",
        "time_diff",
        "detection_id",
        "stereo_cone_id",
        "u_bottom_center",
        "v_bottom_center",
        "X_mono",
        "Y_mono",
        "X_stereo",
        "Y_stereo",
        "Z_stereo",
        "error_x",
        "error_y",
        "error_2d",
        "range_mono",
        "range_stereo",
        "range_error",
        "stereo_color",
        "stereo_confidence",
    ]

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for match in matches:
            writer.writerow(match)


def main():
    args = parse_args()

    mono_csv = os.path.expanduser(args.mono)
    stereo_csv = os.path.expanduser(args.stereo)
    output_csv = os.path.expanduser(args.output)

    mono_rows = read_mono_csv(mono_csv)
    stereo_rows = read_stereo_csv(stereo_csv)

    grouped_stereo, sorted_stamps = group_stereo_by_stamp(stereo_rows)

    matches = []
    used_stereo_keys = set()

    skipped_no_time_match = 0
    skipped_no_distance_match = 0

    for mono_row in mono_rows:
        closest_stamp, dt = find_closest_stamp(
            mono_row["bbox_stamp"],
            sorted_stamps,
            args.max_time_diff,
        )

        if closest_stamp is None:
            skipped_no_time_match += 1
            continue

        stereo_candidates = grouped_stereo[closest_stamp]

        stereo_match, nearest_dist = find_nearest_stereo_cone(
            mono_row,
            stereo_candidates,
            used_stereo_keys,
            args.max_match_dist,
        )

        if stereo_match is None:
            skipped_no_distance_match += 1
            continue

        used_stereo_keys.add((stereo_match["stamp"], stereo_match["cone_id"]))

        error_x = mono_row["X_mono"] - stereo_match["X_stereo"]
        error_y = mono_row["Y_mono"] - stereo_match["Y_stereo"]
        error_2d = math.sqrt(error_x ** 2 + error_y ** 2)

        range_mono = math.sqrt(mono_row["X_mono"] ** 2 + mono_row["Y_mono"] ** 2)
        range_stereo = math.sqrt(
            stereo_match["X_stereo"] ** 2 + stereo_match["Y_stereo"] ** 2
        )
        range_error = range_mono - range_stereo

        matches.append(
            {
                "bbox_stamp": mono_row["bbox_stamp"],
                "closest_stereo_stamp": closest_stamp,
                "time_diff": dt,
                "detection_id": mono_row["detection_id"],
                "stereo_cone_id": stereo_match["cone_id"],
                "u_bottom_center": mono_row["u_bottom_center"],
                "v_bottom_center": mono_row["v_bottom_center"],
                "X_mono": mono_row["X_mono"],
                "Y_mono": mono_row["Y_mono"],
                "X_stereo": stereo_match["X_stereo"],
                "Y_stereo": stereo_match["Y_stereo"],
                "Z_stereo": stereo_match["Z_stereo"],
                "error_x": error_x,
                "error_y": error_y,
                "error_2d": error_2d,
                "range_mono": range_mono,
                "range_stereo": range_stereo,
                "range_error": range_error,
                "stereo_color": stereo_match["color"],
                "stereo_confidence": stereo_match["confidence"],
            }
        )

    write_matches_csv(output_csv, matches)
    metrics = compute_metrics(matches)

    print("Compare mono vs stereo")
    print("----------------------")
    print(f"Mono CSV:                  {mono_csv}")
    print(f"Stereo CSV:                {stereo_csv}")
    print(f"Output CSV:                {output_csv}")
    print(f"Max time diff:             {args.max_time_diff:.3f} s")
    print(f"Max match distance:        {args.max_match_dist:.3f} m")
    print()
    print(f"Mono cones:                {len(mono_rows)}")
    print(f"Stereo cones:              {len(stereo_rows)}")
    print(f"Matched cones:             {len(matches)}")
    print(f"Skipped no time match:     {skipped_no_time_match}")
    print(f"Skipped no distance match: {skipped_no_distance_match}")

    if len(mono_rows) > 0:
        match_rate = 100.0 * len(matches) / len(mono_rows)
        print(f"Match rate:                {match_rate:.1f}%")

    if metrics is None:
        print("\nNo matches found. Cannot compute metrics.")
        return

    print()
    print("Error metrics")
    print("-------------")
    print(f"Mean 2D error:             {metrics['mean_2d']:.3f} m")
    print(f"Median 2D error:           {metrics['median_2d']:.3f} m")
    print(f"95th percentile 2D error:  {metrics['p95_2d']:.3f} m")
    print(f"RMSE 2D error:             {metrics['rmse_2d']:.3f} m")
    print(f"Mean absolute X error:     {metrics['mean_abs_x']:.3f} m")
    print(f"Mean absolute Y error:     {metrics['mean_abs_y']:.3f} m")
    print(f"Mean range error:          {metrics['mean_range_error']:.3f} m")
    print(f"Mean abs range error:      {metrics['mean_abs_range_error']:.3f} m")


if __name__ == "__main__":
    main()