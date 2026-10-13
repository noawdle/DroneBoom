#!/usr/bin/env python3
"""Detect GNSS anomalies from GNSS/Gazebo residuals in a processed CSV.

The detector uses only the measured GNSS and Gazebo ENU columns.  The scenario
label is read solely after detection to calculate evaluation metrics.
"""

import argparse
import csv
import math
import statistics
from pathlib import Path


DEFAULT_SOURCE = Path("data/processed/spoofed_flight_20261013_193740.csv")
PERSISTENCE_SAMPLES = 3


def percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summary(values):
    return {
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "std": statistics.pstdev(values),
        "max": max(values),
        "p95": percentile(values, 0.95),
    }


def finite_value(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def residual(row, east_field, north_field, up_field=None):
    east = finite_value(row, east_field)
    north = finite_value(row, north_field)
    true_east = finite_value(row, "true_east")
    true_north = finite_value(row, "true_north")
    if None in (east, north, true_east, true_north):
        return None
    horizontal = math.hypot(east - true_east, north - true_north)
    if up_field is None:
        return horizontal
    up = finite_value(row, up_field)
    true_up = finite_value(row, "true_up")
    if None in (up, true_up):
        return horizontal, None
    return horizontal, math.sqrt(horizontal * horizontal + (up - true_up) ** 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--persistence", type=int, default=PERSISTENCE_SAMPLES)
    args = parser.parse_args()
    if args.persistence < 1:
        parser.error("--persistence must be at least 1")
    if not args.source.is_file():
        parser.error(f"source CSV not found: {args.source}")

    with args.source.open(newline="", encoding="utf-8") as source_file:
        reader = csv.DictReader(source_file)
        source_fields = reader.fieldnames or []
        rows = list(reader)
    required = {"gps_east", "gps_north", "gps_up", "true_east", "true_north", "true_up", "spoofed_gps_east", "spoofed_gps_north", "spoofed_gps_up"}
    missing = sorted(required - set(source_fields))
    if missing:
        parser.error("missing required columns: " + ", ".join(missing))

    # Calibration uses the original GNSS measurement, not a scenario label,
    # zone, or injected offset. In deployment this corresponds to a known-good
    # baseline flight or calibration period.
    normal_residuals = [
        value
        for row in rows
        for value in [residual(row, "gps_east", "gps_north")]
        if value is not None
    ]
    if not normal_residuals:
        parser.error("no valid GNSS/Gazebo baseline residuals")
    baseline = summary(normal_residuals)
    warning_threshold = baseline["mean"] + 3.0 * baseline["std"]
    # A Gaussian estimate alone can sit just below a real, already-observed
    # normal outlier. Keep a 25% margin above the calibration maximum; this is
    # derived from the baseline rather than from the spoofing scenario.
    suspect_threshold = max(
        baseline["mean"] + 5.0 * baseline["std"],
        baseline["max"] * 1.25,
    )

    consecutive_suspect = 0
    spoofed_residuals = []
    for row in rows:
        values = residual(row, "spoofed_gps_east", "spoofed_gps_north", "spoofed_gps_up")
        if values is None:
            row["horizontal_residual"] = ""
            row["position_error_3d"] = ""
            row["detection_state"] = "NORMAL"
            row["detected_spoofing"] = 0
            consecutive_suspect = 0
            continue

        horizontal, three_d = values
        spoofed_residuals.append(horizontal)
        row["horizontal_residual"] = horizontal
        row["position_error_3d"] = "" if three_d is None else three_d

        # No scenario label, zone geometry, or injected offset is used here.
        if horizontal >= suspect_threshold:
            consecutive_suspect += 1
            if consecutive_suspect >= args.persistence:
                state, detected = "SPOOFING_SUSPECTED", 1
            else:
                state, detected = "OBSERVE", 0
        elif horizontal >= warning_threshold:
            consecutive_suspect = 0
            state, detected = "OBSERVE", 0
        else:
            consecutive_suspect = 0
            state, detected = "NORMAL", 0
        row["detection_state"] = state
        row["detected_spoofing"] = detected

    # Labels are intentionally used only after detection for evaluation.
    tp = fp = tn = fn = 0
    labels_available = "scenario_spoof_active" in source_fields
    if labels_available:
        for row in rows:
            actual = finite_value(row, "scenario_spoof_active")
            predicted = int(row["detected_spoofing"])
            if actual is None:
                continue
            if actual >= 0.5 and predicted:
                tp += 1
            elif actual >= 0.5:
                fn += 1
            elif predicted:
                fp += 1
            else:
                tn += 1

    output_name = args.source.name
    if output_name.startswith("spoofed_"):
        output_name = "detected_" + output_name[len("spoofed_"):]
    else:
        output_name = "detected_" + output_name
    output_path = Path("data/processed") / output_name
    output_path.parent.mkdir(parents=True, exist_ok=True)
    appended_fields = ["horizontal_residual", "position_error_3d", "detection_state", "detected_spoofing"]
    fieldnames = source_fields + [field for field in appended_fields if field not in source_fields]
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    spoofed_summary = summary(spoofed_residuals)
    detected_count = sum(int(row["detected_spoofing"]) for row in rows)
    print(f"Source: {args.source}")
    print(f"Output: {output_path}")
    print(f"Samples: {len(rows)}")
    print("Normal horizontal baseline [m]: " + ", ".join(f"{key}={value:.4f}" for key, value in baseline.items()))
    print(
        f"Thresholds [m]: warning={warning_threshold:.4f} (mean+3std), "
        f"suspect={suspect_threshold:.4f} (max of mean+5std and 1.25*normal_max), "
        f"persistence={args.persistence} samples"
    )
    print("Spoofed horizontal residual [m]: " + ", ".join(f"{key}={value:.4f}" for key, value in spoofed_summary.items()))
    print(f"Detected spoofing samples: {detected_count}")
    if labels_available:
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        accuracy = (tp + tn) / (tp + tn + fp + fn) if tp + tn + fp + fn else 0.0
        print(f"Confusion matrix: TP={tp}, FP={fp}, TN={tn}, FN={fn}")
        print(f"Metrics: precision={precision:.4f}, recall={recall:.4f}, accuracy={accuracy:.4f}")


if __name__ == "__main__":
    main()
