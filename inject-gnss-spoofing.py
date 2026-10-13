#!/usr/bin/env python3
"""Create a software-only GNSS spoofing scenario from a flight CSV.

This script never changes the source CSV, Gazebo Ground Truth, or PX4.  It
converts the original GNSS samples to a local WGS84 ENU frame, adds a labelled
step offset only to a copied GNSS position, and writes a new CSV.
"""

import argparse
import csv
import math
import statistics
from pathlib import Path


WGS84_A = 6_378_137.0
WGS84_F = 1.0 / 298.257_223_563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
DEFAULT_SOURCE = Path("data/flight_20261013_193740.csv")


def geodetic_to_ecef(latitude_deg, longitude_deg, height_m):
    """Convert WGS84 geodetic coordinates to Earth-centred Cartesian metres."""

    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude * sin_latitude)
    return (
        (radius + height_m) * math.cos(latitude) * math.cos(longitude),
        (radius + height_m) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - WGS84_E2) + height_m) * sin_latitude,
    )


def ecef_delta_to_enu(delta_ecef, reference_latitude_deg, reference_longitude_deg):
    """Rotate an ECEF displacement into the WGS84 local East/North/Up frame."""

    dx, dy, dz = delta_ecef
    latitude = math.radians(reference_latitude_deg)
    longitude = math.radians(reference_longitude_deg)
    sin_latitude, cos_latitude = math.sin(latitude), math.cos(latitude)
    sin_longitude, cos_longitude = math.sin(longitude), math.cos(longitude)
    return (
        -sin_longitude * dx + cos_longitude * dy,
        -sin_latitude * cos_longitude * dx
        - sin_latitude * sin_longitude * dy
        + cos_latitude * dz,
        cos_latitude * cos_longitude * dx
        + cos_latitude * sin_longitude * dy
        + sin_latitude * dz,
    )


def percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def statistics_summary(values):
    return {
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "std": statistics.pstdev(values),
        "max": max(values),
        "p95": percentile(values, 0.95),
    }


def correlation(values_a, values_b):
    """Return the Pearson correlation used only to validate ENU axis alignment."""

    mean_a = statistics.fmean(values_a)
    mean_b = statistics.fmean(values_b)
    numerator = sum(
        (value_a - mean_a) * (value_b - mean_b)
        for value_a, value_b in zip(values_a, values_b)
    )
    denominator = math.sqrt(
        sum((value - mean_a) ** 2 for value in values_a)
        * sum((value - mean_b) ** 2 for value in values_b)
    )
    return numerator / denominator if denominator else math.nan


def require_float(row, field):
    value = row.get(field, "")
    if value in (None, ""):
        raise ValueError(f"missing required value '{field}'")
    return float(value)


def first_valid_row(rows):
    required = ("gps_lat", "gps_lon", "gps_alt", "true_x", "true_y", "true_z")
    for row in rows:
        try:
            for field in required:
                require_float(row, field)
            return row
        except ValueError:
            continue
    raise ValueError("no row contains all required GPS and Ground Truth values")


def add_enu_columns(rows):
    """Return rows with relative GNSS ENU and relative Gazebo ENU coordinates."""

    origin = first_valid_row(rows)
    origin_latitude = require_float(origin, "gps_lat")
    origin_longitude = require_float(origin, "gps_lon")
    origin_height = require_float(origin, "gps_alt")
    origin_ecef = geodetic_to_ecef(origin_latitude, origin_longitude, origin_height)
    origin_true = tuple(require_float(origin, field) for field in ("true_x", "true_y", "true_z"))

    processed = []
    for source_row in rows:
        row = dict(source_row)
        try:
            latitude = require_float(row, "gps_lat")
            longitude = require_float(row, "gps_lon")
            height = require_float(row, "gps_alt")
            current_ecef = geodetic_to_ecef(latitude, longitude, height)
            enu = ecef_delta_to_enu(
                tuple(value - reference for value, reference in zip(current_ecef, origin_ecef)),
                origin_latitude,
                origin_longitude,
            )
            true_enu = (
                require_float(row, "true_x") - origin_true[0],
                require_float(row, "true_y") - origin_true[1],
                require_float(row, "true_z") - origin_true[2],
            )
        except ValueError:
            enu = (math.nan, math.nan, math.nan)
            true_enu = (math.nan, math.nan, math.nan)

        row.update(
            gps_east=enu[0],
            gps_north=enu[1],
            gps_up=enu[2],
            true_east=true_enu[0],
            true_north=true_enu[1],
            true_up=true_enu[2],
        )
        processed.append(row)
    return processed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--east-offset", type=float, default=3.0)
    parser.add_argument("--north-offset", type=float, default=2.0)
    parser.add_argument("--start-fraction", type=float, default=0.30)
    parser.add_argument("--end-fraction", type=float, default=0.70)
    args = parser.parse_args()

    if not (0.0 <= args.start_fraction < args.end_fraction <= 1.0):
        parser.error("zone fractions must satisfy 0 <= start < end <= 1")
    if not args.source.is_file():
        parser.error(f"source CSV not found: {args.source}")

    with args.source.open(newline="", encoding="utf-8") as source_file:
        reader = csv.DictReader(source_file)
        source_fields = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        parser.error("source CSV has no rows")

    rows = add_enu_columns(rows)
    valid_rows = [
        row
        for row in rows
        if all(math.isfinite(float(row[field])) for field in ("gps_east", "gps_north", "gps_up", "true_east", "true_north", "true_up"))
    ]
    if not valid_rows:
        parser.error("source CSV has no valid converted samples")

    normal_horizontal_errors = [
        math.hypot(float(row["gps_east"]) - float(row["true_east"]), float(row["gps_north"]) - float(row["true_north"]))
        for row in valid_rows
    ]
    normal_3d_errors = [
        math.sqrt(
            (float(row["gps_east"]) - float(row["true_east"])) ** 2
            + (float(row["gps_north"]) - float(row["true_north"])) ** 2
            + (float(row["gps_up"]) - float(row["true_up"])) ** 2
        )
        for row in valid_rows
    ]

    north_values = [float(row["true_north"]) for row in valid_rows]
    east_values = [float(row["true_east"]) for row in valid_rows]
    north_min, north_max = min(north_values), max(north_values)
    north_span = north_max - north_min
    zone_north_min = north_min + args.start_fraction * north_span
    zone_north_max = north_min + args.end_fraction * north_span
    # The path is predominantly North/South.  The East bounds deliberately
    # encompass the whole observed corridor; North is the zone gate.
    zone_east_min, zone_east_max = min(east_values) - 0.25, max(east_values) + 0.25

    active_count = 0
    for row in rows:
        try:
            east = float(row["true_east"])
            north = float(row["true_north"])
            active = int(
                zone_east_min <= east <= zone_east_max
                and zone_north_min <= north <= zone_north_max
            )
            gps_east = float(row["gps_east"])
            gps_north = float(row["gps_north"])
            gps_up = float(row["gps_up"])
        except (TypeError, ValueError):
            active = 0
            gps_east = gps_north = gps_up = math.nan

        row["scenario_spoof_active"] = active
        row["spoofed_gps_east"] = gps_east + (args.east_offset if active else 0.0)
        row["spoofed_gps_north"] = gps_north + (args.north_offset if active else 0.0)
        row["spoofed_gps_up"] = gps_up
        active_count += active

    processed_dir = Path("data/processed")
    processed_dir.mkdir(parents=True, exist_ok=True)
    output_path = processed_dir / f"spoofed_{args.source.name}"
    appended_fields = [
        "gps_east", "gps_north", "gps_up",
        "true_east", "true_north", "true_up",
        "spoofed_gps_east", "spoofed_gps_north", "spoofed_gps_up",
        "scenario_spoof_active",
    ]
    fieldnames = source_fields + [field for field in appended_fields if field not in source_fields]
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    horizontal_summary = statistics_summary(normal_horizontal_errors)
    three_d_summary = statistics_summary(normal_3d_errors)
    axis_correlations = {
        "east": correlation(
            [float(row["gps_east"]) for row in valid_rows],
            [float(row["true_east"]) for row in valid_rows],
        ),
        "north": correlation(
            [float(row["gps_north"]) for row in valid_rows],
            [float(row["true_north"]) for row in valid_rows],
        ),
        "up": correlation(
            [float(row["gps_up"]) for row in valid_rows],
            [float(row["true_up"]) for row in valid_rows],
        ),
    }
    print(f"Source: {args.source}")
    print(f"Output: {output_path}")
    print(f"Samples: {len(rows)} (valid ENU samples: {len(valid_rows)})")
    print("Normal horizontal error [m]: " + ", ".join(f"{key}={value:.4f}" for key, value in horizontal_summary.items()))
    print("Normal 3D error [m]: " + ", ".join(f"{key}={value:.4f}" for key, value in three_d_summary.items()))
    print("ENU axis correlation (GNSS vs Gazebo): " + ", ".join(f"{key}={value:.4f}" for key, value in axis_correlations.items()))
    print(
        "Spoofing zone (Gazebo-relative ENU): "
        f"east=[{zone_east_min:.3f}, {zone_east_max:.3f}], "
        f"north=[{zone_north_min:.3f}, {zone_north_max:.3f}] m"
    )
    print(
        f"Step offset: east=+{args.east_offset:.3f} m, north=+{args.north_offset:.3f} m; "
        f"active samples={active_count}"
    )


if __name__ == "__main__":
    main()
