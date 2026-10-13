#!/usr/bin/env python3
"""Create a reproducible, spatially connected GNSS spoofing-zone scenario.

Only a new processed CSV is written.  The source flight CSV and Gazebo ground
truth remain unchanged.  Scenario labels and zone geometry are evaluation-only
metadata; the existing detector does not consume them for decisions.
"""

import argparse
import csv
import json
import math
import random
from pathlib import Path


WGS84_A = 6_378_137.0
WGS84_F = 1.0 / 298.257_223_563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
DEFAULT_SEED = 42


def require_float(row, field):
    value = row.get(field, "")
    if value in (None, ""):
        raise ValueError(f"missing required value '{field}'")
    return float(value)


def geodetic_to_ecef(latitude_deg, longitude_deg, height_m):
    latitude, longitude = math.radians(latitude_deg), math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude * sin_latitude)
    return (
        (radius + height_m) * math.cos(latitude) * math.cos(longitude),
        (radius + height_m) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - WGS84_E2) + height_m) * sin_latitude,
    )


def ecef_delta_to_enu(delta_ecef, latitude_deg, longitude_deg):
    dx, dy, dz = delta_ecef
    latitude, longitude = math.radians(latitude_deg), math.radians(longitude_deg)
    sin_latitude, cos_latitude = math.sin(latitude), math.cos(latitude)
    sin_longitude, cos_longitude = math.sin(longitude), math.cos(longitude)
    return (
        -sin_longitude * dx + cos_longitude * dy,
        -sin_latitude * cos_longitude * dx - sin_latitude * sin_longitude * dy + cos_latitude * dz,
        cos_latitude * cos_longitude * dx + cos_latitude * sin_longitude * dy + sin_latitude * dz,
    )


def first_valid_row(rows):
    for row in rows:
        try:
            for field in ("gps_lat", "gps_lon", "gps_alt", "true_x", "true_y", "true_z"):
                require_float(row, field)
            return row
        except ValueError:
            continue
    raise ValueError("no row contains valid GNSS and Gazebo ground truth")


def add_enu_columns(rows):
    origin = first_valid_row(rows)
    latitude = require_float(origin, "gps_lat")
    longitude = require_float(origin, "gps_lon")
    height = require_float(origin, "gps_alt")
    origin_ecef = geodetic_to_ecef(latitude, longitude, height)
    origin_true = tuple(require_float(origin, field) for field in ("true_x", "true_y", "true_z"))
    for source_row in rows:
        row = source_row
        try:
            current_ecef = geodetic_to_ecef(
                require_float(row, "gps_lat"), require_float(row, "gps_lon"), require_float(row, "gps_alt")
            )
            enu = ecef_delta_to_enu(
                tuple(value - reference for value, reference in zip(current_ecef, origin_ecef)), latitude, longitude
            )
            true_enu = tuple(
                require_float(row, field) - origin_true[index]
                for index, field in enumerate(("true_x", "true_y", "true_z"))
            )
        except ValueError:
            enu = true_enu = (math.nan, math.nan, math.nan)
        row.update(
            gps_east=enu[0], gps_north=enu[1], gps_up=enu[2],
            true_east=true_enu[0], true_north=true_enu[1], true_up=true_enu[2],
        )


def finite_path_samples(rows):
    samples = []
    for row in rows:
        try:
            east, north = float(row["true_east"]), float(row["true_north"])
            if math.isfinite(east) and math.isfinite(north):
                samples.append((east, north))
        except (KeyError, TypeError, ValueError):
            pass
    if not samples:
        raise ValueError("no valid relative Ground Truth samples")
    return samples


def inside(zone, east, north):
    return ((east - zone["center_east"]) / zone["radius_east"]) ** 2 + (
        (north - zone["center_north"]) / zone["radius_north"]
    ) ** 2 <= 1.0


def make_zones(samples, rng, count):
    """Select centres from observed path samples, so every zone is traversed."""

    zones = []
    candidates = list(samples)
    east_min, east_max = min(east for east, _ in candidates), max(east for east, _ in candidates)
    north_min, north_max = min(north for _, north in candidates), max(north for _, north in candidates)
    rng.shuffle(candidates)
    for east, north in candidates:
        radius_east = rng.uniform(0.55, 0.85)
        radius_north = rng.uniform(0.65, 1.00)
        # The whole ellipse, not merely its centre, stays inside the measured
        # survey envelope.  This avoids presenting an unvisited outside area
        # as part of a spoofing zone.
        if not (
            east_min + radius_east <= east <= east_max - radius_east
            and north_min + radius_north <= north <= north_max - radius_north
        ):
            continue
        # Keep zones visually distinct; their extent is still large enough to
        # intersect at least one 1 m-spaced survey line.
        if any(
            math.hypot(east - existing["center_east"], north - existing["center_north"])
            < max(radius_east, radius_north) + max(existing["radius_east"], existing["radius_north"]) + 0.8
            for existing in zones
        ):
            continue
        magnitude = rng.uniform(2.5, 4.5)
        angle = rng.uniform(0.0, 2.0 * math.pi)
        zone = {
            "id": len(zones) + 1,
            "center_east": east,
            "center_north": north,
            "radius_east": radius_east,
            "radius_north": radius_north,
            "offset_east": magnitude * math.cos(angle),
            "offset_north": magnitude * math.sin(angle),
            "offset_magnitude": magnitude,
        }
        if sum(inside(zone, sample_east, sample_north) for sample_east, sample_north in samples) >= 4:
            zones.append(zone)
        if len(zones) == count:
            return zones
    raise RuntimeError("could not place enough distinct zones on the observed flight path")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="raw lawnmower flight CSV")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="reproducible scenario seed (default: 42)")
    parser.add_argument("--zones", type=int, default=4, help="number of ellipse zones, 3 through 5 (default: 4)")
    args = parser.parse_args()
    if not 3 <= args.zones <= 5:
        parser.error("--zones must be from 3 through 5")
    if not args.source.is_file():
        parser.error(f"source CSV not found: {args.source}")

    with args.source.open(newline="", encoding="utf-8") as source_file:
        reader = csv.DictReader(source_file)
        source_fields = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        parser.error("source CSV has no rows")

    add_enu_columns(rows)
    zones = make_zones(finite_path_samples(rows), random.Random(args.seed), args.zones)
    active_counts = {zone["id"]: 0 for zone in zones}
    for row in rows:
        try:
            east, north, up = float(row["true_east"]), float(row["true_north"]), float(row["true_up"])
            gps_east, gps_north, gps_up = float(row["gps_east"]), float(row["gps_north"]), float(row["gps_up"])
            selected = next((zone for zone in zones if inside(zone, east, north)), None)
        except (KeyError, TypeError, ValueError):
            selected = None
            gps_east = gps_north = gps_up = math.nan
        row["scenario_spoof_active"] = int(selected is not None)
        row["scenario_zone_id"] = selected["id"] if selected else 0
        row["spoofed_gps_east"] = gps_east + (selected["offset_east"] if selected else 0.0)
        row["spoofed_gps_north"] = gps_north + (selected["offset_north"] if selected else 0.0)
        row["spoofed_gps_up"] = gps_up
        if selected:
            active_counts[selected["id"]] += 1

    processed_dir = Path("data/processed")
    processed_dir.mkdir(parents=True, exist_ok=True)
    output_path = processed_dir / f"spoofed_random_{args.source.name}"
    scenario_path = processed_dir / f"scenario_random_{args.source.stem}.json"
    appended_fields = [
        "gps_east", "gps_north", "gps_up", "true_east", "true_north", "true_up",
        "spoofed_gps_east", "spoofed_gps_north", "spoofed_gps_up",
        "scenario_spoof_active", "scenario_zone_id",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(
            output_file,
            fieldnames=source_fields + [field for field in appended_fields if field not in source_fields],
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)
    with scenario_path.open("w", encoding="utf-8") as scenario_file:
        json.dump({"seed": args.seed, "source": str(args.source), "zones": zones}, scenario_file, indent=2)

    print(f"Source: {args.source}")
    print(f"Output: {output_path}")
    print(f"Scenario metadata: {scenario_path}")
    print(f"Seed: {args.seed}; zones: {len(zones)}")
    for zone in zones:
        print(
            f"Zone {zone['id']}: centre=({zone['center_east']:.2f}, {zone['center_north']:.2f}) m, "
            f"radii=({zone['radius_east']:.2f}, {zone['radius_north']:.2f}) m, "
            f"offset=({zone['offset_east']:+.2f}, {zone['offset_north']:+.2f}) m, "
            f"samples={active_counts[zone['id']]}"
        )


if __name__ == "__main__":
    main()
