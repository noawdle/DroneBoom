#!/usr/bin/env python3
"""Record PX4 MAVLink telemetry and Gazebo X500 ground truth in one CSV file.

Run this after starting ``./run-px4-x500.sh`` and sourcing ``px4-gz-env.sh``.
It intentionally does not control the vehicle: it only sends a GCS heartbeat and
requests the standard HIGHRES_IMU MAVLink stream needed for logging.
"""

import csv
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from threading import Lock

from pymavlink import mavutil


SCRIPT_DIR = Path(__file__).resolve().parent
# PX4 SITL's existing API/offboard link.  It is separate from the GCS link
# (14550) used by test-px4-flight.py and QGroundControl, so both can receive
# complete telemetry at the same time.
MAVLINK_CONNECTION = "udpin:127.0.0.1:14540"
GAZEBO_POSE_TOPIC = "/world/default/pose/info"
MODEL_NAME = "x500_0"
SAMPLE_RATE_HZ = 10.0

FIELDNAMES = [
    "timestamp",
    "gps_lat",
    "gps_lon",
    "gps_alt",
    "local_x",
    "local_y",
    "local_z",
    "true_x",
    "true_y",
    "true_z",
    "roll",
    "pitch",
    "yaw",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
]


def import_gazebo_transport():
    """Import the already-installed local Gazebo Harmonic Python bindings.

    The PX4 virtual environment contains pymavlink but deliberately omits the
    system protobuf package.  Adding this existing system path avoids installing
    any new dependency or changing the established PX4/Gazebo environment.
    """

    gazebo_python = (
        SCRIPT_DIR
        / "PX4-Autopilot/.gz-harmonic/root/usr/lib/python3/dist-packages"
    )
    system_python = Path("/usr/lib/python3/dist-packages")

    for path in (system_python, gazebo_python):
        path_string = str(path)
        if path_string not in sys.path:
            sys.path.insert(0, path_string)

    try:
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
    except ImportError as error:
        raise RuntimeError(
            "Gazebo Python bindings could not be loaded. Run: "
            "source ./px4-gz-env.sh"
        ) from error

    return Node, Pose_V


class GroundTruthSubscriber:
    """Keeps the latest Gazebo world pose for the X500 model."""

    def __init__(self):
        node_type, pose_v_type = import_gazebo_transport()
        self._lock = Lock()
        self._latest = (None, None, None)
        self._received = False
        self._node = node_type()

        if not self._node.subscribe(pose_v_type, GAZEBO_POSE_TOPIC, self._callback):
            raise RuntimeError(
                f"Could not subscribe to Gazebo topic {GAZEBO_POSE_TOPIC}. "
                "Start ./run-px4-x500.sh first."
            )

    def _callback(self, message):
        for pose in message.pose:
            # Pose_V includes the model and all of its links.  Exact matching
            # selects the model's world pose, not a link-relative pose.
            if pose.name == MODEL_NAME:
                with self._lock:
                    self._latest = (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    )
                    self._received = True
                return

    def snapshot(self):
        with self._lock:
            return self._latest, self._received


def send_gcs_heartbeat(connection):
    """Identify the logger as a harmless MAVLink ground station."""

    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def request_highres_imu(connection):
    """Ask PX4 for HIGHRES_IMU at the same 10 Hz rate as the CSV."""

    interval_us = int(1_000_000 / SAMPLE_RATE_HZ)
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def update_mavlink_state(message, state):
    """Store the newest value of every requested PX4 telemetry stream."""

    message_type = message.get_type()

    if message_type == "GPS_RAW_INT":
        # GPS_RAW_INT uses degrees * 1E7 and altitude in millimetres (AMSL).
        state["gps_lat"] = message.lat / 1e7
        state["gps_lon"] = message.lon / 1e7
        state["gps_alt"] = message.alt / 1000.0
    elif message_type == "LOCAL_POSITION_NED":
        state["local_x"] = message.x
        state["local_y"] = message.y
        state["local_z"] = message.z
    elif message_type == "ATTITUDE":
        state["roll"] = message.roll
        state["pitch"] = message.pitch
        state["yaw"] = message.yaw
    elif message_type == "HIGHRES_IMU":
        state["accel_x"] = message.xacc
        state["accel_y"] = message.yacc
        state["accel_z"] = message.zacc
        state["gyro_x"] = message.xgyro
        state["gyro_y"] = message.ygyro
        state["gyro_z"] = message.zgyro


def csv_value(value):
    """Leave unavailable telemetry blank rather than fabricating a value."""

    return "" if value is None else f"{value:.9f}"


def main():
    output_dir = SCRIPT_DIR / "data"
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / datetime.now().strftime("flight_%Y%m%d_%H%M%S.csv")

    print(f"Connecting to PX4 MAVLink at {MAVLINK_CONNECTION} ...")
    connection = mavutil.mavlink_connection(MAVLINK_CONNECTION, source_system=253)
    heartbeat = connection.wait_heartbeat(timeout=15)
    if heartbeat is None:
        raise RuntimeError("No PX4 MAVLink heartbeat. Start ./run-px4-x500.sh first.")

    print(
        f"Connected to PX4 system {connection.target_system}, "
        f"component {connection.target_component}."
    )
    ground_truth = GroundTruthSubscriber()
    send_gcs_heartbeat(connection)
    request_highres_imu(connection)

    # Avoid an avoidable empty true_* row while Gazebo delivers its first pose.
    pose_deadline = time.monotonic() + 5.0
    while time.monotonic() < pose_deadline:
        _, received_ground_truth = ground_truth.snapshot()
        if received_ground_truth:
            break
        time.sleep(0.02)

    if not received_ground_truth:
        print(
            "Warning: Gazebo x500_0 pose has not arrived yet; true_x/true_y/true_z "
            "will remain blank until it does."
        )

    state = {name: None for name in FIELDNAMES if name != "timestamp"}
    period = 1.0 / SAMPLE_RATE_HZ
    start_time = time.monotonic()
    next_sample = start_time
    last_heartbeat = 0.0
    rows_written = 0
    running = True

    def stop_logging(_signal, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_logging)
    signal.signal(signal.SIGTERM, stop_logging)

    print(f"Logging at {SAMPLE_RATE_HZ:g} Hz: {output_path}")
    print("Press Ctrl+C to finish safely.")

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
        writer.writeheader()

        while running:
            now = time.monotonic()
            if now - last_heartbeat >= 1.0:
                send_gcs_heartbeat(connection)
                last_heartbeat = now

            # Drain pending MAVLink messages before taking this 10 Hz snapshot.
            while True:
                message = connection.recv_match(blocking=False)
                if message is None:
                    break
                update_mavlink_state(message, state)

            now = time.monotonic()
            if now >= next_sample:
                (true_x, true_y, true_z), received_ground_truth = ground_truth.snapshot()
                if received_ground_truth:
                    state["true_x"] = true_x
                    state["true_y"] = true_y
                    state["true_z"] = true_z

                row = {"timestamp": f"{now - start_time:.6f}"}
                row.update({name: csv_value(state[name]) for name in state})
                writer.writerow(row)
                csv_file.flush()
                rows_written += 1

                # Keep the target cadence stable even if one iteration is late.
                next_sample += period
                if next_sample <= now:
                    next_sample = now + period
            else:
                time.sleep(min(0.01, next_sample - now))

    (_, _, _), received_ground_truth = ground_truth.snapshot()
    print(f"Saved {rows_written} rows to {output_path}")
    if not received_ground_truth:
        print(
            "Warning: no Gazebo x500_0 pose was received; true_x/true_y/true_z "
            "remain blank. Verify that the X500 simulation was running."
        )


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
