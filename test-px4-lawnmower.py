#!/usr/bin/env python3
"""Scan a small ENU-area with PX4 SITL using a safe lawnmower path.

The PX4 LOCAL_NED frame is used directly: x is North, y is East, and z is
Down.  The script keeps an independent GCS heartbeat running for its entire
lifetime, matching the proven connection pattern in test-px4-flight.py.
"""

import math
import sys
import time
from threading import Event, Lock, Thread

from pymavlink import mavutil


CONNECTION = "udpin:127.0.0.1:14550"
PX4_SYSTEM_ID = 1
PX4_AUTOPILOT_COMPONENT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
ALTITUDE_M = 2.5
NORTH_SPAN_M = 8.0
EAST_SPAN_M = 8.0
LANE_SPACING_M = 1.0
CRUISE_SPEED_M_S = 1.0
POSITION_TYPE_MASK = 3576  # Position enabled; velocity, acceleration, yaw ignored.
HEARTBEAT_PERIOD_S = 1.0
SETPOINT_PERIOD_S = 0.1
ARRIVAL_TOLERANCE_M = 0.35


def gcs_heartbeat(vehicle):
    vehicle.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def is_px4_autopilot_message(message):
    """Reject forwarded heartbeats from the data logger or other GCS clients."""

    return (
        message.get_srcSystem() == PX4_SYSTEM_ID
        and message.get_srcComponent() == PX4_AUTOPILOT_COMPONENT
    )


def wait_for_px4_heartbeat(vehicle, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if message is not None and is_px4_autopilot_message(message):
            vehicle.target_system = PX4_SYSTEM_ID
            vehicle.target_component = PX4_AUTOPILOT_COMPONENT
            return message
    return None


class GCSHeartbeatSender:
    """Send one GCS heartbeat per second independently of waypoint progress."""

    def __init__(self, vehicle, tx_lock):
        self._vehicle = vehicle
        self._tx_lock = tx_lock
        self._stop_event = Event()
        self._thread = Thread(target=self._run, name="gcs-heartbeat", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)

    def _run(self):
        while not self._stop_event.is_set():
            with self._tx_lock:
                gcs_heartbeat(self._vehicle)
            self._stop_event.wait(HEARTBEAT_PERIOD_S)


def position_setpoint(vehicle, tx_lock, north, east, down):
    with tx_lock:
        vehicle.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            POSITION_TYPE_MASK,
            north,
            east,
            down,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )


def send_command(vehicle, tx_lock, command, *parameters):
    values = list(parameters) + [0.0] * (7 - len(parameters))
    with tx_lock:
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            command,
            0,
            *values[:7],
        )


def consume_messages(vehicle, latest_position=None, armed=False, samples=None):
    """Read only real PX4 state; 14550 can forward other GCS messages."""

    while True:
        message = vehicle.recv_match(blocking=False)
        if message is None:
            break
        if not is_px4_autopilot_message(message):
            continue
        if message.get_type() == "LOCAL_POSITION_NED":
            latest_position = (message.x, message.y, message.z)
            if samples is not None:
                samples.append(latest_position)
        elif message.get_type() == "HEARTBEAT":
            armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return latest_position, armed


def wait_for_position(vehicle, timeout=5.0):
    deadline = time.monotonic() + timeout
    latest_position = None
    while time.monotonic() < deadline:
        message = vehicle.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.4)
        if message is not None and is_px4_autopilot_message(message):
            latest_position = (message.x, message.y, message.z)
            return latest_position
    raise TimeoutError("PX4 did not provide LOCAL_POSITION_NED")


def stream_position(vehicle, tx_lock, north, east, down, duration, samples=None):
    deadline = time.monotonic() + duration
    latest_position = None
    armed = False
    while time.monotonic() < deadline:
        position_setpoint(vehicle, tx_lock, north, east, down)
        latest_position, armed = consume_messages(
            vehicle, latest_position, armed, samples
        )
        time.sleep(SETPOINT_PERIOD_S)
    return latest_position, armed


def fly_to_waypoint(vehicle, tx_lock, target, latest_position, samples, index, total):
    """Move at a setpoint rate near CRUISE_SPEED_M_S and verify arrival."""

    if latest_position is None:
        latest_position = wait_for_position(vehicle)
    start = latest_position
    distance = math.hypot(target[0] - start[0], target[1] - start[1])
    travel_duration = max(distance / CRUISE_SPEED_M_S, SETPOINT_PERIOD_S)
    deadline = time.monotonic() + travel_duration + 12.0
    begin = time.monotonic()
    armed = True
    print(
        f"Waypoint {index}/{total}: N={target[0]:.2f}, E={target[1]:.2f} m "
        f"(planned {distance:.2f} m)"
    )

    while time.monotonic() < deadline:
        progress = min(1.0, (time.monotonic() - begin) / travel_duration)
        # A moving absolute-position target limits the commanded path speed while
        # retaining PX4's normal position controller and obstacle-free SITL logic.
        command_north = start[0] + (target[0] - start[0]) * progress
        command_east = start[1] + (target[1] - start[1]) * progress
        position_setpoint(vehicle, tx_lock, command_north, command_east, target[2])
        latest_position, armed = consume_messages(
            vehicle, latest_position, armed, samples
        )
        if not armed:
            raise RuntimeError("vehicle disarmed before waypoint completion")
        if latest_position is not None and progress >= 1.0:
            horizontal_error = math.hypot(
                target[0] - latest_position[0], target[1] - latest_position[1]
            )
            vertical_error = abs(target[2] - latest_position[2])
            if horizontal_error <= ARRIVAL_TOLERANCE_M and vertical_error <= 0.45:
                print(f"  reached: error={horizontal_error:.2f} m")
                return latest_position, armed
        time.sleep(SETPOINT_PERIOD_S)
    raise TimeoutError(f"waypoint {index} was not reached before timeout")


def wait_for_landing(vehicle, timeout=40.0):
    deadline = time.monotonic() + timeout
    latest_position = None
    landed = False
    armed = True
    while time.monotonic() < deadline:
        message = vehicle.recv_match(blocking=True, timeout=0.2)
        if message is None or not is_px4_autopilot_message(message):
            continue
        if message.get_type() == "LOCAL_POSITION_NED":
            latest_position = (message.x, message.y, message.z)
        elif message.get_type() == "EXTENDED_SYS_STATE":
            landed = message.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
        elif message.get_type() == "HEARTBEAT":
            armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if landed and not armed:
            return latest_position, armed
    raise TimeoutError("vehicle did not land and disarm within the timeout")


def lawnmower_waypoints(home_north, home_east):
    """Build North/East waypoint rows separated by LANE_SPACING_M."""

    lane_count = int(round(EAST_SPAN_M / LANE_SPACING_M))
    if not math.isclose(lane_count * LANE_SPACING_M, EAST_SPAN_M, abs_tol=1e-6):
        raise ValueError("EAST_SPAN_M must be divisible by LANE_SPACING_M")
    waypoints = []
    for lane in range(lane_count + 1):
        east = home_east + lane * LANE_SPACING_M
        north = home_north + (NORTH_SPAN_M if lane % 2 == 0 else 0.0)
        waypoints.append((north, east, -ALTITUDE_M))
    return waypoints


def main():
    vehicle = mavutil.mavlink_connection(
        CONNECTION,
        source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )
    if wait_for_px4_heartbeat(vehicle, timeout=30.0) is None:
        raise TimeoutError("no PX4 autopilot heartbeat received on UDP 14550")
    print(f"Connected: system={vehicle.target_system} component={vehicle.target_component}")

    tx_lock = Lock()
    heartbeat_sender = GCSHeartbeatSender(vehicle, tx_lock)
    armed = False
    local_samples = []
    try:
        heartbeat_sender.start()
        home_position = wait_for_position(vehicle)
        home_north, home_east, _ = home_position
        print(f"Home local NED: N={home_north:.2f}, E={home_east:.2f}")

        # PX4 requires a pre-offboard setpoint stream before the mode request.
        stream_position(vehicle, tx_lock, home_north, home_east, -ALTITUDE_M, 3.0, local_samples)
        with tx_lock:
            vehicle.set_mode_px4("OFFBOARD", 0, 0)
        send_command(vehicle, tx_lock, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)

        latest_position, armed = stream_position(
            vehicle, tx_lock, home_north, home_east, -ALTITUDE_M, 10.0, local_samples
        )
        print(f"Takeoff position: {latest_position}, armed={armed}")
        if not armed or latest_position is None or latest_position[2] > -1.5:
            raise RuntimeError("takeoff verification failed")

        waypoints = lawnmower_waypoints(home_north, home_east)
        print(
            f"Scanning {NORTH_SPAN_M:.1f} m North × {EAST_SPAN_M:.1f} m East, "
            f"{LANE_SPACING_M:.1f} m lanes at {CRUISE_SPEED_M_S:.1f} m/s."
        )
        for index, waypoint in enumerate(waypoints, start=1):
            latest_position, armed = fly_to_waypoint(
                vehicle, tx_lock, waypoint, latest_position, local_samples, index, len(waypoints)
            )

        if local_samples:
            north_values = [sample[0] for sample in local_samples]
            east_values = [sample[1] for sample in local_samples]
            print(
                f"Observed PX4 local coverage: North={max(north_values) - min(north_values):.2f} m, "
                f"East={max(east_values) - min(east_values):.2f} m"
            )

        send_command(
            vehicle, tx_lock, mavutil.mavlink.MAV_CMD_NAV_LAND,
            0.0, 0.0, 0.0, math.nan, math.nan, math.nan, 0.0,
        )
        position, armed = wait_for_landing(vehicle)
        print(f"Landed position: {position}, armed={armed}")
        print("LAWNMOWER_FLIGHT_PASS")
        return 0
    finally:
        if armed:
            send_command(
                vehicle, tx_lock, mavutil.mavlink.MAV_CMD_NAV_LAND,
                0.0, 0.0, 0.0, math.nan, math.nan, math.nan, 0.0,
            )
        heartbeat_sender.stop()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"LAWNMOWER_FLIGHT_FAIL: {error}", file=sys.stderr)
        sys.exit(1)
