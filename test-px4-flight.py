#!/usr/bin/env python3
"""Minimal PX4 SITL MAVLink smoke test: arm, fly 3 m, and land."""

import math
import sys
import time
from threading import Event, Lock, Thread

from pymavlink import mavutil


CONNECTION = "udpin:127.0.0.1:14550"
PX4_SYSTEM_ID = 1
PX4_AUTOPILOT_COMPONENT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
ALTITUDE_M = 2.5
MOVE_NORTH_M = 3.0
POSITION_TYPE_MASK = 3576  # Position enabled; velocity, acceleration, yaw ignored.
HEARTBEAT_PERIOD_S = 1.0


def gcs_heartbeat(vehicle):
    vehicle.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def is_px4_autopilot_message(message):
    """Ignore forwarded messages from the logger or other MAVLink clients."""

    return (
        message.get_srcSystem() == PX4_SYSTEM_ID
        and message.get_srcComponent() == PX4_AUTOPILOT_COMPONENT
    )


def wait_for_px4_heartbeat(vehicle, timeout=30.0):
    """Wait specifically for the PX4 autopilot, not a forwarded GCS heartbeat."""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if message is not None and is_px4_autopilot_message(message):
            vehicle.target_system = PX4_SYSTEM_ID
            vehicle.target_component = PX4_AUTOPILOT_COMPONENT
            return message
    return None


class GCSHeartbeatSender:
    """Keep the PX4 GCS data link alive for the whole flight script lifetime."""

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
            # The main thread also sends commands and setpoints on this MAVLink
            # object. UDP writes are protected to keep MAVLink sequencing orderly.
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


def stream_position(vehicle, tx_lock, north, east, down, duration):
    deadline = time.monotonic() + duration
    latest_position = None
    armed = False

    while time.monotonic() < deadline:
        position_setpoint(vehicle, tx_lock, north, east, down)

        while True:
            message = vehicle.recv_match(blocking=False)
            if message is None:
                break
            if not is_px4_autopilot_message(message):
                continue
            if message.get_type() == "LOCAL_POSITION_NED":
                latest_position = (message.x, message.y, message.z)
            elif message.get_type() == "HEARTBEAT":
                armed = bool(
                    message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

        time.sleep(0.1)

    return latest_position, armed


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


def wait_for_landing(vehicle, timeout=30.0):
    deadline = time.monotonic() + timeout
    latest_position = None
    landed = False
    armed = True

    while time.monotonic() < deadline:
        message = vehicle.recv_match(blocking=True, timeout=0.2)
        if message is None:
            continue
        if not is_px4_autopilot_message(message):
            continue
        if message.get_type() == "LOCAL_POSITION_NED":
            latest_position = (message.x, message.y, message.z)
        elif message.get_type() == "EXTENDED_SYS_STATE":
            landed = (
                message.landed_state
                == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            )
        elif message.get_type() == "HEARTBEAT":
            armed = bool(
                message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

        if landed and not armed:
            return latest_position, armed

    raise TimeoutError("vehicle did not land and disarm within the timeout")


def main():
    vehicle = mavutil.mavlink_connection(
        CONNECTION,
        source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )
    heartbeat = wait_for_px4_heartbeat(vehicle, timeout=30)
    if heartbeat is None:
        raise TimeoutError("no PX4 heartbeat received on UDP 14550")

    print(
        f"Connected: system={vehicle.target_system} "
        f"component={vehicle.target_component}"
    )

    armed = False
    tx_lock = Lock()
    heartbeat_sender = GCSHeartbeatSender(vehicle, tx_lock)
    try:
        # Start before the first setpoint and retain it through every command,
        # verification loop, and landing wait.
        heartbeat_sender.start()

        position, _ = stream_position(
            vehicle, tx_lock, 0.0, 0.0, -ALTITUDE_M, 3.0
        )
        print(f"Initial local position: {position}")

        # pymavlink 2.4.50 requires placeholders even when a named PX4 mode is used.
        with tx_lock:
            vehicle.set_mode_px4("OFFBOARD", 0, 0)
        send_command(
            vehicle,
            tx_lock,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1.0,
        )

        position, armed = stream_position(
            vehicle, tx_lock, 0.0, 0.0, -ALTITUDE_M, 10.0
        )
        print(f"Takeoff position: {position}, armed={armed}")
        if not armed or position is None or position[2] > -1.5:
            raise RuntimeError("takeoff verification failed")

        position, armed = stream_position(
            vehicle, tx_lock, MOVE_NORTH_M, 0.0, -ALTITUDE_M, 8.0
        )
        print(f"Moved position: {position}, armed={armed}")
        if position is None or position[0] < 2.0:
            raise RuntimeError("horizontal movement verification failed")

        send_command(
            vehicle,
            tx_lock,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0.0,
            0.0,
            0.0,
            math.nan,
            math.nan,
            math.nan,
            0.0,
        )
        position, armed = wait_for_landing(vehicle)
        print(f"Landed position: {position}, armed={armed}")
        print("FLIGHT_TEST_PASS")
        return 0
    finally:
        if armed:
            send_command(
                vehicle,
                tx_lock,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0.0,
                0.0,
                0.0,
                math.nan,
                math.nan,
                math.nan,
                0.0,
            )
        heartbeat_sender.stop()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"FLIGHT_TEST_FAIL: {error}", file=sys.stderr)
        sys.exit(1)
