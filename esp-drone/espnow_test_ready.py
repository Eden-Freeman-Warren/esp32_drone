#!/usr/bin/env python3
"""
Autonomous ESP32-S3 ESP-Drone hover over ESP-NOW + OptiTrack.

Keep these files together:
    drone_test.py
    espnow_driver.py
    NatNetClient.py

First test:
    1. Remove propellers and confirm the full sequence runs.
    2. Reinstall propellers only for a cage or loose-tether test.
"""

import threading
import time

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator

from espnow_driver import register_espnow_driver
from NatNetClient import NatNetClient


URI = "espnow://COM9/115200"

CLIENT_IP = "192.168.0.50"
SERVER_IP = "192.168.0.4"
RIGID_BODY_ID = 555

HOVER_HEIGHT = 0.10
TAKEOFF_TIME = 2.5
HOVER_TIME = 5.0
LAND_TIME = 2.5

TRACKING_TIMEOUT = 0.25
HOME_CAPTURE_SECONDS = 1.0
MIN_EXT_POS_PACKETS = 50


lock = threading.Lock()

cf = None
latest_world_position = None
latest_sample_time = 0.0
home_position = None
extpos_count = 0


def receive_rigid_body(
    rigid_body_id,
    position,
    quaternion,
    *extra,
):
    del quaternion, extra

    global latest_world_position
    global latest_sample_time
    global extpos_count

    if int(rigid_body_id) != RIGID_BODY_ID:
        return

    world_position = (
        float(position[0]),
        float(position[1]),
        float(position[2]),
    )
    now = time.monotonic()

    with lock:
        latest_world_position = world_position
        latest_sample_time = now
        active_cf = cf
        active_home = home_position

    if active_cf is not None and active_home is not None:
        x = world_position[0] - active_home[0]
        y = world_position[1] - active_home[1]
        z = world_position[2] - active_home[2]

        # Position only for the first flight. The onboard IMU estimates
        # attitude and yaw without risking an OptiTrack quaternion mismatch.
        active_cf.extpos.send_extpos(x, y, z)
        extpos_count += 1


def read_tracking():
    with lock:
        position = latest_world_position
        age = time.monotonic() - latest_sample_time

    return position, age


def wait_for_tracking():
    print(f"Waiting for OptiTrack rigid body {RIGID_BODY_ID}...")

    deadline = time.monotonic() + 10.0

    while time.monotonic() < deadline:
        position, age = read_tracking()

        if position is not None and age <= TRACKING_TIMEOUT:
            print("OptiTrack tracking active.")
            return

        time.sleep(0.05)

    raise RuntimeError(
        f"No fresh OptiTrack data for rigid body {RIGID_BODY_ID}."
    )


def capture_home():
    global home_position

    print("Keep the drone still while HOME is captured.")

    sums = [0.0, 0.0, 0.0]
    count = 0
    deadline = time.monotonic() + HOME_CAPTURE_SECONDS

    while time.monotonic() < deadline:
        position, age = read_tracking()

        if position is None or age > TRACKING_TIMEOUT:
            raise RuntimeError("OptiTrack became stale during HOME capture.")

        for index in range(3):
            sums[index] += position[index]

        count += 1
        time.sleep(0.02)

    if count == 0:
        raise RuntimeError("No samples captured for HOME.")

    home_position = (
        sums[0] / count,
        sums[1] / count,
        sums[2] / count,
    )

    print(
        "HOME captured: "
        f"({home_position[0]:+.3f}, "
        f"{home_position[1]:+.3f}, "
        f"{home_position[2]:+.3f}) m"
    )
    print("The estimator will receive local position (0, 0, 0).")


def set_parameter(cf_object, name, value):
    print(f"Setting {name} = {value}...")
    cf_object.param.set_value(name, value)
    time.sleep(0.25)


def wait_for_external_position():
    print("Feeding local OptiTrack position to the estimator...")

    deadline = time.monotonic() + 10.0

    while extpos_count < MIN_EXT_POS_PACKETS:
        _, age = read_tracking()

        if age > TRACKING_TIMEOUT:
            raise RuntimeError("OptiTrack stopped before takeoff.")

        if time.monotonic() > deadline:
            raise RuntimeError("External-position stream did not start.")

        time.sleep(0.01)

    print(f"External-position packets sent: {extpos_count}")


def main():
    global cf

    register_espnow_driver()

    optitrack = NatNetClient()
    optitrack.set_client_address(CLIENT_IP)
    optitrack.set_server_address(SERVER_IP)
    optitrack.set_use_multicast(True)
    optitrack.rigid_body_listener = receive_rigid_body

    if not optitrack.run():
        raise RuntimeError("Could not start OptiTrack.")

    try:
        wait_for_tracking()
        capture_home()

        print(f"Connecting through {URI}...")

        with SyncCrazyflie(
            URI,
            cf=Crazyflie(rw_cache="./cache"),
        ) as scf:
            cf = scf.cf
            print("CONNECTED THROUGH ESP-NOW.")

            set_parameter(cf, "stabilizer.estimator", "2")
            set_parameter(cf, "stabilizer.controller", "1")
            set_parameter(cf, "commander.enHighLevel", "1")

            wait_for_external_position()

            print("Resetting Kalman estimator...")
            reset_estimator(scf)
            time.sleep(1.0)

            print("Arming...")
            cf.supervisor.send_arming_request(True)
            time.sleep(1.0)

            # Re-enable high-level setpoints after any low-level activity.
            cf.commander.send_notify_setpoint_stop()

            print(
                f"Taking off to {HOVER_HEIGHT:.2f} m "
                f"over {TAKEOFF_TIME:.1f} s..."
            )
            cf.high_level_commander.takeoff(
                HOVER_HEIGHT,
                TAKEOFF_TIME,
                yaw=None,
            )
            time.sleep(TAKEOFF_TIME)

            print(f"Hovering for {HOVER_TIME:.1f} s...")
            hover_deadline = time.monotonic() + HOVER_TIME

            while time.monotonic() < hover_deadline:
                position, age = read_tracking()

                if position is None or age > TRACKING_TIMEOUT:
                    raise RuntimeError("OptiTrack stopped during flight.")

                x = position[0] - home_position[0]
                y = position[1] - home_position[1]
                z = position[2] - home_position[2]

                print(
                    f"LOCAL x={x:+.3f} "
                    f"y={y:+.3f} "
                    f"z={z:+.3f} m"
                )
                time.sleep(0.20)

            print(f"Landing over {LAND_TIME:.1f} s...")
            cf.high_level_commander.land(
                0.0,
                LAND_TIME,
                yaw=None,
            )
            time.sleep(LAND_TIME)

            cf.high_level_commander.stop()
            cf.commander.send_stop_setpoint()
            cf.supervisor.send_arming_request(False)

            print("Flight sequence complete.")

    except KeyboardInterrupt:
        print("\nCtrl+C received.")

        if cf is not None:
            try:
                cf.high_level_commander.stop()
                cf.commander.send_stop_setpoint()
                cf.supervisor.send_arming_request(False)
            except Exception:
                pass

    finally:
        cf = None
        optitrack.shutdown()
        print("Program stopped.")


if __name__ == "__main__":
    main()