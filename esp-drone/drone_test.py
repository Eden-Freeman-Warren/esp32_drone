import time

import cflib.crtp
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

cf = None
latest_sample = None
extpos_count = 0


def receive_rigid_body(rigid_body_id, position, quaternion, *extra):
    global latest_sample, extpos_count

    if int(rigid_body_id) != RIGID_BODY_ID:
        return

    x, y, z = map(float, position)
    qx, qy, qz, qw = map(float, quaternion)

    latest_sample = (x, y, z, time.monotonic())

    if cf is not None:
        cf.extpos.send_extpose(
            x, y, z,
            qx, qy, qz, qw,
        )
        extpos_count += 1


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
        with SyncCrazyflie(
            URI,
            cf=Crazyflie(rw_cache="./cache"),
        ) as scf:
            cf = scf.cf

            cf.param.set_value("stabilizer.estimator", "2")
            cf.param.set_value("stabilizer.controller", "1")
            cf.param.set_value("commander.enHighLevel", "1")

            print("Waiting for OptiTrack...")

            timeout = time.monotonic() + 10.0
            while extpos_count < 50:
                if time.monotonic() > timeout:
                    raise RuntimeError("No OptiTrack pose.")
                time.sleep(0.01)

            reset_estimator(scf)

            cf.supervisor.send_arming_request(True)
            time.sleep(1.0)

            cf.commander.send_notify_setpoint_stop()

            cf.high_level_commander.takeoff(
                HOVER_HEIGHT,
                TAKEOFF_TIME,
            )
            time.sleep(TAKEOFF_TIME)

            print("Hovering...")
            hover_end = time.monotonic() + HOVER_TIME

            while time.monotonic() < hover_end:
                if (
                    latest_sample is None
                    or time.monotonic() - latest_sample[3] > 0.25
                ):
                    raise RuntimeError("OptiTrack stopped.")

                time.sleep(0.02)

            cf.high_level_commander.land(0.0, LAND_TIME)
            time.sleep(LAND_TIME)

            cf.commander.send_stop_setpoint()
            cf.supervisor.send_arming_request(False)

    finally:
        cf = None
        optitrack.shutdown()


if __name__ == "__main__":
    main()