import math
import threading
import time

from NatNetClient import NatNetClient


ROBOT_ID = 800

CLIENT_ADDRESS = "192.168.0.13"
OPTITRACK_SERVER_ADDRESS = "192.168.0.4"
USE_MULTICAST = True

TRACKING_TIMEOUT_SECONDS = 0.25

PRINT_RATE_HZ = 10
PRINT_DT = 1.0 / PRINT_RATE_HZ

MOVEMENT_DEADBAND_METERS = 0.01


data_lock = threading.Lock()

latest_position = None
latest_update_time = 0.0
seen_rigid_body_ids = set()


def receive_rigid_body_frame(
    rigid_body_id,
    position,
    rotation_quaternion,
    *extra,
):
    global latest_position
    global latest_update_time

    rigid_body_id = int(rigid_body_id)

    with data_lock:
        seen_rigid_body_ids.add(rigid_body_id)

        if rigid_body_id != ROBOT_ID:
            return

        latest_position = (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )

        latest_update_time = time.monotonic()


def get_latest_position():
    with data_lock:
        position = latest_position
        update_time = latest_update_time
        ids = sorted(seen_rigid_body_ids)

    if position is None:
        return None, float("inf"), ids

    age = time.monotonic() - update_time

    return position, age, ids


def direction_label(dx, dy, dz):
    labels = []

    if abs(dx) >= MOVEMENT_DEADBAND_METERS:
        labels.append("X+" if dx > 0 else "X-")

    if abs(dy) >= MOVEMENT_DEADBAND_METERS:
        labels.append("Y+" if dy > 0 else "Y-")

    if abs(dz) >= MOVEMENT_DEADBAND_METERS:
        labels.append("Z+" if dz > 0 else "Z-")

    if not labels:
        return "STILL"

    return " ".join(labels)


def main():
    streaming_client = NatNetClient()

    streaming_client.set_client_address(CLIENT_ADDRESS)
    streaming_client.set_server_address(OPTITRACK_SERVER_ADDRESS)
    streaming_client.set_use_multicast(USE_MULTICAST)

    streaming_client.rigid_body_listener = receive_rigid_body_frame

    started = streaming_client.run()

    if not started:
        raise RuntimeError(
            "NatNet failed to start. Check Motive streaming, "
            "IP addresses, multicast, and Windows firewall."
        )

    print("OPTITRACK X/Y/Z TEST")
    print("This program does not open COM9 and cannot move the motors.")
    print(f"Waiting for rigid body ID {ROBOT_ID}...")

    try:
        # Wait until the selected rigid body is detected.
        while True:
            position, age, ids = get_latest_position()

            if (
                position is not None
                and age < TRACKING_TIMEOUT_SECONDS
            ):
                break

            if ids:
                print(
                    f"Waiting for ID {ROBOT_ID}. "
                    f"Active IDs seen: {ids}"
                )
            else:
                print("No rigid bodies received yet.")

            time.sleep(0.5)

        # Save the initial position as the home position.
        home_x, home_y, home_z = position

        previous_x = home_x
        previous_y = home_y
        previous_z = home_z
        previous_time = time.monotonic()

        print("\nHOME POSITION SAVED")
        print(f"X = {home_x:+.3f} m")
        print(f"Y = {home_y:+.3f} m")
        print(f"Z = {home_z:+.3f} m")

        print("\nMove the drone by hand along one axis at a time.")
        print("Raise it vertically and confirm that Z changes.")
        print("Press Ctrl+C to stop.\n")

        while True:
            loop_start = time.monotonic()

            position, age, ids = get_latest_position()

            if (
                position is None
                or age > TRACKING_TIMEOUT_SECONDS
            ):
                print(
                    "TRACKING LOST OR STALE: "
                    f"age={age:.3f} s"
                )

                time.sleep(PRINT_DT)
                continue

            x, y, z = position

            now = time.monotonic()
            dt = now - previous_time

            if dt <= 0.0:
                dt = PRINT_DT

            # Calculate velocity in meters per second.
            vx = (x - previous_x) / dt
            vy = (y - previous_y) / dt
            vz = (z - previous_z) / dt

            # Calculate position relative to the saved home position.
            relative_x = x - home_x
            relative_y = y - home_y
            relative_z = z - home_z

            distance_from_home = math.sqrt(
                relative_x * relative_x
                + relative_y * relative_y
                + relative_z * relative_z
            )

            motion = direction_label(
                x - previous_x,
                y - previous_y,
                z - previous_z,
            )

            print(
                f"ABS  "
                f"X:{x:+.3f} "
                f"Y:{y:+.3f} "
                f"Z:{z:+.3f}  |  "
                f"FROM HOME  "
                f"dX:{relative_x:+.3f} "
                f"dY:{relative_y:+.3f} "
                f"dZ:{relative_z:+.3f}  |  "
                f"VEL  "
                f"vX:{vx:+.2f} "
                f"vY:{vy:+.2f} "
                f"vZ:{vz:+.2f}  |  "
                f"DIST:{distance_from_home:.3f}  "
                f"MOVE:{motion}"
            )

            previous_x = x
            previous_y = y
            previous_z = z
            previous_time = now

            elapsed = time.monotonic() - loop_start
            sleep_time = PRINT_DT - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping OptiTrack X/Y/Z test.")

    finally:
        streaming_client.shutdown()
        print("Program stopped.")


if __name__ == "__main__":
    main()