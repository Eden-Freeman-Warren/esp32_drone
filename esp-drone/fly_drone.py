import struct
import time

import serial


PORT = "COM9"
BAUD = 115200
RATE_HZ = 50

SYNC = 0xAA
CRTP_SETPOINT = 0x30

BOOT_WAIT_SECONDS = 8

START_THRUST = 5000
TARGET_THRUST = 100000


RAMP_UP_SECONDS = 5
HOLD_SECONDS = 3


def make_packet(thrust: int) -> bytes:
    thrust = max(0, min(65535, int(thrust)))

    crtp = bytes([CRTP_SETPOINT]) + struct.pack(
        "<fffH",
        0.0,  # roll
        0.0,  # pitch
        0.0,  # yaw rate
        thrust,
    )

    checksum = sum(crtp) & 0xFF
    payload = crtp + bytes([checksum])

    return bytes([SYNC, len(payload)]) + payload


def send_thrust(ser: serial.Serial, thrust: int) -> None:
    ser.write(make_packet(thrust))


def hold_thrust(
    ser: serial.Serial,
    thrust: int,
    seconds: float,
) -> None:
    period = 1.0 / RATE_HZ
    end_time = time.monotonic() + seconds
    next_send = time.monotonic()

    print(f"Holding thrust at {thrust}")

    while time.monotonic() < end_time:
        send_thrust(ser, thrust)

        next_send += period
        remaining = next_send - time.monotonic()

        if remaining > 0:
            time.sleep(remaining)


def ramp_up(
    ser: serial.Serial,
    start_thrust: int,
    target_thrust: int,
    seconds: float,
) -> None:
    total_steps = max(1, int(seconds * RATE_HZ))
    period = 1.0 / RATE_HZ
    next_send = time.monotonic()

    print(
        f"Ramping from {start_thrust} to {target_thrust} "
        f"over {seconds} seconds"
    )

    last_printed_bucket = None

    for step in range(total_steps + 1):
        progress = step / total_steps

        thrust = round(
            start_thrust
            + ((target_thrust - start_thrust) * progress)
        )

        send_thrust(ser, thrust)

        bucket = thrust // 1000

        if bucket != last_printed_bucket:
            print(f"Thrust: {thrust}")
            last_printed_bucket = bucket

        next_send += period
        remaining = next_send - time.monotonic()

        if remaining > 0:
            time.sleep(remaining)


with serial.Serial(
    PORT,
    BAUD,
    timeout=0.1,
    write_timeout=2.0,
) as ser:
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("Connected to XIAO transmitter.")
    print("Waiting for drone boot and calibration.")
    print("Keep the drone completely still.")

    time.sleep(BOOT_WAIT_SECONDS)

    try:
        print("Sending zero thrust.")
        hold_thrust(ser, 0, 2)

        ramp_up(
            ser,
            START_THRUST,
            TARGET_THRUST,
            RAMP_UP_SECONDS,
        )

        hold_thrust(
            ser,
            TARGET_THRUST,
            HOLD_SECONDS,
        )

    except KeyboardInterrupt:
        print("\nEmergency stop requested.")

    finally:
        print("Stopping motors immediately.")
        hold_thrust(ser, 0, 3)

print("Done.")