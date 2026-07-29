#!/usr/bin/env python3
"""
Autonomous ESP32-S3 ESP-Drone hover using generic position setpoints.

Changes in this version:
- OptiTrack callback only stores the newest sample.
- A dedicated thread sends external position at a controlled 30 Hz.
- Kalman reset has visible variance output and a timeout.
- The program never waits forever at estimator convergence.

Keep together:
    espnow_test_ready_v2.py
    espnow_driver.py
    NatNetClient.py
"""

import threading
import time

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

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

# Keep this controlled. Do not send from every NatNet callback.
EXTPOS_RATE_HZ = 30.0
EXTPOS_PERIOD = 1.0 / EXTPOS_RATE_HZ
MIN_EXT_POS_PACKETS = 60

ESTIMATOR_LOG_PERIOD_MS = 500
ESTIMATOR_HISTORY_LENGTH = 10
ESTIMATOR_VARIANCE_SPREAD_LIMIT = 0.001
ESTIMATOR_TIMEOUT = 15.0

TELEMETRY_PERIOD_MS = 200

POSITION_COMMAND_RATE_HZ = 20.0
POSITION_COMMAND_PERIOD = 1.0 / POSITION_COMMAND_RATE_HZ


data_lock = threading.Lock()

latest_world_position = None
latest_sample_time = 0.0
home_position = None

cf = None


def receive_rigid_body(
    rigid_body_id,
    position,
    quaternion,
    *extra,
):
    """NatNet callback: store data only; never transmit from this callback."""
    del quaternion, extra

    global latest_world_position
    global latest_sample_time

    if int(rigid_body_id) != RIGID_BODY_ID:
        return

    sample = (
        float(position[0]),
        float(position[1]),
        float(position[2]),
    )

    with data_lock:
        latest_world_position = sample
        latest_sample_time = time.monotonic()


def read_tracking():
    with data_lock:
        position = latest_world_position
        update_time = latest_sample_time

    if position is None:
        return None, float("inf")

    return position, time.monotonic() - update_time


def local_position(world_position):
    with data_lock:
        home = home_position

    if home is None:
        raise RuntimeError("HOME has not been captured.")

    return (
        world_position[0] - home[0],
        world_position[1] - home[1],
        world_position[2] - home[2],
    )


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
            raise RuntimeError(
                "OptiTrack became stale during HOME capture."
            )

        for axis in range(3):
            sums[axis] += position[axis]

        count += 1
        time.sleep(0.02)

    if count == 0:
        raise RuntimeError("No samples captured for HOME.")

    with data_lock:
        home_position = (
            sums[0] / count,
            sums[1] / count,
            sums[2] / count,
        )
        home = home_position

    print(
        "HOME captured: "
        f"({home[0]:+.3f}, {home[1]:+.3f}, {home[2]:+.3f}) m"
    )
    print("The estimator will receive local position (0, 0, 0).")


class ExternalPositionSender:
    def __init__(self, crazyflie):
        self._cf = crazyflie
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="ExternalPositionSender",
            daemon=True,
        )

        self.packet_count = 0
        self.error = None

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self):
        next_send = time.monotonic()

        try:
            while not self._stop.is_set():
                position, age = read_tracking()

                if position is None or age > TRACKING_TIMEOUT:
                    raise RuntimeError(
                        f"OptiTrack stale in pose sender: {age:.3f}s"
                    )

                x, y, z = local_position(position)

                self._cf.extpos.send_extpos(x, y, z)
                self.packet_count += 1

                next_send += EXTPOS_PERIOD
                delay = next_send - time.monotonic()

                if delay > 0:
                    self._stop.wait(delay)
                else:
                    next_send = time.monotonic()

        except Exception as error:
            self.error = error
            self._stop.set()


def set_parameter(crazyflie, name, value):
    print(f"Setting {name} = {value}...")
    crazyflie.param.set_value(name, value)
    time.sleep(0.25)


def wait_for_external_position(sender):
    print(
        f"Feeding local OptiTrack position at "
        f"{EXTPOS_RATE_HZ:.0f} Hz..."
    )

    deadline = time.monotonic() + 10.0

    while sender.packet_count < MIN_EXT_POS_PACKETS:
        if sender.error is not None:
            raise sender.error

        if time.monotonic() > deadline:
            raise RuntimeError(
                "External-position stream did not start."
            )

        time.sleep(0.02)

    print(
        f"External-position packets sent: {sender.packet_count}"
    )


def reset_and_wait_for_estimator(crazyflie):
    """
    Reset the ESP-Drone Kalman estimator and wait for stable velocity
    covariance. Unlike cflib's helper, this version prints values and times out.
    """
    print("Resetting Kalman estimator...")

    crazyflie.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.10)
    crazyflie.param.set_value("kalman.resetEstimation", "0")

    log_config = LogConfig(
        name="Kalman variance",
        period_in_ms=ESTIMATOR_LOG_PERIOD_MS,
    )
    log_config.add_variable("kalman.varPX", "float")
    log_config.add_variable("kalman.varPY", "float")
    log_config.add_variable("kalman.varPZ", "float")

    samples = []
    sample_event = threading.Event()
    error_holder = {"error": None}

    def on_data(timestamp, data, logconf):
        del timestamp, logconf

        sample = (
            float(data["kalman.varPX"]),
            float(data["kalman.varPY"]),
            float(data["kalman.varPZ"]),
        )
        samples.append(sample)

        if len(samples) > ESTIMATOR_HISTORY_LENGTH:
            samples.pop(0)

        print(
            "Kalman variance "
            f"PX={sample[0]:.6f} "
            f"PY={sample[1]:.6f} "
            f"PZ={sample[2]:.6f}"
        )

        sample_event.set()

    def on_error(logconf, message):
        del logconf
        error_holder["error"] = RuntimeError(
            f"Kalman log error: {message}"
        )
        sample_event.set()

    crazyflie.log.add_config(log_config)

    if not log_config.valid:
        raise RuntimeError(
            "The firmware does not expose kalman.varPX/varPY/varPZ."
        )

    log_config.data_received_cb.add_callback(on_data)
    log_config.error_cb.add_callback(on_error)
    log_config.start()

    deadline = time.monotonic() + ESTIMATOR_TIMEOUT

    try:
        print("Waiting for estimator convergence...")

        while time.monotonic() < deadline:
            sample_event.wait(timeout=1.0)
            sample_event.clear()

            if error_holder["error"] is not None:
                raise error_holder["error"]

            if len(samples) < ESTIMATOR_HISTORY_LENGTH:
                continue

            spread_x = max(v[0] for v in samples) - min(
                v[0] for v in samples
            )
            spread_y = max(v[1] for v in samples) - min(
                v[1] for v in samples
            )
            spread_z = max(v[2] for v in samples) - min(
                v[2] for v in samples
            )

            print(
                "Variance spread "
                f"X={spread_x:.6f} "
                f"Y={spread_y:.6f} "
                f"Z={spread_z:.6f}"
            )

            if (
                spread_x < ESTIMATOR_VARIANCE_SPREAD_LIMIT
                and spread_y < ESTIMATOR_VARIANCE_SPREAD_LIMIT
                and spread_z < ESTIMATOR_VARIANCE_SPREAD_LIMIT
            ):
                print("Estimator converged.")
                return

        raise RuntimeError(
            "Kalman estimator did not converge within "
            f"{ESTIMATOR_TIMEOUT:.0f} seconds. "
            "Do not fly; inspect the printed variance values."
        )

    finally:
        log_config.stop()



class FlightTelemetry:
    """Log the complete high-level-command-to-motor path."""

    def __init__(self, crazyflie):
        self._cf = crazyflie
        self._configs = []

    def start(self):
        control_log = LogConfig(
            name="Flight control path",
            period_in_ms=TELEMETRY_PERIOD_MS,
        )
        control_log.add_variable("sys.armed", "uint8_t")
        control_log.add_variable("health.checkStops", "uint8_t")
        control_log.add_variable("ctrltarget.z", "float")
        control_log.add_variable("stateEstimate.z", "float")
        control_log.add_variable("stabilizer.thrust", "float")
        control_log.add_variable(
            "controller.actuatorThrust",
            "float",
        )

        motor_log = LogConfig(
            name="Motor outputs",
            period_in_ms=TELEMETRY_PERIOD_MS,
        )
        motor_log.add_variable("motor.m1", "uint32_t")
        motor_log.add_variable("motor.m2", "uint32_t")
        motor_log.add_variable("motor.m3", "uint32_t")
        motor_log.add_variable("motor.m4", "uint32_t")

        self._cf.log.add_config(control_log)
        self._cf.log.add_config(motor_log)

        if not control_log.valid:
            raise RuntimeError(
                "One or more control-path log variables are absent "
                "from the firmware TOC."
            )

        if not motor_log.valid:
            raise RuntimeError(
                "The motor.m1/m2/m3/m4 log variables are absent "
                "from the firmware TOC."
            )

        control_log.data_received_cb.add_callback(
            self._on_control
        )
        motor_log.data_received_cb.add_callback(
            self._on_motors
        )

        control_log.start()
        motor_log.start()

        self._configs = [control_log, motor_log]

    def stop(self):
        for config in self._configs:
            try:
                config.stop()
            except Exception:
                pass
        self._configs = []

    @staticmethod
    def _on_control(timestamp, data, logconf):
        del timestamp, logconf

        print(
            "CTRL "
            f"armed={int(data['sys.armed'])} "
            f"gate={int(data['health.checkStops'])} "
            f"zCmd={data['ctrltarget.z']:+.3f} "
            f"zEst={data['stateEstimate.z']:+.3f} "
            f"actT={data['controller.actuatorThrust']:.0f} "
            f"outT={data['stabilizer.thrust']:.0f}"
        )

    @staticmethod
    def _on_motors(timestamp, data, logconf):
        del timestamp, logconf

        print(
            "MOTOR "
            f"{int(data['motor.m1'])},"
            f"{int(data['motor.m2'])},"
            f"{int(data['motor.m3'])},"
            f"{int(data['motor.m4'])}"
        )


def force_arm_legacy_esp_drone(crazyflie, enabled):
    """
    ESP-Drone protocol 4 does not implement cflib's legacy platform
    arming command. Use the firmware's system.forceArm parameter.
    """
    group = crazyflie.param.toc.toc.get("system", {})

    if "forceArm" not in group:
        raise RuntimeError(
            "Firmware parameter system.forceArm is missing. "
            "Patch platformservice.c to implement command 0x01, "
            "or expose system.forceArm."
        )

    value = "1" if enabled else "0"
    print(f"Setting system.forceArm = {value}...")
    crazyflie.param.set_value("system.forceArm", value)
    time.sleep(0.30)



def read_estimated_pose(crazyflie):
    """Read one estimated Z and yaw sample before starting position control."""
    log_config = LogConfig(
        name="Initial estimated pose",
        period_in_ms=100,
    )
    log_config.add_variable("stateEstimate.z", "float")
    log_config.add_variable("stateEstimate.yaw", "float")

    result = {}
    received = threading.Event()

    def on_data(timestamp, data, logconf):
        del timestamp, logconf
        result["z"] = float(data["stateEstimate.z"])
        result["yaw"] = float(data["stateEstimate.yaw"])
        received.set()

    crazyflie.log.add_config(log_config)

    if not log_config.valid:
        raise RuntimeError(
            "stateEstimate.z or stateEstimate.yaw is missing "
            "from the firmware log TOC."
        )

    log_config.data_received_cb.add_callback(on_data)
    log_config.start()

    try:
        if not received.wait(timeout=2.0):
            raise RuntimeError(
                "No state-estimate sample received."
            )
    finally:
        log_config.stop()

    return result["z"], result["yaw"]


def run_position_segment(
    crazyflie,
    sender,
    start_z,
    end_z,
    seconds,
    yaw_degrees,
    label,
):
    """
    Send absolute X/Y/Z/yaw setpoints continuously.

    The onboard position controller calculates roll, pitch and thrust.
    """
    print(
        f"{label}: Z {start_z:+.3f} -> {end_z:+.3f} m "
        f"over {seconds:.1f} s"
    )

    steps = max(1, int(seconds * POSITION_COMMAND_RATE_HZ))
    next_send = time.monotonic()
    next_print = time.monotonic()

    for step in range(steps + 1):
        if sender.error is not None:
            raise sender.error

        _, age = read_tracking()
        if age > TRACKING_TIMEOUT:
            raise RuntimeError(
                f"OptiTrack stale during {label}: {age:.3f}s"
            )

        progress = step / steps
        target_z = start_z + (end_z - start_z) * progress

        crazyflie.commander.send_position_setpoint(
            0.0,
            0.0,
            target_z,
            yaw_degrees,
        )

        now = time.monotonic()
        if now >= next_print:
            position, _ = read_tracking()
            x, y, z = local_position(position)

            print(
                f"{label:<7} "
                f"POS=({x:+.3f},{y:+.3f},{z:+.3f}) "
                f"CMD=(+0.000,+0.000,{target_z:+.3f})"
            )
            next_print = now + 0.20

        next_send += POSITION_COMMAND_PERIOD
        delay = next_send - time.monotonic()

        if delay > 0:
            time.sleep(delay)
        else:
            next_send = time.monotonic()


def hold_position(
    crazyflie,
    sender,
    target_z,
    seconds,
    yaw_degrees,
):
    print(
        f"HOVER: holding X=0, Y=0, Z={target_z:.3f} m "
        f"for {seconds:.1f} s"
    )

    deadline = time.monotonic() + seconds
    next_send = time.monotonic()
    next_print = time.monotonic()

    while time.monotonic() < deadline:
        if sender.error is not None:
            raise sender.error

        position, age = read_tracking()
        if position is None or age > TRACKING_TIMEOUT:
            raise RuntimeError(
                f"OptiTrack stale during hover: {age:.3f}s"
            )

        crazyflie.commander.send_position_setpoint(
            0.0,
            0.0,
            target_z,
            yaw_degrees,
        )

        now = time.monotonic()
        if now >= next_print:
            x, y, z = local_position(position)

            print(
                "HOVER   "
                f"POS=({x:+.3f},{y:+.3f},{z:+.3f}) "
                f"CMD=(+0.000,+0.000,{target_z:+.3f})"
            )
            next_print = now + 0.20

        next_send += POSITION_COMMAND_PERIOD
        delay = next_send - time.monotonic()

        if delay > 0:
            time.sleep(delay)
        else:
            next_send = time.monotonic()


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

    sender = None
    telemetry = None

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

            sender = ExternalPositionSender(cf)
            sender.start()

            wait_for_external_position(sender)
            reset_and_wait_for_estimator(cf)

            if sender.error is not None:
                raise sender.error

            # The ESP-Drone reports protocol version 4. Its high-level
            # takeoff packet is not becoming an active planner setpoint.
            # Use the generic absolute-position commander instead.
            set_parameter(cf, "commander.enHighLevel", "0")

            estimated_z, estimated_yaw = read_estimated_pose(cf)
            print(
                "Initial estimator state: "
                f"Z={estimated_z:+.3f} m, "
                f"yaw={estimated_yaw:+.1f} deg"
            )

            force_arm_legacy_esp_drone(cf, True)

            telemetry = FlightTelemetry(cf)
            telemetry.start()
            time.sleep(0.75)

            run_position_segment(
                crazyflie=cf,
                sender=sender,
                start_z=estimated_z,
                end_z=HOVER_HEIGHT,
                seconds=TAKEOFF_TIME,
                yaw_degrees=estimated_yaw,
                label="TAKEOFF",
            )

            hold_position(
                crazyflie=cf,
                sender=sender,
                target_z=HOVER_HEIGHT,
                seconds=HOVER_TIME,
                yaw_degrees=estimated_yaw,
            )

            run_position_segment(
                crazyflie=cf,
                sender=sender,
                start_z=HOVER_HEIGHT,
                end_z=0.0,
                seconds=LAND_TIME,
                yaw_degrees=estimated_yaw,
                label="LAND",
            )

            # Hold the zero-height command briefly before stopping.
            hold_position(
                crazyflie=cf,
                sender=sender,
                target_z=0.0,
                seconds=0.5,
                yaw_degrees=estimated_yaw,
            )

            cf.commander.send_stop_setpoint()
            force_arm_legacy_esp_drone(cf, False)

            print("Flight sequence complete.")

    except KeyboardInterrupt:
        print("\nCtrl+C received.")

        if cf is not None:
            try:
                cf.commander.send_stop_setpoint()
                force_arm_legacy_esp_drone(cf, False)
            except Exception:
                pass

    finally:
        if telemetry is not None:
            telemetry.stop()

        if cf is not None:
            try:
                force_arm_legacy_esp_drone(cf, False)
            except Exception:
                pass

        if sender is not None:
            sender.stop()

        cf = None
        optitrack.shutdown()
        print("Program stopped.")


if __name__ == "__main__":
    main()