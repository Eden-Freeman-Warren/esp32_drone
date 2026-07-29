# ESP32-S3 Drone Toward Autonomous Flight

This repository documents the development of an ESP32-S3 micro drone with ESP-NOW communication, Python control, and external-position data.

The goal is to provide enough information for another person to reproduce the project up through the external-position and Kalman-estimator integration.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Downloading the Firmware](#downloading-the-firmware)
- [Creating the ESP-NOW Drone Transport](#creating-the-esp-now-drone-transport)
- [Updating CMakeLists.txt](#updating-cmakeliststxt)
- [Connecting the Transport to the Crazyflie Stack](#connecting-the-transport-to-the-crazyflie-stack)
- [ESP-NOW Packet Format](#esp-now-packet-format)
- [Programming the XIAO Transmitter](#programming-the-xiao-transmitter)
- [Python ESP-NOW Driver](#python-esp-now-driver)
- [Testing the Communication Link](#testing-the-communication-link)
- [Enabling External-Position Packets](#enabling-external-position-packets)
- [Python External-Position Program](#python-external-position-program)
- [HOME Position](#home-position)
- [Current Project Status](#current-project-status)

---

## Project Overview

The communication system uses the following path:

```text
Computer
   ↓ USB Serial
XIAO ESP32-S3 Transmitter
   ↓ ESP-NOW
ESP32-S3 Drone
```

The return communication path is:

```text
ESP32-S3 Drone
   ↓ ESP-NOW
XIAO ESP32-S3 Transmitter
   ↓ USB Serial
Computer
```

The project currently includes:

- ESP-NOW communication between the transmitter and drone
- Full-duplex communication
- A custom Python cflib communication driver
- External-position data
- Kalman-estimator integration
- Position setpoint commands
- Motor and hover testing

---

## Downloading the Firmware

1. Download or clone the ESP-FLY repository:

   [Seeed Projects: Co-Create ESP-FLY](https://github.com/Seeed-Projects/Co-Create_ESP-FLY.git)

2. Clone the repository using Git:

   ```bash
   git clone https://github.com/Seeed-Projects/Co-Create_ESP-FLY.git
   ```

3. Open the firmware folder in Visual Studio Code.

4. Install the ESP-IDF extension in Visual Studio Code.

5. Use ESP-IDF to:

   - Configure the firmware
   - Build the firmware
   - Flash the firmware
   - Open the serial monitor

6. Confirm that the original firmware builds successfully before making any modifications.

---

## Creating the ESP-NOW Drone Transport

Create two new files:

- A header file
- A C source file

### Header File

Example filename:

```text
serial_transport.h
```

Place it in:

```text
components/core/crazyflie/hal/interface/
```

### Source File

Example filename:

```text
serial_transport.c
```

Place it in:

```text
components/core/crazyflie/hal/src/
```

The ESP-NOW transport should:

- Initialize Wi-Fi in station mode
- Disable Wi-Fi power saving
- Set the ESP32-S3 to a fixed ESP-NOW channel
- Initialize ESP-NOW
- Register an ESP-NOW receive callback
- Register an ESP-NOW send callback
- Store received packets in a FreeRTOS queue
- Retrieve packets from the queue
- Forward incoming packets to the CRTP communication system
- Send outgoing CRTP replies through ESP-NOW

Both the drone and the transmitter must use the same ESP-NOW channel.

Example:

```c
#define ESPNOW_CHANNEL 1
```

The receive callback should remain short and should not block.

---

## Updating CMakeLists.txt

Open the core component file:

```text
components/core/crazyflie/CMakeLists.txt
```

Add the new C source file to the `SRCS` section.

Example:

```cmake
"./hal/src/serial_transport.c"
```

Make sure the header directory is included in `INCLUDE_DIRS`.

Example:

```cmake
INCLUDE_DIRS
    "."
    "./hal/interface"
    "./modules/interface"
```

Add any required ESP-IDF components to the `REQUIRES` section.

Examples may include:

```cmake
REQUIRES
    esp_wifi
    esp_event
    esp_netif
    nvs_flash
```

Only add components that are actually used by the source file.

Do not add the header filename itself to `REQUIRES`.

After changing `CMakeLists.txt`, run:

```bash
idf.py fullclean
idf.py build
```

If the build succeeds, flash the firmware:

```bash
idf.py flash
```

---

## Connecting the Transport to the Crazyflie Stack

Open the existing communication file:

```text
components/core/crazyflie/hal/src/wifilink.c
```

Replace the original Wi-Fi or UDP transmission functions with the new ESP-NOW transport functions.

For example, replace:

```c
wifiSendData(dataSize, data);
```

with:

```c
serialSendData(dataSize, data);
```

Include the new transport header:

```c
#include "serial_transport.h"
```

Any code that previously handled UDP packets should be changed to use the new serial and ESP-NOW packet system.

The communication must work in both directions:

```text
Computer → XIAO → Drone
Drone → XIAO → Computer
```

Full-duplex communication is required for cflib to:

- Receive connection responses
- Download the parameter table
- Download the logging table
- Read telemetry
- Send commands

---

## ESP-NOW Packet Format

The computer and XIAO communicate through USB serial.

The serial packet format is:

```text
0xAA | Length | CRTP Data | Checksum
```

### Synchronization Byte

```text
0xAA
```

### Length

The length byte includes:

- The CRTP packet
- The checksum byte

### Checksum

The checksum is calculated by adding all CRTP bytes:

```python
checksum = sum(crtp_data) & 0xFF
```

### Packet Size

A standard CRTP packet contains:

```text
1-byte CRTP header + up to 30 data bytes
```

The maximum raw CRTP size is therefore:

```text
31 bytes
```

Including the checksum, the maximum drone-bound payload is:

```text
32 bytes
```

The XIAO should validate the checksum before forwarding a packet to the drone.

The drone-side receiver should also validate the expected packet format.

Both directions must use matching framing rules.

---

## Programming the XIAO Transmitter

A Seeed XIAO ESP32-S3 is used as the transmitter.

The XIAO can be programmed using the Arduino IDE.

### Arduino Setup

1. Install the ESP32 board package.
2. Select the Seeed XIAO ESP32-S3 board.
3. Select the correct COM port.
4. Upload the transmitter code.

The XIAO transmitter should:

- Open USB serial communication at `115200` baud
- Read binary CRTP packets from the computer
- Detect the `0xAA` synchronization byte
- Read the packet length
- Validate the packet checksum
- Send valid packets to the drone using ESP-NOW
- Receive ESP-NOW replies from the drone
- Add serial framing to the replies
- Send the replies back to the computer

Do not use:

```cpp
Serial.print()
```

or:

```cpp
Serial.println()
```

while the binary communication system is running.

Printed text can corrupt the CRTP serial stream.

Use FreeRTOS queues so that ESP-NOW callbacks do not block.

The transmitter must use:

- The same ESP-NOW channel as the drone
- The drone’s correct Wi-Fi station MAC address

Example:

```cpp
static uint8_t droneAddress[6] = {
    0x90, 0x70, 0x69, 0x10, 0xAB, 0x78
};
```

Replace this address with the actual station MAC address of the drone.

---

## Python ESP-NOW Driver

Create a custom Python cflib driver.

Example filename:

```text
espnow_driver.py
```

The driver should recognize a URI such as:

```text
espnow://COM9/115200
```

The Python driver should:

- Open the selected COM port
- Convert cflib CRTP packets into the serial packet format
- Add the synchronization byte
- Add the packet length
- Add the checksum
- Send packets to the XIAO
- Continuously read replies from the XIAO
- Validate received packet lengths
- Validate received checksums
- Convert received data back into cflib CRTP packets
- Pass received packets to cflib
- Include a timeout if no reply is received
- Wait briefly after opening the serial port because the XIAO may reset

Register the driver before opening the connection:

```python
from espnow_driver import register_espnow_driver

register_espnow_driver()
```

Example URI:

```python
URI = "espnow://COM9/115200"
```

---

## Testing the Communication Link

Test the ESP-NOW link before attempting flight.

Remove the propellers before testing.

Close:

- Arduino Serial Monitor
- ESP-IDF Serial Monitor
- Any other program using the XIAO COM port

Power-cycle both the XIAO and drone.

Run the communication test:

```bash
python espnow_link_test.py
```

A successful test should show messages similar to:

```text
First valid CRTP reply received
CONNECTED: TOCs downloaded
PASS: full-duplex cflib communication works
```

A successful test confirms that:

- The computer can send packets to the XIAO
- The XIAO can send packets to the drone
- The drone can return packets to the XIAO
- The XIAO can return packets to the computer
- cflib can download the log table
- cflib can download the parameter table

If no reply is received, check:

- The drone MAC address
- The ESP-NOW channel
- The COM port
- The battery connection
- The transmitter firmware
- The drone firmware
- Whether the drone completed startup

---

## Enabling External-Position Packets

The Crazyflie localization service must be enabled in the drone firmware.

Open the communication initialization file and find the localization initialization call.

Enable:

```c
locSrvInit();
```

Make sure the following file is included in the build:

```text
crtp_localization_service.c
```

Add it to `CMakeLists.txt` if necessary:

```cmake
"./modules/src/crtp_localization_service.c"
```

The original localization service may include optional dependencies for:

- LPS positioning
- Lighthouse positioning
- Peer localization
- Additional positioning decks

These optional functions may cause linker errors if they are not included in the firmware.

A simplified localization service can be used when only external-position packets are required.

The localization service should:

- Receive external-position CRTP packets
- Decode X, Y, and Z
- Send the measurements to the onboard estimator

---

## Python External-Position Program

The Python flight program receives position data from the motion-capture system and sends it to the drone.

The motion-capture system is not being developed in this project. It is only being used as an external position source.

The data path is:

```text
Motion-capture cameras
        ↓
Motive
        ↓ NatNet
Python program
        ↓ cflib
XIAO ESP32-S3
        ↓ ESP-NOW
Drone
        ↓
Kalman estimator
```

The Python program needs the following files in the same folder:

```text
espnow_test_ready_V6.py
espnow_driver.py
NatNetClient.py
```

---

## Configure the Connection Information

At the top of the Python program, enter the correct connection information.

```python
URI = "espnow://COM9/115200"

CLIENT_IP = "192.168.0.13"
SERVER_IP = "192.168.0.4"

RIGID_BODY_ID = 555
```

These values mean:

- `URI` is the XIAO serial port and baud rate.
- `CLIENT_IP` is the IP address of the computer running Python.
- `SERVER_IP` is the IP address of the computer running Motive.
- `RIGID_BODY_ID` identifies the drone inside Motive.

The IP addresses and COM port may be different on another computer.

---

## Start the NatNet Client

Create a NatNet client in Python.

```python
optitrack = NatNetClient()

optitrack.set_client_address(CLIENT_IP)
optitrack.set_server_address(SERVER_IP)
optitrack.set_use_multicast(True)

optitrack.rigid_body_listener = receive_rigid_body
```

Start the NatNet client:

```python
if not optitrack.run():
    raise RuntimeError("Could not start the NatNet client.")
```

The NatNet client receives rigid-body position and quaternion data from Motive.

---

## Store the Latest Rigid-Body Data

The NatNet callback should only save the newest measurement.

It should not send packets to the drone directly.

```python
def receive_rigid_body(
    rigid_body_id,
    position,
    quaternion,
    *extra,
):
    global latest_world_position
    global latest_world_quaternion
    global latest_sample_time

    if int(rigid_body_id) != RIGID_BODY_ID:
        return

    with data_lock:
        latest_world_position = (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )

        latest_world_quaternion = (
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        )

        latest_sample_time = time.monotonic()
```

The callback stores:

- World X position
- World Y position
- World Z position
- Rigid-body quaternion
- Time of the newest sample

A separate thread sends the data to the drone later.

This prevents the NatNet callback from becoming blocked by serial or ESP-NOW communication.

---

## Wait for Valid Tracking

Before continuing, the program should wait until rigid body `555` is being tracked.

```python
def wait_for_tracking():
    print(
        f"Waiting for rigid body {RIGID_BODY_ID}..."
    )

    while True:
        with data_lock:
            position = latest_world_position
            update_time = latest_sample_time

        age = time.monotonic() - update_time

        if position is not None and age < 0.25:
            print("Tracking is active.")
            return

        time.sleep(0.05)
```

The program should not continue if the position data is missing or outdated.

---

## Capture the HOME Position

The drone should be placed still and level before HOME is captured.

HOME becomes the local origin:

```text
X = 0
Y = 0
Z = 0
```

The program averages several motion-capture samples instead of using only one sample.

```python
home_position = (
    average_x,
    average_y,
    average_z,
)
```

Future measurements are measured relative to HOME.

```python
dx_world = world_x - home_x
dy_world = world_y - home_y
dz_world = world_z - home_z
```

This allows the drone to begin at local position:

```text
(0, 0, 0)
```

even if its Motive world position is something such as:

```text
(-5.35, 1.57, 0.32)
```

---

## Capture the HOME Yaw

Motive reports position using the fixed room coordinate system.

The drone controller expects:

```text
+X = forward
+Y = left
+Z = up
```

The drone may not be pointing in the same direction as Motive world `+X`.

For this reason, the program captures the drone’s starting yaw from the rigid-body quaternion.

Convert the quaternion to yaw:

```python
def quaternion_to_yaw(quaternion):
    qx, qy, qz, qw = quaternion

    sin_yaw = 2.0 * (
        qw * qz + qx * qy
    )

    cos_yaw = 1.0 - 2.0 * (
        qy * qy + qz * qz
    )

    return math.atan2(
        sin_yaw,
        cos_yaw,
    )
```

The average starting yaw is stored as:

```python
home_yaw_radians
```

---

## Convert World Position to Local Drone Coordinates

After subtracting HOME, rotate the X and Y position by negative HOME yaw.

```python
def local_position(world_position):
    dx_world = (
        world_position[0] - home_position[0]
    )

    dy_world = (
        world_position[1] - home_position[1]
    )

    dz_world = (
        world_position[2] - home_position[2]
    )

    cosine = math.cos(home_yaw_radians)
    sine = math.sin(home_yaw_radians)

    x_forward = (
        cosine * dx_world
        + sine * dy_world
    )

    y_left = (
        -sine * dx_world
        + cosine * dy_world
    )

    z_up = dz_world

    return (
        x_forward,
        y_left,
        z_up,
    )
```

The result should follow this coordinate system:

```text
Forward  = positive X
Backward = negative X

Left     = positive Y
Right    = negative Y

Up       = positive Z
Down     = negative Z
```

Verify these directions manually before installing the propellers.

---

## Send External Position to the Drone

The program sends local position to the drone using:

```python
cf.extpos.send_extpos(
    x,
    y,
    z,
)
```

External position should be sent continuously.

A separate thread can send it at approximately 30 Hz.

```python
EXTPOS_RATE_HZ = 30.0
EXTPOS_PERIOD = 1.0 / EXTPOS_RATE_HZ
```

Example sender loop:

```python
while not stop_event.is_set():
    world_position, age = read_tracking()

    if world_position is None:
        raise RuntimeError(
            "No position data available."
        )

    if age > 0.25:
        raise RuntimeError(
            "Position data became stale."
        )

    x, y, z = local_position(
        world_position
    )

    cf.extpos.send_extpos(
        x,
        y,
        z,
    )

    time.sleep(EXTPOS_PERIOD)
```

The program should send at least several dozen external-position packets before resetting the estimator.

Example:

```python
MIN_EXT_POS_PACKETS = 60
```

---

## Connect to the Drone

Register the custom ESP-NOW driver before opening the connection.

```python
register_espnow_driver()
```

Open the cflib connection:

```python
with SyncCrazyflie(
    URI,
    cf=Crazyflie(rw_cache="./cache"),
) as scf:
    cf = scf.cf
```

A successful connection confirms that:

- Python can communicate with the XIAO.
- The XIAO can communicate with the drone.
- The drone can return CRTP responses.
- cflib can download the parameter table.
- cflib can download the logging table.

---

## Select the Estimator and Controller

Select the Kalman estimator:

```python
cf.param.set_value(
    "stabilizer.estimator",
    "2",
)
```

Select the PID controller:

```python
cf.param.set_value(
    "stabilizer.controller",
    "1",
)
```

The Kalman estimator combines:

- IMU measurements
- External X position
- External Y position
- External Z position

The estimator produces the position and attitude used by the onboard controller.

---

## Reset the Kalman Estimator

After external-position packets are being received, reset the estimator.

```python
cf.param.set_value(
    "kalman.resetEstimation",
    "1",
)

time.sleep(0.1)

cf.param.set_value(
    "kalman.resetEstimation",
    "0",
)
```

Do not reset the estimator before the external-position stream has started.

---

## Wait for Estimator Convergence

Monitor these variables:

```text
kalman.varPX
kalman.varPY
kalman.varPZ
```

Create a log configuration:

```python
log_config = LogConfig(
    name="Kalman variance",
    period_in_ms=500,
)

log_config.add_variable(
    "kalman.varPX",
    "float",
)

log_config.add_variable(
    "kalman.varPY",
    "float",
)

log_config.add_variable(
    "kalman.varPZ",
    "float",
)
```

The values should decrease and become stable.

Example:

```text
Kalman variance PX=0.000280
Kalman variance PY=0.000280
Kalman variance PZ=0.000350
```

Do not arm the drone if:

- The values continue increasing.
- The values change significantly.
- External-position packets stop.
- The position data becomes stale.

A successful result should print:

```text
Estimator converged.
```

---

## Enable Position Setpoint Mode

The ESP-Drone firmware uses a firmware-specific position mode.

Disable the high-level commander:

```python
cf.param.set_value(
    "commander.enHighLevel",
    "0",
)
```

Enable position setpoint mode:

```python
cf.param.set_value(
    "flightmode.posSet",
    "1",
)
```

In this mode, the legacy commander packet is interpreted as a position command.

The mapping is:

```text
Roll field   → Y position
Pitch field  → negative X position
Yaw field    → yaw
Thrust field → Z position in millimeters
```

---

## Send a Position Setpoint

Create a function to send X, Y, Z, and yaw.

```python
def send_position_setpoint(
    cf,
    x_m,
    y_m,
    z_m,
    yaw_degrees,
):
    z_millimeters = int(
        z_m * 1000.0
    )

    cf.commander.send_setpoint(
        float(y_m),
        float(-x_m),
        float(yaw_degrees),
        z_millimeters,
    )
```

For example:

```python
send_position_setpoint(
    cf=cf,
    x_m=0.0,
    y_m=0.0,
    z_m=0.10,
    yaw_degrees=0.0,
)
```

This commands:

```text
X = 0.00 meters
Y = 0.00 meters
Z = 0.10 meters
Yaw = 0 degrees
```

The Python program does not calculate motor outputs.

The onboard drone firmware calculates:

```text
Position error
    ↓
Position PID
    ↓
Desired roll and pitch
    ↓
Attitude PID
    ↓
Motor mixer
    ↓
Motor outputs
```

---

## Arm the Drone

The firmware reports an older CRTP protocol version, so the normal cflib supervisor arming command may not work.

Use:

```python
cf.param.set_value(
    "system.forceArm",
    "1",
)
```

To disarm:

```python
cf.param.set_value(
    "system.forceArm",
    "0",
)
```

Always disarm during:

- Normal shutdown
- Errors
- Keyboard interruption
- Loss of position data
- Loss of communication

---

## Recommended Test Order

Use the following order when reproducing the project:

1. Build the original firmware.
2. Flash the original firmware.
3. Add the ESP-NOW drone transport.
4. Program the XIAO transmitter.
5. Test one-way ESP-NOW communication.
6. Test full-duplex ESP-NOW communication.
7. Test the Python cflib driver.
8. Confirm the parameter table downloads.
9. Confirm the log table downloads.
10. Enable the localization service.
11. Start the NatNet client.
12. Confirm rigid-body tracking.
13. Capture HOME position.
14. Capture HOME yaw.
15. Verify local coordinate directions.
16. Begin sending external-position packets.
17. Reset the Kalman estimator.
18. Wait for estimator convergence.
19. Test position commands with propellers removed.
20. Verify motor order.
21. Verify propeller direction.
22. Perform restrained takeoff testing.

---

## Current Result

The project has successfully demonstrated:

- Full-duplex ESP-NOW communication
- Python-to-drone CRTP communication
- cflib parameter-table download
- cflib logging-table download
- External-position packet delivery
- HOME position conversion
- HOME yaw conversion
- Kalman-estimator convergence
- Position-setpoint commands
- Motor activation

Stable autonomous hovering is still under development.

Current issues being investigated include:

- Battery power delivery
- Battery weight
- Center-of-mass balance
- Motor saturation
- Motor and propeller arrangement
- Horizontal drift
- Takeoff wobble
- PID tuning