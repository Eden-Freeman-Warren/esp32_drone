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

Use a NatNet client to receive rigid-body position data.

Configure:

- The computer’s client IP address
- The motion-capture server IP address
- The rigid-body ID

Example:

```python
CLIENT_IP = "192.168.0.13"
SERVER_IP = "192.168.0.4"
RIGID_BODY_ID = 555
```

Store the newest position sample in the NatNet callback.

Do not send CRTP packets directly from the NatNet callback.

Use a separate thread to send position data at a controlled rate.

Example rate:

```python
EXTPOS_RATE_HZ = 30.0
```

Send position data using:

```python
cf.extpos.send_extpos(x, y, z)
```

Wait until enough external-position packets have been sent before resetting the estimator.

Example:

```python
MIN_EXT_POS_PACKETS = 60
```

---

## HOME Position

Keep the drone still while the HOME position is captured.

Average several position samples during the HOME capture period.

Save the average as:

```python
home_position
```

Subtract HOME from every future position measurement:

```python
x_local = x_world - home_x
y_local = y_world - home_y
z_local = z_world - home_z
```

This causes the estimator to begin near:

```text
X = 0
Y = 0
Z = 0
```

---

## HOME-Yaw Coordinate Conversion

The local drone coordinate system is:

```text
+X = Forward
+Y = Left
+Z = Up
```

Motion-capture position is measured in fixed room coordinates.

Capture the rigid body’s yaw while the drone is at HOME.

Calculate the world displacement:

```python
dx_world = world_x - home_x
dy_world = world_y - home_y
dz_world = world_z - home_z
```

Rotate the X and Y displacement into the drone’s HOME frame:

```python
x_forward = (
    math.cos(home_yaw) * dx_world
    + math.sin(home_yaw) * dy_world
)

y_left = (
    -math.sin(home_yaw) * dx_world
    + math.cos(home_yaw) * dy_world
)

z_up = dz_world
```

After the conversion:

- Moving the drone forward should increase X
- Moving the drone backward should decrease X
- Moving the drone left should increase Y
- Moving the drone right should decrease Y
- Lifting the drone should increase Z

Verify these directions manually before installing the propellers.

---

## Kalman Estimator Setup

Select the Kalman estimator:

```python
cf.param.set_value("stabilizer.estimator", "2")
```

Select the PID controller:

```python
cf.param.set_value("stabilizer.controller", "1")
```

Reset the estimator:

```python
cf.param.set_value("kalman.resetEstimation", "1")
time.sleep(0.1)
cf.param.set_value("kalman.resetEstimation", "0")
```

Monitor:

```text
kalman.varPX
kalman.varPY
kalman.varPZ
```

Do not attempt flight unless the estimator converges.

---

## Current Project Status

The following parts currently work:

- ESP-FLY firmware builds and flashes
- ESP-NOW communication from the XIAO to the drone
- ESP-NOW communication from the drone to the XIAO
- Full-duplex communication
- Custom Python cflib driver
- Parameter table download
- Log table download
- External-position packet delivery
- Kalman-estimator convergence
- Position-setpoint commands
- Motor activation

The project is currently being tested for:

- Stable takeoff
- Stable hovering
- Horizontal position control
- Motor and propeller configuration
- Battery power delivery
- PID tuning
- Center-of-mass balance

Stable autonomous hovering has not yet been fully achieved.

---

## Safety

- Remove all propellers during communication and motor-order testing.
- Use a flight cage or loose tether during early flight tests.
- Keep a method available to disconnect the battery immediately.
- Do not use a battery with a higher voltage than the drone hardware supports.
- Stop testing if the battery becomes hot, swollen, or damaged.
- Stop testing if the drone repeatedly flips or motors remain saturated.
- Confirm motor order and propeller direction before flight.

---

## Credits

- Original ESP-FLY firmware:
  [Seeed Projects: Co-Create ESP-FLY](https://github.com/Seeed-Projects/Co-Create_ESP-FLY)
- Flight-control architecture based on the ESP-Drone and Crazyflie software stack.