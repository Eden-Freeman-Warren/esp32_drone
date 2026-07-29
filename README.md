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

## Connecting the Drone to OptiTrack

OptiTrack is used to provide the drone with its measured X, Y, and Z position.

The OptiTrack data is received on the computer using the NatNet SDK. The computer then sends the position to the drone through cflib and the ESP-NOW communication link.

The complete path is:

```text
OptiTrack Cameras
        ↓
Motive
        ↓ NatNet
Python Program
        ↓ cflib
XIAO ESP32-S3
        ↓ ESP-NOW
Drone
```

---

## Required Python Files

Place the following files in the same folder:

```text
espnow_test_ready.py
espnow_driver.py
NatNetClient.py
```

The purpose of each file is:

- `espnow_test_ready.py`
  - Connects to OptiTrack
  - Receives the drone position
  - Sends the position to the drone
  - Resets the estimator
  - Sends flight commands

- `espnow_driver.py`
  - Allows cflib to communicate through the XIAO and ESP-NOW

- `NatNetClient.py`
  - Receives tracking information from Motive

The NatNet client file can be obtained from the OptiTrack NatNet SDK or its Python examples.

---

## OptiTrack Network Settings

The computer running the Python program and the computer running Motive must be connected to the same network.

Set the IP addresses in the Python program:

```python
CLIENT_IP = "192.168.0.13"
SERVER_IP = "192.168.0.4"
```

- `CLIENT_IP` is the IP address of the computer running the Python script.
- `SERVER_IP` is the IP address of the computer running Motive.

The exact addresses will be different on another network.

The NatNet client is configured with:

```python
optitrack = NatNetClient()

optitrack.set_client_address(CLIENT_IP)
optitrack.set_server_address(SERVER_IP)
optitrack.set_use_multicast(True)
```

The rigid-body callback is then assigned:

```python
optitrack.rigid_body_listener = receive_rigid_body
```

Start the NatNet client with:

```python
optitrack.run()
```

---

## Creating the Drone Rigid Body

In Motive, the markers attached to the drone must be grouped into a rigid body.

To create the rigid body:

1. Attach at least three reflective markers to the drone.
2. Make sure the markers are not arranged in a perfectly symmetrical pattern.
3. Place the drone inside the tracking area.
4. Select the markers in Motive.
5. Create a new rigid body from the selected markers.
6. Give the rigid body a recognizable name.
7. Record the rigid-body ID shown in Motive.

The rigid-body ID must match the value in the Python program.

Example:

```python
RIGID_BODY_ID = 555
```

When the callback receives data, it ignores every rigid body except the selected one:

```python
if int(rigid_body_id) != RIGID_BODY_ID:
    return
```

---

## Rigid-Body Coordinate Directions

The rigid body should be defined so that:

```text
+X = Forward
+Y = Left
+Z = Up
```

The front of the rigid body should match the physical front of the drone.

Before attempting flight, move the drone by hand and verify:

- Moving forward increases X.
- Moving backward decreases X.
- Moving left increases Y.
- Moving right decreases Y.
- Moving upward increases Z.
- Moving downward decreases Z.

If these directions are incorrect, the drone may correct in the wrong direction.

---

## Enabling External Position in the Firmware

The drone firmware must accept external-position packets.

Open:

```text
components/core/crazyflie/modules/src/comm.c
```

Find the localization-service initialization.

Enable:

```c
locSrvInit();
```

If the line is commented out:

```c
// locSrvInit();
```

remove the comment marks:

```c
locSrvInit();
```

---

## Adding the Localization Service to the Build

Make sure this file exists:

```text
components/core/crazyflie/modules/src/crtp_localization_service.c
```

Add it to the `SRCS` section of:

```text
components/core/crazyflie/CMakeLists.txt
```

Example:

```cmake
"./modules/src/crtp_localization_service.c"
```

The localization service receives external-position CRTP packets and sends the measurements to the onboard estimator.

If the original localization file creates linker errors because of unsupported LPS, Lighthouse, or peer-localization functions, use a simplified localization service that only supports external position.

After making the changes, rebuild and flash the firmware:

```bash
idf.py fullclean
idf.py build
idf.py flash
```

---

## Receiving the Rigid-Body Position

The NatNet callback receives:

- The rigid-body ID
- Position
- Rotation quaternion

Example:

```python
def receive_rigid_body(
    rigid_body_id,
    position,
    quaternion,
    *extra,
):
    if int(rigid_body_id) != RIGID_BODY_ID:
        return

    x = float(position[0])
    y = float(position[1])
    z = float(position[2])
```

The newest position should be stored for use by the external-position sender.

Do not send CRTP packets directly from the NatNet callback. The callback should only save the newest measurement.

---

## Sending Position to the Drone

Use a separate loop or thread to send the position to the drone.

Example:

```python
cf.extpos.send_extpos(x, y, z)
```

A sending rate of approximately 30 Hz was used:

```python
EXTPOS_RATE_HZ = 30.0
```

The position should be sent continuously while the estimator and flight controller are running.

---

## Confirming That External Position Works

Before attempting flight:

1. Start Motive.
2. Confirm that the rigid body is visible and tracking.
3. Start the Python program.
4. Confirm that the correct rigid-body ID is detected.
5. Keep the drone still while HOME is captured.
6. Confirm that external-position packets are being sent.
7. Reset the onboard estimator.
8. Watch the Kalman variance values.
9. Do not fly unless the estimator converges.

A successful connection should show output similar to:

```text
Waiting for OptiTrack rigid body 555...
OptiTrack tracking active.
HOME captured.
External-position packets sent: 60
Resetting Kalman estimator...
Estimator converged.
```

If the estimator does not converge, check:

- The rigid-body ID
- The client and server IP addresses
- The NatNet connection
- `locSrvInit()`
- `crtp_localization_service.c`
- The external-position sending thread