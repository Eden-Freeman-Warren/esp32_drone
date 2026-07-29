# esp32_drone
esp32-s3 drone towards autonomos fly

This repository contains the esp32-s3 micro drone firmware. 

**Downloading the Firmware**
Download the ESP-FLY repository from:
https://github.com/Seeed-Projects/Co-Create_ESP-FLY.git
Open the firmware folder in Visual Studio Code.
Install the ESP-IDF extension in Visual Studio Code.
Use ESP-IDF to configure, build, flash, and monitor the drone firmware.
Confirm that the original firmware builds successfully before making modifications.
**Creating the ESP-NOW Drone Transport**
Create a header file for the ESP-NOW transport.
Example filename:
serial_transport.h
Place the header file in:
components/core/crazyflie/hal/interface/
Create a C source file for the ESP-NOW transport.
Example filename:
serial_transport.c
Place the C source file in:
components/core/crazyflie/hal/src/
The transport should initialize Wi-Fi in station mode.
The transport should disable Wi-Fi power saving.
The transport should place the ESP32-S3 on a fixed ESP-NOW channel.
Both the drone and transmitter must use the same ESP-NOW channel.
The transport should initialize ESP-NOW.
The transport should register a receive callback.
The transport should register a send callback if required.
**Updating CMakeLists.txt**
Open the core component’s CMakeLists.txt file.
Add the new C source file to the SRCS section.
Example:
The receive callback should copy incoming packets into a FreeRTOS queue.
The receive callback should remain short and should not block.
The firmware should retrieve incoming packets from the queue.
The firmAdd required ESP-IDF components to REQUIRES.
Examples may include:ware should send outgoing CRTP packets back through ESP-NOW.
Example: "./hal/src/serial_transport.c
Add required ESP-IDF components to REQUIRES.
Examples may include: esp_wifi, esp_event, esp_netif, nvs_flash
Only add components that are actually used by the new source file.
Run a full clean after changing the source list
**Connecting the Transport to the Crazyflie Stack**
Open the existing communication file used by the Crazyflie firmware.
In this project, the communication layer used wifilink.c.
Replace calls to the original Wi-Fi sending function with the new ESP-NOW transport function.
Replace: wifiSendData(...) with serialSendData(...)
change anything dealing a UDP packect into serial
**ESP-NOW Packet Format**
The computer and XIAO communicate using USB serial.
The serial packet begins with a synchronization byte.
The synchronization byte is: 0xAA
The serial packet format is: 0xAA | length | CRTP data | checksum
The checksum is calculated by adding all CRTP bytes.
The checksum is limited to one byte.
The length includes the checksum byte.
Standard CRTP packets contain a maximum of 31 bytes.
The drone-bound packet may contain up to 32 bytes when the checksum is included.
The XIAO should validate the checksum before forwarding a packet.
The drone-side receiver should also validate the expected packet format.
Both directions must use matching framing rules.
**Programming the XIAO Transmitter**
Use a Seeed XIAO ESP32-S3 as the transmitter.
Program the XIAO through the Arduino IDE.
Install the ESP32 board package in Arduino.
Select the correct XIAO ESP32-S3 board.
The XIAO should connect to the computer through USB serial.
The XIAO should read binary CRTP packets from the serial port.
The XIAO should validate each serial packet.
The XIAO should send valid packets to the drone using ESP-NOW.
The XIAO should receive ESP-NOW replies from the drone.
The XIAO should frame the replies and send them to the computer through serial.
Do not use Serial.print() while the binary communication system is active.
Printed text can corrupt the binary CRTP stream.
Use queues to prevent the ESP-NOW callback from blocking.
The transmitter and drone must use the same channel.
The transmitter must use the drone’s correct Wi-Fi station MAC address.
**Python ESP-NOW Driver**
Create a Python cflib link driver.
Example filename:
espnow_driver.py
The driver should recognize a URI such as:espnow://COM9/115200
The driver should open the correct COM port.
The driver should convert cflib CRTP packets into the serial packet format.
The driver should send packets to the XIAO.
The driver should continuously read replies from the XIAO.
The driver should validate synchronization bytes, packet lengths, and checksums.
The driver should convert received data back into cflib CRTP packets.
The driver should register itself with cflib before the connection is opened.
**Testing the Communication Link**
Run the standalone ESP-NOW link test.
**Enabling External Position Packets**
The Crazyflie localization service must be enabled in the firmware.
Open the firmware communication initialization file.
Find the localization initialization call.
Enable: locSrvInit();
Make sure crtp_localization_service.c is included in the build.
The original localization service may contain dependencies for unsupported systems.
Optional LPS, lighthouse, or peer-localization code may cause linker errors.
A simplified localization service can be used if only external position is needed.
The localization service should receive external position packets.
The localization service should send the position measurement to the onboard estimator.
**Python External-Position Program**
Use the NatNet client to receive rigid-body position data.
Configure the correct client IP address.
Configure the correct motion-capture server IP address.
Configure the correct rigid-body ID.
Use:cf.extpos.send_extpos(x, y, z)
Wait until enough external-position packets have been sent before resetting the estimator.
HOME Position
Keep the drone still while HOME is captured.
Average several position samples during HOME capture.
Save the average position as the HOME position.
Subtract HOME from every future position measurement.
This causes the estimator to begin near:X = 0, Y = 0, Z = 0
