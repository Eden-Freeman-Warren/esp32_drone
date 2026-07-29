# esp32_drone
esp32-s3 drone towards autonomos fly

This repository contains the esp32-s3 micro drone firmware. 

***Downloading the Firmware***
Download the ESP-FLY repository from:
https://github.com/Seeed-Projects/Co-Create_ESP-FLY.git
Open the firmware folder in Visual Studio Code.
Install the ESP-IDF extension in Visual Studio Code.
Use ESP-IDF to configure, build, flash, and monitor the drone firmware.
Confirm that the original firmware builds successfully before making modifications.
***Creating the ESP-NOW Drone Transport***
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
***Updating CMakeLists.txt***
Open the core component’s CMakeLists.txt file.
Add the new C source file to the SRCS section.
Example:
The receive callback should copy incoming packets into a FreeRTOS queue.
The receive callback should remain short and should not block.
The firmware should retrieve incoming packets from the queue.
The firmware should send outgoing CRTP packets back through ESP-NOW.
