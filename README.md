# ESP32 BLE Scanner

- [ESP32 BLE Scanner](#esp32-ble-scanner)
  - [Hardware \& Platform](#hardware--platform)
  - [Introduction](#introduction)


## Hardware & Platform
- ESP32-S3 with ESP-IDF v5.1.1


## Introduction
- This is a simple ESP32 BLE scanner application using ESP-IDF v5.1.1
- The device will only scan for GATT server on the while-list, after that it will forward advertisement data to the MQTT broker