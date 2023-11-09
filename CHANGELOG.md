## V0.0.1 - Oct 20, 2023
- Initial release
- Finish driver for Wi-Fi, BLE GATT Client, MQTT Client
- Finish BLE scanner application which will scan for BLE GATT server on the while-list, after that it will forward BLE data to the MQTT broker

## V0.0.2 - Nov 9, 2023
- The device will first connect and subscribe for indication to the number of "BLE_NUMBER_TARGET_DEVICE". Only after that it will send all the BLE data receive via indication to the MQTT broker every "GATT_DEVICE_MANAGER_SEND_DATA_PERIOD"

## V0.0.3 - Nov 10, 2023
- Do not prepare and send data to MQTT when Wi-Fi is not connected