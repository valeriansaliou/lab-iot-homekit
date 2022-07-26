lab-iot-homekit
===============

Some HomeKit-based IoT experiments. An ESP32 board is required to run the code.

# Requirements

All projects have the following common requirements:

 * Arduino IDE
 * An ESP32 board (mine is: `ESP32-WROOM-32`)

# Projects

## Sprinkler Tank Water Level

The goal of this project is to measure the water level in a tank used for plant watering, using an ultrasonic sensor. The water level is periodically reported via a battery service to HomeKit (as there is no water level service, as of 2022).

This project requires an HC-SR04 ultrasonic sensor to be attached to the ESP32 board, with the following pin connections:

- Sensor `TRIG` to ESP32 `PIN 22`
- Sensor `ECHO` to ESP32 `PIN 21`
- Sensor `GND` to ESP32 `GND`
- Sensor `VCC` to ESP32 `VCC`

The following resistance values are used to obtain the `ECHO` value:

- `R1` of 330 ohms
- `R2` of 470 ohms

The custom board that should be built follows the same schematics [as described here](https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/).

The CAD files for the sensor casing parts are also provided in this project. They should be 3D printed on a SLA printer (mine is: Formlabs Form 3).

<p>
  <img src="https://user-images.githubusercontent.com/1451907/180968484-fc47840e-9791-4bb7-8c44-35c769fafaab.png" width="240" alt="Water level sensor in the Home app" />
  <img src="https://user-images.githubusercontent.com/1451907/180968453-dfd05101-4b7c-45a8-acbc-9d36aaf2abe1.png" width="640" alt="Periodic water level reporting" />
<p>
