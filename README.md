# CO2 Traffic Light
Traffic light which warns about rising CO2 levels in the air to motivate venting. CO2 seems to be a good indicator of aerosol levels in the air, so keeping it at a low level should also decrease the chance of Corona spreading.

## Hardware
* ESP32
* MH-Z19B (CO2 Sensor)
* LEDs
* Something that looks like a traffic light (or is a traffic light)

## Wiring
Connect the MH-Z19B to RX2 und TX2 of the ESP32, connecting TX from the sensor to RX2 of the ESP (and RX to TX2). LEDs should be connected to 25 (green), 26 (yellow) and 27 (red).