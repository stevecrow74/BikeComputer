# ESP32-S3 Bike Computer

Firmware for a GPS bike computer based on an ESP32-S3, a 240 x 240 GC9A01 round display, a GPS receiver, and a BME280 environmental sensor. The application is built with PlatformIO and the Arduino framework.

## Features

- GPS speedometer with automatic 40 km/h and 120 km/h gauge ranges.
- GPS trip distance, maximum speed, sampled average speed, and persistent lifetime odometer.
- Detailed GPS status: fix state, satellites, HDOP, location, altitude, speed, and time.
- GPS-course compass while moving at 1 km/h or more.
- GPS-synchronised clock with Irish Summer Time adjustment and temperature readout.
- Weather display for BME280 temperature, pressure, humidity, calculated barometric altitude, and trends.
- Analog barometer from 980 to 1040 hPa.
- Speed and GPS altitude history graphs covering the latest 10 minutes.
- Dark and light display themes.
- Physical buttons and capacitive touch-pad navigation.

A three-mode display theme

Dark -> Light -> Night -> Dark

Hold physical Button 2 for three seconds to change mode. Night mode uses a black background with subdued red primary text, darker red secondary text/lines, and red gauge rings to reduce glare while riding at night. The selected mode is also printed to the serial monitor.

## Pages

### Button 1 / Touch GPIO 2

Short presses cycle through Main speedometer, Trip, GPS, Compass, Clock, and History. The next press from History returns to the main speedometer.

Hold the physical Button 1 for 3 seconds to reset the current trip. It resets trip distance, maximum speed, and average-speed totals. The lifetime odometer is not reset.

### Button 2 / Touch GPIO 3

Short presses cycle through Clock, Weather, Barometer, and System.

Hold the physical Button 2 for 3 seconds to switch between dark and light themes.

## Main Speedometer

The main page displays current GPS speed, a colour-coded gauge, GPS quality and satellite count, current trip distance, calibrated BME280 temperature, a GPS-based altitude direction arrow, and GPS-derived local time.

The altitude arrow compares valid GPS altitude samples 10 seconds apart. It indicates rising or falling only when the change exceeds 2 m; otherwise it shows level.

## Trip And Odometer

Trip distance is calculated from accepted GPS position changes between 0.5 m and 100 m. The Trip page shows distance, current speed, maximum speed, average speed, satellite count, and `ODO`.

Trip measurements and the odometer are stored in ESP32 NVS every 30 seconds and restored after reset or power cycling. The odometer increases from the same validated GPS distance segments as trip distance and survives a Button 1 trip reset. It currently displays whole kilometers.

## GPS And Compass

GPS data uses `HardwareSerial(1)` at 9600 baud. The GPS page provides fix status, satellites, latitude, longitude, altitude, HDOP, time, and speed.

The compass rotates from GPS course, so it needs a valid GPS fix and motion of at least 1 km/h. It is not a magnetic compass and cannot provide a reliable heading while stationary.

GPS time is converted to Irish local time, including daylight-saving transitions.

## Weather And Altitude

The BME280 is sampled every 10 seconds. The Weather page shows temperature, pressure, humidity, and pressure-derived altitude. Temperature is adjusted by `TEMP_CALIBRATION_OFFSET` to compensate for enclosure and display heating.

At boot, barometric altitude uses the fallback `seaLevelPressureHpa` value of 1020.0 hPa. Once a strong GPS signal is acquired, the firmware recalibrates this reference exactly once using current GPS altitude and measured BME280 pressure. Calibration requires valid GPS location and altitude, valid HDOP of 1.5 or lower, and an available BME280 pressure reading.

The calibrated reference remains fixed until the next restart. It is printed to the serial monitor as `Sea-level pressure calibrated: ...`.

## History Graphs

The History page retains 60 samples in RAM, captured every 10 seconds. It shows separate graphs for speed and valid GPS altitude, covering the latest 10 minutes. The newest point is at the right of each graph.

History is not saved to flash and clears after a restart.

## Hardware And Pinout

### GC9A01 Display

| Signal | ESP32-S3 pin |
| --- | --- |
| SCLK | GPIO 12 |
| MOSI | GPIO 11 |
| DC | GPIO 9 |
| CS | GPIO 10 |
| RST | GPIO 8 |
| Backlight | GPIO 7 |

The display uses SPI2 and is configured for 240 x 240 pixels.

### GPS Receiver

| GPS signal | ESP32-S3 pin |
| --- | --- |
| GPS TX | GPIO 18 (ESP32 RX) |
| GPS RX | GPIO 17 (ESP32 TX) |
| VCC | 3.3V |
| GND | GND |

Connect TX to RX and RX to TX as shown.

### BME280

| BME280 signal | ESP32-S3 pin |
| --- | --- |
| SDA | GPIO 5 |
| SCL | GPIO 4 |
| VCC | 3.3V |
| GND | GND |

The firmware attempts BME280 I2C addresses `0x76` and `0x77`.

### Controls

| Control | ESP32-S3 pin |
| --- | --- |
| Physical Button 1 | GPIO 1 |
| Physical Button 2 | GPIO 20 |
| Touch pad 1 | GPIO 13 |
| Touch pad 2 | GPIO 14 |

The physical buttons use `INPUT_PULLUP` and should pull their GPIO low when pressed.

Touch pads are calibrated at startup. Keep both pads untouched during startup while the firmware averages 40 readings for each baseline. A deviation of 5000 counts triggers a touch.

## Build And Upload

Install PlatformIO, connect the ESP32-S3, and run from the project directory:

```sh
platformio run
platformio run --target upload
platformio device monitor --baud 115200
```

The PlatformIO environment is `esp32-s3-devkitc-1`. Dependencies are LovyanGFX, TinyGPSPlus, Adafruit Unified Sensor, and Adafruit BME280 Library.
