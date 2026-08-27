# ESP32-S3 Bike Computer

A compact GPS-enabled bike computer built around an **ESP32-S3**, **1.28" round GC9A01 TFT display**, **NEO-6M GPS**, and **BME280 environmental sensor**.

The project is designed to provide a motorcycle/bicycle-style dashboard with a speedometer, GPS information, trip data, compass, clock, weather information, barometric pressure and system information.

---

## Features

### Main Dashboard

The main screen provides the primary riding information at a glance.

- GPS-derived speed in km/h
- Large central speed display
- Circular speedometer gauge
- Automatic gauge range
- GPS satellite indicator
- Distance/trip information
- Temperature display
- Digital clock
- High-speed warning colours

### Automatic Speedometer Range

The speedometer has two operating ranges.

#### Normal Mode

Maximum speed:

**40 km/h**

Gauge markings:

**2 km/h**

Numbered markings:

**10 km/h**

#### High-Speed Mode

When speed exceeds approximately 35 km/h, the gauge automatically changes to:

**120 km/h**

Gauge markings:

**5 km/h**

Numbered markings:

**20 km/h**

This prevents the display from becoming difficult to read at higher speeds.

### Speed Warning Colours

The speed indicator changes colour according to speed.

Normal range:

- Cyan — normal
- Yellow — approximately 25 km/h and above
- Red — approximately 35 km/h and above

High-speed range:

- Cyan — normal
- Yellow — approximately 70 km/h and above
- Red — approximately 100 km/h and above

---

# Display Pages

The project uses two physical buttons to navigate through the different pages.

## Button 1

Button 1 cycles through:

1. Main
2. Trip
3. GPS
4. Compass
5. Clock

## Button 2

Button 2 cycles through:

1. Clock
2. Weather
3. Barometer
4. System

The button system is designed so that both buttons can be used independently to access different information.

---

## Main Page

The main page is designed as the primary riding screen.

It contains:

- Circular speed gauge
- Large speed readout
- `KM/H` indicator
- GPS status
- Satellite information
- Distance
- Temperature
- Clock

The display is designed to resemble a small motorcycle/bike dashboard rather than a conventional rectangular GPS display.

---

## Trip Page

The trip page provides information relating to the current journey.

The GPS is used to calculate travelled distance.

Distance is calculated using GPS coordinates and the TinyGPSPlus distance calculation functions.

---

## GPS Page

The GPS information page displays GPS-related information including:

- GPS fix status
- Number of satellites
- HDOP
- GPS information
- Current GPS-derived speed

The GPS indicator changes according to GPS status/quality.

### GPS Status

The dashboard uses different indicator states to provide a quick visual indication of GPS reception.

---

## Compass Page

The compass page is reserved for directional/navigation information.

The display layout is designed so additional compass functionality can be incorporated as the project develops.

---

## Clock Page

The clock uses GPS date and time information.

The project includes Irish Summer Time handling so the displayed time can be adjusted appropriately for Irish daylight-saving time.

GPS time is used rather than relying solely on the ESP32 internal clock.

---

# Weather Page

A **BME280** sensor provides environmental information.

The weather page displays:

- Temperature
- Humidity
- Atmospheric pressure
- Trend information

Measurements are updated periodically rather than continuously to provide stable readings.

The current configuration updates the environmental readings every:

**10 seconds**

---

## Weather Trends

The project monitors changes in:

### Temperature

Trend threshold:

**0.2°C**

### Pressure

Trend threshold:

**1.0 hPa**

### Humidity

Trend threshold:

**2%**

The display can indicate whether a value is:

- Rising
- Falling
- Remaining approximately stable

The trend is calculated from previous measurements rather than simply showing the current value.

---

# Barometer Page

The BME280 atmospheric pressure sensor is also used as a barometer.

Current gauge range:

**980–1040 hPa**

The barometer uses a circular gauge similar to the main speedometer.

This provides a quick indication of changes in atmospheric pressure while riding.

---

# System Page

The system page provides information about the ESP32-S3 and the current operating state.

It is intended as a diagnostic/information page and can be expanded with additional system information as the project develops.

---

# Light Theme

A light display theme has been added to the project.

The theme can be activated using a long press of **Button 2**.

This allows the dashboard to be switched between the normal dark display and a lighter display suitable for different lighting conditions.

---

# Touch / Capacitive Input

The ESP32-S3 provides capacitive touch-capable GPIOs.

The project has also been experimenting with using:

- GPIO 2
- GPIO 3

as capacitive touch inputs.

The intention is to allow touch activation of the same functions as the physical buttons while retaining the physical button functionality.

Touch sensitivity is based on the ESP32-S3 capacitive touch readings.

---

# Hardware

## Required Parts

### 1. ESP32-S3-N16R8

An ESP32-S3 development board is the main controller.

Recommended configuration:

- ESP32-S3
- USB programming connection
- 3.3V GPIO
- Sufficient flash/RAM for the project

The project has been developed using:

**ESP32-S3-N16R8**

---

### 2. 1.28" Round TFT Display

Display:

**1.28" round TFT**

Controller:

**GC9A01**

Resolution:

**240 × 240 pixels**

The display uses SPI.

---

### 3. GPS Module

GPS:

**GY-GPS6MV2**

The module provides:

- Latitude
- Longitude
- Speed
- Date
- Time
- Satellites
- HDOP
- GPS fix

The GPS is connected to one of the ESP32-S3 hardware UARTs.

---

### 4. BME280

Environmental sensor:

**BME280**

Provides:

- Temperature
- Humidity
- Atmospheric pressure

The sensor uses I²C.

---

### 5. Push Buttons

Two physical push buttons are used for dashboard navigation.

Additional capacitive touch inputs are being developed using the ESP32-S3 touch-capable GPIOs.

---

# Pinout

## TFT Display

| Display | ESP32-S3 |
|---|---:|
| SCLK / SCL | GPIO 12 |
| MOSI / SDA | GPIO 11 |
| MISO | Not used |
| DC | GPIO 9 |
| CS | GPIO 10 |
| RST | GPIO 8 |
| BL | GPIO 7 |
| VCC | 3.3V |
| GND | GND |

The display is configured for SPI using the ESP32-S3 SPI2 interface.

---

# GPS Pinout

| GPS | ESP32-S3 |
|---|---:|
| VCC | 3.3V |
| GND | GND |
| TX | GPIO 17 |
| RX | GPIO 18 |

The GPS uses:

**HardwareSerial(1)**

The ESP32-S3 receives GPS data through the GPS UART.

### Important

GPS module TX connects to ESP32 RX.

GPS module RX connects to ESP32 TX.

Therefore:

```text
GPS TX → ESP32 GPIO 17
GPS RX → ESP32 GPIO 18
