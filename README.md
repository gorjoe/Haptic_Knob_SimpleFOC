# Haptic_Knob_SimpleFOC
This repo is trying to replicate the [smartknob](https://github.com/scottbez1/smartknob) project with minimum and on hand hardware.

Credit to [scottbez1](https://github.com/scottbez1) inspire this repo

## Compontents:
2804 BLDC motor

ESP32 S3

SimpleFOCmini

GC9A01 round display

---

## Wiring table:
| Component Pin | ESP32-S3 Pin | Notes / Function                         |
|---------------|--------------|------------------------------------------|
| AS5600 SDA    | GPIO 8       | I2C Data                                 |
| AS5600 SCL    | GPIO 9       | I2C Clock                                |
| AS5600 VCC    | 3.3V         | Power (Ensure AS5600 jumper set to 3.3V) |
| AS5600 GND    | GND          | Sensor Ground                            |
| AS5600 DIR    | GND          | Sets direction (GND for clockwise count) |
| Mini IN1      | GPIO 1       | PWM High Phase A                         |
| Mini IN2      | GPIO 2       | PWM High Phase B                         |
| Mini IN3      | GPIO 4       | PWM High Phase C                         |
| Mini EN       | GPIO 5       | Driver Enable Pin                        |
| Mini GND      | GND          | Crucial: Must connect to ESP32-S3 GND    |

## Round display Wiring table:
| GC9A01 Pin | ESP32-S3 Pin | Function                   |
|------------|--------------|----------------------------|
| VCC        | 3.3V         | Power Supply               |
| GND        | GND          | Ground                     |
| SCL        | GPIO 13      | SPI Clock (SCK)            |
| SDA        | GPIO 14      | SPI Data (MOSI)            |
| RES        | GPIO 12      | Reset                      |
| DC         | GPIO 11      | Data/Command Select        |
| CS         | GPIO 10      | Chip Select                |
| BLK        | 3.3V         | Backlight Power (Optional) |
