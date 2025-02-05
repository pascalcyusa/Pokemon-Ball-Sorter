# Color Ball Sorting Robot

This project implements an automated ball sorting system using a Raspberry Pi that can detect and sort colored balls into different containers based on their colors.

## Features

- Color detection using TCS3200 color sensor
- Stepper motor control for ball feeding mechanism
- Servo motor control for sorting mechanism
- Supports sorting 4 different colors:
  - Red (0° servo position)
  - Green (45° servo position)
  - Blue (90° servo position)
  - Yellow (135° servo position)

## Hardware Requirements

- Raspberry Pi
- TCS3200 Color Sensor
- Stepper Motor with L298N driver
- Servo Motor
- GPIO connections:
  - Color Sensor: pins 13, 15, 16, 18, 22
  - Stepper Motor: pins 37, 36, 31, 29
  - Servo Motor: pin 11

## How It Works

1. The stepper motor continuously moves at a slow pace to feed balls
2. When a ball reaches the color sensor, it detects the color
3. Based on the detected color, the servo motor moves to the corresponding angle:
   - Red → 0°
   - Blue → 45°
   - Green → 90°
   - Yellow → 135°
4. The ball falls through the correct slot for collection

## Usage

Run the main script:
```python
python colorSensor.py