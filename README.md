# Color Ball Sorting Robot

A Raspberry Pi-based robot that automatically detects and sorts colored balls using color detection and servo positioning.

## Features

- Continuous ball feeding using threaded stepper motor control
- RGB color detection and classification
- Automated servo positioning for ball sorting
- Configurable sorting positions for different colors
- Returns to rest position after each sort

## Hardware Components

- Raspberry Pi
- RGB Color Sensor (detects Red, Green, Blue values)
- Stepper Motor (continuous ball feeding)
- Servo Motor (sorting mechanism)
- Ball feeding mechanism

## Technical Details

### Color Detection
- Reads RGB values from color sensor
- Classifies colors based on RGB intensity ratios
- Currently detects: Red, Green, Blue, Yellow

### Motor Control
- Stepper motor runs in separate thread for continuous operation
- Servo motor moves to specific angles based on detected color
- Servo returns to rest position after each sort

### Operating Sequence
1. Stepper motor continuously feeds balls
2. Color sensor detects RGB values
3. Program classifies the color
4. Servo moves to corresponding position for sorting
5. Servo returns to rest position
6. Process repeats

## Usage

```bash
python colorSensor.py