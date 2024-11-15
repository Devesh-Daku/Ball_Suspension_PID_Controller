# Ball Suspension by PID Controller on Air Thrust

## Project Overview
This project suspends a ball in a tube using a PID-controlled thrust system to maintain height stability through air thrust adjustments.

## Features
- Adjustable height control via potentiometeSr
- Real-time feedback with ultrasonic sensor
- LCD display showing target and current heights

## Components
- **Arduino Mega**
- **ESC BelHeli 30A**
- **BLDC Motor**
- **Ultrasonic Sensor**
- **LCD Display (16x2)**
- **Potentiometer**

## Setup
1. **Hardware Assembly**: Connect Arduino, ESC, motor, LCD, ultrasonic sensor, and potentiometer.
2. **Control System**: Tune PID values as per requirements (suggested Kp, Ki, Kd provided).
3. **3D Printed Parts**: Attach printed components to form the tube and motor mounts.

## Code
The code (included in this repo) initializes the components, reads sensor data, computes PID adjustments, and displays results on LCD.

## Usage
- Adjust potentiometer to set desired object height.
- Monitor the LCD display for real-time height feedback.

## Future Development
- Test PID constants for a range of object weights.
- 3d Desing of PropAreaFile can be changed ( Elongated for less loss of air ).


