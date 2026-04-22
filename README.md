# 1-DOF Thrust Vector Control (TVC) Demonstration 

## Overview
This repository contains control software for a 1-degree-of-freedom (1-DOF) thrust vector control (TVC) demonstration system, originally developed for AE 4610.

The system demonstrates how a mass-balanced rocket model, pinned at its center of gravity, can stabilize to a commanded pitch angle using a PID controller designed in Simulink.

The platform allows the user to:
- Command a desired pitch angle
- Observe closed-loop stabilization behavior
- Apply external perturbations and observe recovery

## Materials
- Arduino Nano 33 IoT
- Adafruit BNO085 Breakout Board
- 20 kg·cm servo motor
- BLDC motor + propeller
- 30A ESC
- 9V battery
- 12V AC-to-DC power adapter
- B10K potentiometer
- 5V voltage regulator (e.g., LM7805 Voltage Regulator)
- Wiring, connectors, and mounting hardware

## Software Requirements
- Visual Studio Code
- PlatformIO extension (installed in VS Code)
  - PlatformIO automatically installs required compilers and toolchains—no manual C++ installation is required.

## Installation
### Setup
- Install Visual Studio Code
 - Install PlatformIO extension

### Clone Repository
`git clone <repo-url>`

### Open Project
- Open the folder in VS Code
- PlatformIO will automatically configure the environment

## Operation
### Hardware Setup
- Connect Arduino Nano 33 IoT via USB
- Connect IMU, servo, and ESC according to wiring diagram
- Power system:
  - 9V battery (logic power)
  - 12V supply (actuation power)

### Configuration
Set desired pitch angle in code:

`pitch_ref = <desired angle in degrees> * PI / 180;`

### Upload Code
- Build project
- Upload to board via PlatformIO

### IMU Calibration
During upload/reset:
- Hold system steady at 0° pitch
- Allow IMU calibration to complete

### Running the Demonstration 
- System stabilizes at pitch_ref
- Apply manual disturbances
- Observe recovery and stabilization behavior
