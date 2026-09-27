# Balance Bot

A self-balancing two-wheeled robot built from scratch using an Arduino Nano, MPU6050 IMU, and NEMA17 stepper motors. 

This is a personal hobby project created to explore control theory (inverted pendulum), real-time embedded systems, and hardware integration. The firmware is entirely custom-written in C++ using the PlatformIO environment.

## Hardware Components
- **Microcontroller:** Arduino Nano (ATmega328P)
- **Sensors:** MPU6050 (using onboard Digital Motion Processor at 200Hz)
- **Actuators:** 2x NEMA17 Stepper Motors (1/16 microstepping)
- **Motor Drivers:** 2x A4988 

## Key Features
- **High-Speed Control Loop:** Runs at a deterministic 200Hz driven by the MPU6050's DMP interrupt.
- **Custom Stepper Engine:** Timer1 hardware-interrupt based step generation capable of driving motors up to 187 RPM (10,000 steps/s) simultaneously.
- **Robust PID Controller:** Features derivative-on-measurement (to prevent derivative kick), a first-order IIR low-pass filter on the D-term, and integral anti-windup.
- **Auto-Calibration:** Automatically calibrates the balance angle on startup.
- **Fallen State Recovery:** Detects when the robot has fallen, safely cuts power to motors, and cleanly restarts the PID loop when held upright again.

## Documentation
- [Tuning Guide](tuning_guide.md): A comprehensive guide on tuning the PID loop and other control parameters.
- [Project Report](PROJECT_REPORT.md): Detailed technical breakdown of the hardware, software architecture, and control theory behind the robot.

## Build Instructions
This project uses [PlatformIO](https://platformio.org/).
1. Clone the repository.
2. Open the project folder in VS Code with the PlatformIO extension installed.
3. Build and upload to your Arduino Nano.

## Author
Developed as a personal hobby project. Code written independently with mathematical and hardware references from the open-source community.
