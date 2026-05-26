# Automated Car Production Line Using Dual Robotic Arms and Conveyor Belt

## Overview
This project implements an automated miniature car production line using two 4-DOF robotic arms, a conveyor belt system, ultrasonic sensing, and dual Arduino Uno microcontrollers.

The system demonstrates industrial robotics concepts including:
- Pick-and-place automation
- Conveyor belt coordination
- Forward and inverse kinematics
- Inter-Arduino communication
- Smooth servo interpolation

The production line performs a complete automated sequence where objects are transported using a conveyor belt and transferred between two robotic arms.

---

## Features
- Dual robotic arm coordination
- Conveyor belt automation
- Ultrasonic object detection
- Inverse Kinematics (IK) implementation
- Smooth servo interpolation
- Inter-Arduino digital communication
- Automated pick-and-place operations
- Real-time servo calibration system

---

## System Architecture
- **Arduino 1**
  - Controls ARM 1
  - Controls conveyor belt motor
  - Detects objects using HC-SR04 sensor

- **Arduino 2**
  - Controls ARM 2
  - Detects objects at second station
  - Sends stop signal to Arduino 1

---

## Hardware Components
- Arduino Uno ×2
- MG996R Servo Motors ×8
- PCA9685 PWM Servo Drivers
- HC-SR04 Ultrasonic Sensors ×2
- L298N Motor Driver
- DC Motor
- Conveyor Belt
- External Power Supply

---

## Software Technologies
- Arduino C++
- Wire.h Library
- Adafruit_PWMServoDriver Library
- Forward Kinematics (FK)
- Inverse Kinematics (IK)

---

## Operational Sequence
1. Conveyor belt starts moving.
2. ARM 1 detects object using ultrasonic sensor.
3. Conveyor stops.
4. ARM 1 performs pick-and-place operation.
5. Conveyor restarts.
6. ARM 2 detects object at second station.
7. Arduino 2 signals Arduino 1 to stop belt.
8. ARM 2 performs final pick-and-place operation.
9. System halts after successful transfer.

---

## Inverse Kinematics
The project uses a geometric 2-DOF inverse kinematics solver to calculate shoulder and elbow angles automatically from Cartesian coordinates.

This significantly improved:
- Calibration speed
- Position accuracy
- Reconfiguration flexibility

---

## Smooth Servo Interpolation
Servo movements are executed gradually instead of instant jumps to:
- Reduce mechanical stress
- Improve positioning accuracy
- Prevent servo damage
- Create smoother motion

---

## Files Included
- `arm1.ino` → Arduino 1 firmware (ARM 1 + Conveyor Belt Control)
- `arm2.ino` → Arduino 2 firmware (ARM 2 Control)
- `Production_Line_Report_Final2.docx` → Full project report

---

## Challenges Solved
- H-Bridge overvoltage failure
- Conveyor belt fabrication and alignment
- Servo overheating
- Servo angle calibration
- FK to IK transition
- Stable power delivery

---

## Results
The system successfully achieved:
- Reliable object detection
- Smooth robotic arm motion
- Accurate pick-and-place operations
- Stable inter-Arduino communication
- Fully automated production cycle

---

## Future Improvements
- Multi-cycle continuous operation
- Vision-based object detection
- Full 3D inverse kinematics
- Advanced path planning
- Improved gripper cooling

---

## Authors
Alaa Waheed


---

## References
- Arduino Documentation
- Adafruit PCA9685 Documentation
- Robotics: Modelling, Planning and Control
- Introduction to Robotics: Mechanics and Control
