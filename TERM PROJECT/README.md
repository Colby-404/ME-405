# ME 405 Term Project

## Project Overview

The ME 405 term project centers on the design, assembly, programming, and testing of an autonomous **Pololu Romi** mobile robot capable of reliably navigating a printed line track during an in-class **time-trial demonstration**. The project combines mechanical assembly, electrical integration, embedded programming, sensor interfacing, and closed-loop control into a complete mechatronic system.

The final robot is built around an **STM32 Nucleo-L476RG** microcontroller mounted on a **Shoe of Brian** interface board and installed on a Romi chassis with motors, encoders, and onboard battery power. To complete the project, the robot must sense the line, estimate its motion, respond to changing track conditions, and drive the course as quickly and consistently as possible.

---

## Project Goals

The main goals of the term project are to:

- build a fully functional Romi-based autonomous robot
- integrate sensors and actuators into a single embedded control system
- implement reliable **line-following behavior**
- use feedback control to regulate wheel motion and steering
- improve robot performance for **speed, repeatability, and robustness**
- document the design process, hardware setup, software structure, and project results in a professional portfolio

---

## System Description

The robot uses a **differential-drive** configuration with two independently driven wheels and wheel encoders for motion feedback. A **QTR reflectance sensor array** is used to detect the position of the printed line beneath the robot, allowing the control system to compute line error and steer back toward the center of the track. An **IMU** can be used to provide orientation or rotational information, and bumper switches may be incorporated for obstacle/contact detection depending on the final design.

The overall system combines:

- **mechanical design** of the Romi hardware stack
- **electrical integration** of motors, encoders, sensors, and power wiring
- **driver development** for the hardware devices
- **task-based embedded software** running in MicroPython
- **closed-loop motor and steering control**
- **testing and tuning** for reliable time-trial performance

---

## Control Approach

The robot’s control system is based on feedback from its sensors.

At a high level, the robot:

1. reads the reflectance sensors to determine line position
2. computes a line-following error relative to the center of the robot
3. adjusts wheel commands to steer toward the line
4. uses motor/encoder feedback to regulate wheel speed
5. repeats this process continuously while driving the track

This creates a closed-loop system in which the robot can correct its path in real time rather than simply following a fixed command sequence.

---

## Major Hardware Used

The term project hardware typically includes:

- **Pololu Romi chassis**
- **STM32 Nucleo-L476RG**
- **Shoe of Brian** interface board
- **DC motors and quadrature encoders**
- **QTR-MD-08A reflectance sensor array**
- **BNO055/BNO085 IMU** depending on project configuration
- **bumper switches** for contact sensing
- custom power, motor, encoder, and sensor wiring

---

## Major Software Features

The term project software includes:

- motor driver control
- encoder feedback processing
- reflectance sensor reading and calibration
- line position estimation
- closed-loop speed control
- closed-loop steering / line-following control
- multitasking or cooperative task scheduling
- serial user interface for testing and tuning
- optional state estimation and IMU integration

---

## Design Challenges

This project requires balancing several competing design goals:

- **speed vs. stability**
- **aggressive steering vs. smooth tracking**
- **sensor sensitivity vs. noise**
- **fast lap times vs. repeatable performance**

In practice, a successful design depends not only on making the robot fast, but also on making it reliable. Small issues in wiring, calibration, sensor placement, control gains, or task timing can significantly affect performance on the track.

---

## Final Deliverables

The term project concludes with two major deliverables:

1. an **in-class robot demonstration and time-trial**, where the team presents the design and runs the robot on the printed course
2. a **project portfolio/repository**, containing the source code, hardware documentation, design files, and a written explanation of the project, its challenges, and its results

---

## Folder Contents

This folder contains the files directly related to the final Romi project, including:

- project source code
- hardware setup and pinout references
- sensor and control task files
- tuning and testing utilities
- documentation and supporting write-ups
- final materials used for the portfolio and submission

---

## Summary

This term project brings together the major topics of ME 405 into one complete robotic system. The final result is an autonomous Romi robot that uses embedded hardware and software, sensor feedback, and closed-loop control to follow a track and compete in a time-trial event. The project demonstrates the integration of mechatronics concepts including system assembly, driver development, multitasking control, sensing, estimation, and real-world performance tuning.
