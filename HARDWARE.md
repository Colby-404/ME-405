# Hardware Description

This document describes the hardware components used in the Romi robot platform for this project. The system consists of a Romi mobile robot chassis controlled by an STM32 Nucleo microcontroller and equipped with several sensors for navigation and environment interaction.

---

# System Overview

The robot platform is based on the **Pololu Romi chassis** and is controlled using an **STM32 Nucleo-L476RG** microcontroller mounted on a **Shoe of Brian interface board**. The system includes sensors for line detection, collision detection, and orientation measurement.

Primary hardware components include:

- Romi chassis with motors and encoders
- STM32 Nucleo-L476RG microcontroller
- Shoe of Brian interface board
- QTR reflectance sensor array
- Bumper switches
- BNO085 IMU

---

# Romi Robot Platform

The Romi robot serves as the base mobile platform for the system.

### Main Components

- Romi chassis
- Two DC motors
- Wheel encoders
- Wheels and caster
- Power distribution board

The Romi chassis provides both mechanical support and electrical power distribution to the system.

The motors drive the robot while the **quadrature encoders measure wheel rotation**, allowing the software to determine wheel velocity and displacement.

---

# Microcontroller System

## STM32 Nucleo-L476RG

The Nucleo-L476RG microcontroller is used to control the robot and interface with sensors.

It provides:

- ADC inputs
- digital I/O
- PWM motor control
- hardware timers
- I2C communication for sensors

The board runs **MicroPython firmware** and executes the robot control software.

---

# Shoe of Brian Interface Board

The **Shoe of Brian** board connects the Nucleo to the Romi robot platform.

It provides:

- convenient breakout pins
- power routing from the Romi battery
- connectors for motors and encoders

During assembly, the ferrite bead on the Shoe of Brian must be removed so that the Nucleo receives power from the Romi battery rather than USB power.

The Nucleo and Shoe are stacked together using Morpho headers and mounted to the Romi chassis using standoffs.

The assembly procedure follows the instructions provided in the Romi assembly guide. :contentReference[oaicite:4]{index=4}

---

# Line Detection Sensor

## QTR-MD-08A Reflectance Sensor Array

Product link:  
https://www.pololu.com/product/4248

The QTR-MD-08A is an 8-channel infrared reflectance sensor used for line detection.

### Operation

Each sensor emits infrared light toward the surface below the robot.

- White surfaces reflect more IR light
- Dark surfaces reflect less IR light

The sensor outputs an **analog voltage proportional to the reflected light intensity**.

By reading all eight sensors simultaneously, the system can determine the position of a line relative to the robot.

### Purpose

This sensor array enables **closed-loop line following control**.

---

# Collision Detection

## Romi Bumper Switch Kit

The bumper switches detect collisions with obstacles.

Two switches are mounted on the front of the robot:

| Switch | Location |
|------|------|
| Left bumper | Left front of robot |
| Right bumper | Right front of robot |

When the robot hits an object, the switch closes and the microcontroller reads a digital signal indicating a collision.

This allows the robot to stop or change direction.

---

# Inertial Measurement Unit

## BNO085 IMU

The robot uses a **BNO085 inertial measurement unit** to measure motion and orientation.

### Sensors Included

The IMU contains:

- 3-axis accelerometer
- 3-axis gyroscope
- 3-axis magnetometer

### Sensor Fusion

The onboard processor performs sensor fusion to estimate orientation.

Typical outputs include:

- angular velocity
- acceleration
- orientation angles
- quaternions

### Communication

The IMU communicates with the microcontroller using **I2C**.

Example connections:

| IMU Pin | MCU Pin | Function |
|------|------|------|
| SCL | PB13 | I2C clock |
| SDA | PB14 | I2C data |
| VCC | 3.3V or 5V | Power |
| GND | GND | Ground |

---

# Cabling

The Romi platform requires several cable assemblies between the power distribution board and the Nucleo.

These include:

- motor control cables
- encoder cables
- power cable
- IMU cable
- sensor cables

Each cable follows a specific pinout to ensure correct operation.

Incorrect wiring can damage the Nucleo, so connections should always be verified before powering the system. :contentReference[oaicite:5]{index=5}

---

# Summary

The Romi robot platform combines a mobile chassis, microcontroller system, and multiple sensors to enable autonomous navigation.

The reflectance sensor array provides line position feedback, the bumper switches detect collisions, and the IMU measures orientation and motion. Together these components allow the robot to perceive its environment and perform closed-loop control during operation.
