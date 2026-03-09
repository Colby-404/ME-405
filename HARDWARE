# Hardware Description

This document describes the hardware components used in the Romi robot platform for this project. The system consists of a line-following sensor array, bumper switches for collision detection, and an inertial measurement unit (IMU) for orientation sensing.

---

# System Overview

The robot uses a **Romi chassis with an STM32 Nucleo microcontroller** to control motors and read sensor data. Several external sensors are used to provide environmental feedback and state estimation.

The main hardware components include:

- Reflectance sensor array for **line detection**
- Bumper switches for **collision detection**
- IMU for **orientation and motion sensing**

---

# Line Following Sensor

## QTR-MD-08A Reflectance Sensor Array

Product link:  
https://www.pololu.com/product/4248

The **QTR-MD-08A** is an 8-channel infrared reflectance sensor array used to detect lines on the ground.

### Function

Each sensor consists of an **IR LED and phototransistor pair**. The IR light reflects off the surface below the robot:

- **White surface → high reflection → higher analog output**
- **Black line → low reflection → lower analog output**

By reading all eight sensors, the robot can determine the **position of the line relative to the center of the robot**, allowing closed-loop line following control.

### Key Features

- 8 analog reflectance sensors
- 8 mm spacing between sensors
- Analog voltage output
- Adjustable IR LED brightness
- High sampling rate suitable for real-time control

### Typical Use

The sensor array is mounted on the **front underside of the robot**, allowing it to detect the line before the robot passes over it.

The eight analog outputs are connected to ADC-capable pins on the microcontroller.

---

# Collision Detection

## Romi Bumper Switch Kit

Product:  
Bumper Switch Kit for Romi / TI-RSLK MAX

These switches detect when the robot physically contacts an obstacle.

### Function

Each bumper switch acts as a **digital input** to the microcontroller.

When the robot collides with an object:

- The bumper arm presses the switch
- The switch closes
- The microcontroller reads a **logic LOW or HIGH**, depending on wiring

The robot software can then trigger a behavior such as:

- stopping the motors
- backing up
- turning away from the obstacle

### Configuration

Two bumper switches are used:

| Switch | Location |
|------|------|
| Left bumper | Left front side of robot |
| Right bumper | Right front side of robot |

---

# Inertial Measurement Unit

## BNO085 IMU

The **BNO085** is a 9-axis inertial measurement unit that provides orientation and motion sensing.

### Sensors Included

The IMU integrates multiple sensors:

- 3-axis accelerometer
- 3-axis gyroscope
- 3-axis magnetometer

These sensors allow the system to measure:

- angular velocity
- acceleration
- orientation
- heading

### Sensor Fusion

The BNO085 includes an onboard processor that performs **sensor fusion**, combining raw sensor data to produce stable orientation estimates.

Outputs may include:

- Euler angles
- quaternions
- angular velocity
- linear acceleration

### Communication

The IMU communicates with the microcontroller using **I²C**.

Typical connections include:

| IMU Pin | MCU Pin | Function |
|------|------|------|
| SCL | PB13 | I²C Clock |
| SDA | PB14 | I²C Data |
| VCC | 3.3V or 5V | Power |
| GND | GND | Ground |

---

# System Role of Each Sensor

| Sensor | Purpose |
|------|------|
| QTR Reflectance Array | Detect line position for line-following control |
| Bumper Switches | Detect collisions with obstacles |
| BNO085 IMU | Measure robot orientation and motion |

---

# Summary

This hardware configuration enables the robot to perceive its environment through multiple sensing modalities. The reflectance sensor array provides ground-based feedback for line tracking, the bumper switches provide physical collision detection, and the IMU provides orientation data used for motion estimation and advanced control strategies.
