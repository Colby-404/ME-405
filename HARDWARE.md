# Hardware Description

This document describes the hardware used on the Romi robot platform for this project. The system is built around a **Pololu Romi chassis** controlled by an **STM32 Nucleo-L476RG** microcontroller mounted on a **Shoe of Brian** interface board. The robot uses wheel encoders, a reflectance sensor array, bumper switches, and an IMU for closed-loop motion control, line following, obstacle detection, and orientation measurement.

---

# System Overview

The robot platform consists of the following major hardware components:

- Pololu Romi chassis
- Two DC gearmotors
- Two wheel encoders
- Power distribution board
- STM32 Nucleo-L476RG microcontroller
- Modified Shoe of Brian interface board
- Acrylic Romi-to-Shoe adapter
- QTR-MD-08A reflectance sensor array
- Romi bumper switch modules
- BNO055 IMU breakout board
- Custom jumper-wire cable harnesses
- USB-A to USB-Mini B programming cable

The sensing and control roles are divided as follows:

- **Encoders** provide wheel speed and displacement feedback
- **QTR sensor array** detects line position
- **Bumper switches** detect collisions with obstacles
- **IMU** provides orientation and angular motion data

---

# Romi Robot Platform

The Romi robot serves as the base mobile platform for the system. It provides the mechanical chassis, drivetrain, wheel encoders, and battery-powered electronics needed for autonomous operation.

## Main Components

- Romi chassis
- Two DC motors
- Wheel encoders
- Wheels and rear caster
- Power distribution board
- Integrated motor driver hardware

## Key Romi Parameters

- **Chassis diameter:** 163 mm
- **Track width:** 141 mm
- **Wheel radius:** 35 mm
- **Gear ratio:** approximately 119.76:1
- **Motor rated voltage:** 4.5 V
- **No-load speed:** approximately 150 RPM at 4.5 V
- **Maximum translational speed:** approximately 550 mm/s at 4.5 V
- **Effective wheel encoder resolution:** approximately 1440 pulses per revolution

The Romi uses **differential-drive kinematics**, so it is **nonholonomic**. It can move forward, backward, and pivot in place, but it cannot translate sideways directly.

---

# Motors and Encoders

The robot uses two permanent-magnet DC gearmotors. Each motor includes an encoder mounted before the gearbox. Although the encoder resolution at the motor shaft is low, the gearbox increases the effective wheel-side resolution substantially.

The encoders are used to measure:

- wheel position
- wheel velocity
- robot displacement estimate

These measurements are critical for:

- closed-loop wheel speed control
- odometry
- motion profiling

The motors should not be intentionally stalled or forced backward by hand, since that can damage the drivetrain.

---

# Microcontroller System

## STM32 Nucleo-L476RG

The STM32 Nucleo-L476RG is the main controller for the robot. It runs the robot software and interfaces with the motors, encoders, and sensors.

It provides:

- GPIO pins
- ADC inputs
- timer channels
- PWM outputs
- I2C communication
- USB programming interface

The board runs **MicroPython firmware** and executes the robot control tasks.

---

# Shoe of Brian Interface Board

The **Shoe of Brian** is the interface board between the Nucleo and the Romi hardware. It simplifies wiring and allows the controller stack to mount cleanly to the chassis.

It provides:

- convenient breakout connections
- routing between the Romi power board and Nucleo
- access to motor and encoder connections
- support for sensor wiring

## Required Modification

For safe operation on the Romi platform, the Shoe of Brian must be **modified by removing the ferrite bead** that normally connects USB 5V power to the Nucleo power path.

This modification is required because:

- the Nucleo is powered from the **Romi battery**
- the Romi power board supplies voltage through the hardware stack
- leaving the bead in place can create an unsafe power path

After the ferrite bead is removed:

- the Nucleo **cannot be powered through the Shoe's USB power path alone**
- normal operation requires **battery power plus USB communication**

When separating the Nucleo from the Shoe, care must be taken not to bend the Morpho header pins.

---

# Mechanical Assembly Hardware

The controller stack and sensor hardware are mounted to the Romi using the following hardware:

- 4 × M2.5 x 8 mm standoffs
- 4 × M2.5 x 10 mm standoffs
- 4 × M2.5 x 30 mm standoffs
- 4 × M2.5 x 6 mm socket-head cap screws
- 4 × M2.5 x 8 mm socket-head cap screws
- 4 × M2.5 x 10 mm socket-head cap screws
- 8 × M2.5 nylon lock nuts
- 8 × M2.5 nylon washers
- 1 × Acrylic Romi-to-Shoe adapter

Important assembly notes:

- Do not overtighten screws into the Romi chassis
- The ABS plastic chassis can be damaged by excessive force
- Nylon washers should be placed between screw heads and mounted boards where specified
- Avoid cross-threading the standoffs

---

# Motor Driver Interface

The Romi drive system uses **DRV8838 motor drivers**. Each motor driver uses three control signals:

- **nSLP** - enables or disables the driver
- **DIR** - controls motor direction
- **PWM** - controls motor effort

For two motors, the controller requires:

- 2 digital outputs for **nSLP**
- 2 digital outputs for **DIR**
- 2 PWM-capable timer outputs for **PWM**

---

# Line Detection Sensor

## QTR-MD-08A Reflectance Sensor Array

The robot uses a **Pololu QTR-MD-08A** reflectance sensor array for line detection. This is an **8-channel analog-output infrared reflectance sensor** with **8 mm spacing** between adjacent sensors.

## Key Specifications

- **Sensor count:** 8
- **Sensor pitch:** 8 mm
- **Dimensions:** 61.0 x 20.0 x 2.5 mm
- **Operating voltage:** 2.9 V to 5.5 V
- **Output type:** analog voltage
- **Full-brightness LED current:** 30 mA
- **Max board current:** 125 mA
- **Optimal sensing distance:** 5 mm
- **Maximum recommended sensing distance:** 40 mm

## Operation

Each sensor uses an IR emitter and phototransistor pair to measure surface reflectance.

- Light surfaces reflect more IR light
- Dark surfaces reflect less IR light

By reading all eight analog channels, the controller can estimate the line position relative to the robot centerline.

The sensor array also supports **dimmable LED brightness control**, which can be useful during calibration and tuning.

## Purpose

The QTR array enables:

- line detection
- line position estimation
- closed-loop line following

---

# Collision Detection

## Romi Bumper Switch Modules

The robot uses Romi bumper switch modules for obstacle detection. Each module contains **three snap-action roller-lever switches** and can be assembled as either a **left** or **right** module.

When used as a pair, the system can mount up to **six bumper switches** across the front half of the robot.

## Features

- Three switches per module
- Left or right assembly depending on header placement
- 0.1 in pitch connection compatibility
- Can be used with standard female headers and jumper wires

## Electrical Behavior

Each bumper output is connected to a **normally open switch contact**, so the signals should use:

- external pull-up resistors, or
- MCU digital inputs with **internal pull-ups enabled**

When a switch is pressed:

- the corresponding signal is driven **low**

## Purpose

The bumper switches allow the robot to:

- detect contact with an obstacle
- stop or reverse when a collision occurs
- identify approximate impact location based on which switch was pressed

---

# Inertial Measurement Unit

## BNO055 IMU Breakout Board

The robot uses a **BNO055 IMU breakout board** to measure motion and orientation.

## Sensor Functions

The IMU provides:

- angular velocity
- linear acceleration
- orientation estimation

These measurements support:

- heading estimation
- turn control
- inertial sensing
- future state estimation or sensor fusion tasks

## Communication

The IMU communicates with the microcontroller using **I2C**.

Example wiring:

| IMU Pin | MCU Pin | Function |
|--------|---------|----------|
| SCL    | PB13    | I2C clock |
| SDA    | PB14    | I2C data |
| VCC    | 3.3V or 5V | Power |
| GND    | GND     | Ground |

If your project version uses a **BNO085** instead, this section can be updated to match the actual board while keeping the same general purpose.

---

# Cabling and Connectors

The Romi platform requires several cable assemblies between the Romi power distribution board, the Nucleo, and the sensors.

## Cable Types

- power cable
- left motor cable
- right motor cable
- left encoder cable
- right encoder cable
- IMU cable
- QTR sensor cable
- bumper switch cable(s)

The assembly process uses pre-crimped **Dupont-style jumper wires**, which are re-pinned into grouped housings as needed.

## Jumper Wire Hardware

The cable stock used for custom harnessing is based on:

- **2.54 mm pitch female-to-female Dupont wires**
- **20 cm wire length**
- multicolor ribbon-style cable

This style of wire is useful for internal harnesses and grouped sensor connections.

## Header Hardware

Additional connections can be made using:

- **0.100 in (2.54 mm) straight male header strips**
- **1 x 40 pin breakaway header sections**

These can be cut into smaller sections as needed for custom interfaces.

---

# Standard Romi Cable Signals

## Motor Cable Signals

| Signal | Function |
|--------|----------|
| nSLP   | Motor driver enable |
| DIR    | Motor direction |
| PWM    | Motor effort command |

## Encoder Cable Signals

| Signal | Function |
|--------|----------|
| Channel A | Encoder phase A |
| Channel B | Encoder phase B |

## Power Cable Signals

| Signal | Function |
|--------|----------|
| GND    | Ground |
| VIN    | Battery power input to Nucleo |

**Important:** The Romi battery power connection must be routed to **VIN**, **not 5V**.

An incorrectly wired power cable can destroy the Nucleo immediately when power is applied.

---

# Power and Safety Notes

The following precautions are critical during assembly and use:

- The Shoe of Brian must have the **ferrite bead removed**
- The Romi power cable must connect battery power to **VIN**, not 5V
- Incorrect power wiring can permanently damage the Nucleo
- Do not overtighten hardware into the plastic Romi chassis
- Do not intentionally stall the motors
- Keep the wheels clean to reduce slip and improve control performance
- Route cables carefully so they emerge near the correct Nucleo pins

---

# Summary

The Romi robot platform combines a differential-drive chassis, embedded controller, power electronics, and multiple onboard sensors to support autonomous navigation and closed-loop control.

The major hardware roles are:

- **DC motors and encoders** for motion and wheel feedback
- **Nucleo + Shoe of Brian** for control and electrical interfacing
- **QTR-MD-08A** for line sensing
- **Bumper switches** for collision detection
- **BNO055 IMU** for orientation and inertial measurement

Together, these components allow the robot to follow lines, detect obstacles, estimate motion, and perform controlled autonomous behaviors.
