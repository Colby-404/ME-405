# ME 405 – Lab 0x01
## Romi Hardware Confirmation

Author: Colby Cordoba and Hobbs Hegedus  
Course: ME 405 – Mechatronics  
Institution: California Polytechnic State University, San Luis Obispo  

---

## Overview

The objective of this lab was to verify that the Romi robot platform was assembled correctly and that the primary hardware components were functioning as expected. This included confirming proper operation of the DC motors, motor drivers, and quadrature encoders.

A MicroPython program was developed to generate PWM signals that drive the motors at various speeds and directions. The quadrature encoders attached to each motor were read using the STM32 hardware timers configured in encoder mode. Encoder counts were monitored while the motors were running to confirm that the counts increased or decreased depending on the direction of motion.

Successful completion of this lab confirms that the robot platform is capable of controlled motion and provides reliable encoder feedback, which will be required for later labs involving closed-loop control, line following, and sensor integration.

---

## Hardware Setup

The experiment used a **Pololu Romi robot chassis** controlled by an **STM32 Nucleo-L476RG microcontroller** mounted on a **Shoe of Brian interface board**.

### Main Hardware Components

| Component | Description |
|----------|-------------|
| Romi Chassis | Mobile robot platform with wheels and motors |
| DC Motors | Provide propulsion for the robot |
| Quadrature Encoders | Measure wheel rotation |
| Nucleo-L476RG | Microcontroller used to run control software |
| Shoe of Brian | Interface board connecting the Nucleo to the Romi platform |
| BNO055 IMU | Inertial measurement unit used for orientation sensing |

The Romi chassis includes a power distribution board that supplies battery power to the motors and the Nucleo microcontroller.

---

### Motor Connections

Each motor is controlled using three signals:

| Signal | Function |
|------|------|
| DIR | Sets motor direction |
| nSLP | Enables or disables the motor driver |
| PWM | Controls motor effort via duty cycle |

The PWM signals are generated using hardware timers on the Nucleo.

---

### Encoder Connections

Each motor has an attached quadrature encoder with two channels.

| Channel | Purpose |
|------|------|
| Channel A | Encoder pulse signal |
| Channel B | Phase-shifted pulse signal |

These signals are connected to hardware timers configured in **encoder mode**, allowing the microcontroller to automatically track wheel rotation.

---

## Theory

### PWM Motor Control

DC motors are commonly controlled using **pulse width modulation (PWM)**.

PWM works by rapidly switching the motor voltage on and off. The ratio of the on-time to the total period of the signal is known as the **duty cycle**.

For example:

- 20% duty cycle → low motor effort  
- 80% duty cycle → high motor effort  

Because DC motors behave like low-pass filters, they respond primarily to the **average voltage** of the PWM signal rather than the rapid switching itself.

---

### Quadrature Encoders

Quadrature encoders measure rotational motion using two digital signals that are offset by 90 degrees in phase.

By monitoring the order in which the signals change, the system can determine both:

- **rotation magnitude**
- **rotation direction**

Each transition of the encoder signals corresponds to a small increment of motor rotation.

Hardware timers on the STM32 microcontroller can decode these signals automatically, allowing the system to count encoder ticks efficiently.

---

## Software Implementation

The MicroPython program performs the following tasks:

1. Configure GPIO pins for motor direction and driver enable.
2. Configure a hardware timer to generate PWM signals.
3. Set the PWM frequency to **20 kHz** for smooth motor operation.
4. Configure hardware timers to read encoder signals.
5. Run a sequence of motor tests at different speeds and directions.
6. Continuously read encoder counts while the motors spin.
7. Print encoder position data to the terminal.

---

## Key Code Concepts

### Motor Direction Control

Motor direction is controlled using digital output pins.

Example configuration:

```python
DIR_L = pyb.Pin('PC0', pyb.Pin.OUT_PP)
DIR_R = pyb.Pin('PC2', pyb.Pin.OUT_PP)
