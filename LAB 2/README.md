# ME 405 – Lab 0x02
## Writing Hardware Drivers

Author: Colby Cordoba and Hobbs Hegedus  
Course: ME 405 – Mechatronics  
Institution: California Polytechnic State University, San Luis Obispo  

---

## Overview

The objective of this lab was to develop reusable MicroPython hardware drivers for the Romi robot platform. Specifically, two driver classes were implemented: a permanent magnet DC (PMDC) motor driver and a quadrature encoder interface.

The motor driver class encapsulates the control of the DRV8838 motor drivers on the Romi chassis using PWM for motor effort, a direction pin to control forward or reverse rotation, and a sleep pin to enable or disable the driver safely. The encoder driver class interfaces with the magnetic quadrature encoders attached to each motor using STM32 hardware timers configured in encoder mode.

By creating object-oriented drivers for these hardware components, the system becomes modular and easier to maintain. Instead of directly manipulating pins and timers throughout the program, the user can interact with motors and encoders through simple function calls. These drivers will serve as the foundation for future labs involving closed-loop control, velocity estimation, and autonomous robot behaviors. :contentReference[oaicite:0]{index=0}

---

## Hardware Setup

The experiment used the **Pololu Romi robot platform** controlled by an **STM32 Nucleo-L476RG microcontroller** mounted on the **Shoe of Brian interface board**.

### Main Hardware Components

| Component | Description |
|----------|-------------|
| Romi Chassis | Differential drive robot platform |
| DC Gear Motors | Provide propulsion for left and right wheels |
| DRV8838 Motor Drivers | Amplify MCU control signals to drive motors |
| Quadrature Encoders | Measure motor rotation |
| STM32 Nucleo-L476RG | Microcontroller running MicroPython |

Each wheel contains a **magnetic quadrature encoder attached to the motor shaft**, allowing the system to measure wheel displacement and rotational velocity.

---

### Motor Driver Interface

The Romi chassis uses **DRV8838 motor drivers**, which implement an H-bridge to drive the motors.

Three signals are required to control each motor:

| Signal | Function |
|------|------|
| PWM | Controls motor effort using duty cycle |
| DIR | Selects forward or reverse direction |
| nSLP | Enables or disables the motor driver |

The PWM signal determines the effective voltage applied to the motor by adjusting the duty cycle of a high-frequency switching waveform. :contentReference[oaicite:1]{index=1}

---

### Encoder Connections

Each motor includes a **two-channel quadrature encoder**.

| Channel | Function |
|------|------|
| Channel A | Encoder signal A |
| Channel B | Encoder signal B |

These signals are connected to **STM32 hardware timers configured in encoder mode**, allowing the microcontroller to automatically decode the quadrature signals and count rotation events.

---

## Theory

### PWM Motor Control

Permanent magnet DC motors are typically controlled using **pulse width modulation (PWM)**. In PWM control, the motor voltage is switched rapidly between on and off states. The duty cycle of the signal determines the average voltage applied to the motor.

For example:

- 20% duty cycle → low motor effort  
- 80% duty cycle → high motor effort  

Because DC motors act as **low-pass filters**, they respond primarily to the average value of the PWM signal rather than the rapid switching itself. :contentReference[oaicite:2]{index=2}

---

### Quadrature Encoders

Quadrature encoders measure rotational displacement using two digital signals that are offset in phase. Each transition in the signals corresponds to a small movement of the motor shaft.

By observing the sequence of signal transitions, the system can determine both:

- the amount of rotation  
- the direction of rotation

The STM32 microcontroller includes hardware timers that can decode these signals automatically, allowing the system to track encoder counts efficiently. :contentReference[oaicite:3]{index=3}

---

### Timer Counter Overflow

Hardware timers used for encoder counting have a finite range (typically 16-bit), meaning the counter eventually wraps around when it reaches its maximum value.

To avoid errors caused by counter overflow, the encoder driver calculates **the difference between consecutive timer readings** and accumulates the signed change in position. This method allows the system to track position continuously even when the timer counter wraps around. :contentReference[oaicite:4]{index=4}

---

## Software Implementation

Three Python files were created for this lab:

| File | Purpose |
|-----|-----|
| `motor.py` | Motor driver class for controlling the DRV8838 motor driver |
| `encoder.py` | Encoder driver class for reading quadrature encoders |
| `main.py` | Test program used to verify motor and encoder functionality |

The drivers are implemented using **Python classes**, allowing multiple motors and encoders to be instantiated as independent objects.

---

## Key Code Concepts

### Motor Driver Class

The motor driver class encapsulates the control of the DRV8838 motor driver. It manages the PWM channel, direction pin, and sleep pin required to operate the motor.

Example initialization:

```python
motorL = Motor(pwmL, 'PC0', 'PC1')
motorR = Motor(pwmR, 'PC2', 'PC3')
