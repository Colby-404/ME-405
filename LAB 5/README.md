# ME 405 – Lab 0x05
## Line Following with Cascaded Control

Author: Colby Cordoba and Hobbs Hegedus  
Course: ME 405 – Mechatronics  
Institution: California Polytechnic State University, San Luis Obispo  

---

## Overview

The objective of this lab was to implement closed-loop line following on the Romi robot using the Pololu QTR-MD-08A analog reflectance sensor array. The goal was to make the robot reliably follow a circular track by combining line sensor feedback with the wheel-speed control framework developed in earlier labs.

This lab was built as an extension of the Lab 0x04 multitasking control architecture, but with several important improvements. In the earlier version of the system, the wheel-speed controller was implemented as a separate task. While functional, that structure introduced synchronization issues between encoder updates, control computations, and motor commands. It also made sign conventions and unit consistency harder to manage, which led to unstable behavior such as runaway motion and oscillation.

To address these issues, the wheel PI controller was moved directly inside each motor task. This restructuring made the interface cleaner, reduced the number of timing dependencies between tasks, and made it easier to integrate an outer-loop line-following controller. The final system uses a cascaded control structure in which the line-follow task computes a steering correction, and the motor tasks independently regulate left and right wheel speed using internal PI control.

---

## Hardware Setup

The hardware platform used in this lab was the Pololu Romi robot configured with wheel encoders, motor drivers, and an analog reflectance sensor array mounted at the front of the chassis.

### Main Hardware Components

| Component | Description |
|----------|-------------|
| Romi Chassis | Differential drive mobile robot platform |
| DC Gear Motors | Provide propulsion for the robot |
| DRV8838 Motor Drivers | H-bridge motor drivers used for left and right motors |
| Quadrature Encoders | Measure wheel position and speed |
| STM32 Nucleo-L476RG | Microcontroller running MicroPython |
| QTR-MD-08A Sensor Array | 8-channel analog reflectance sensor array for line detection |

The QTR sensor array was connected to ADC-capable STM32 pins so that each sensor channel could be sampled independently.

### Line Sensor Pins

| Sensor Output | MCU Pin |
|--------------|---------|
| OUT7 | PC5 |
| OUT6 | PC4 |
| OUT5 | PB1 |
| OUT4 | PB0 |
| OUT3 | PA7 |
| OUT2 | PA6 |
| OUT1 | PA5 |
| OUT0 | PA4 |

These analog readings were used to determine the position of the line relative to the robot center.

---

## System Architecture

The overall system uses a multitasking architecture based on cooperative scheduling. The software is divided into independent tasks that exchange information through shares and queues.

### Main Tasks

| Task | Responsibility |
|-----|-----|
| `task_motor` | Reads encoder data, performs internal PI speed control, actuates motor, logs data |
| `task_follow_line` | Computes steering correction from line error and updates wheel setpoints |
| `task_user` | Handles data collection commands and printing |
| `task_tuning_ui` | Handles gain tuning, calibration commands, and line-follow settings |
| `cotask` scheduler | Runs tasks periodically using cooperative multitasking |

This task-based architecture allows sensing, control, user interaction, and motor actuation to run concurrently while maintaining modular code organization.

---

## Design Improvements from Previous Lab

A major part of this lab involved restructuring the previous control framework.

### Internal PI Control in Each Motor Task

In the previous architecture, wheel velocity control was implemented in a separate control task. That meant the motor task had to:

1. update the encoder  
2. publish measured speed  
3. wait for a separate control task to compute effort  
4. read effort back later  

This created timing dependencies between tasks and made debugging more difficult.

In this lab, the PI wheel-speed controller was moved directly into each motor task. This improved the design in several ways:

- reduced synchronization problems between tasks  
- created a cleaner interface for the outer-loop line controller  
- kept encoder measurement and effort computation in the same task  
- made sign conventions easier to manage  
- reduced oscillation and runaway issues caused by inconsistent units  

This restructuring made the system more robust and easier to tune.

### Units and Sign Convention Cleanup

Another important improvement was cleaning up unit consistency and sign conventions.

The encoder driver was updated to preserve raw counts and counts-per-second as the default outputs, while still supporting optional scaled conversions. This made the inner PI loop easier to reason about because the setpoints and measurements remained in the same units.

The encoder class also supports an `invert` option so that wheel direction conventions can be corrected in software without rewiring hardware.

---

## Line Sensor Driver

A new hardware driver was written for the QTR-MD-08A analog line sensor array.

### Sensor Operation

Each QTR sensor produces an analog voltage based on the reflectance of the surface below it:

- white surface → stronger reflectance  
- black line → weaker reflectance  

The driver reads all eight analog channels using STM32 ADC inputs.

### Calibration

To improve robustness, the sensor driver supports calibration for both white background and black line conditions.

The calibration process records per-channel reference values for:

- white surface
- black line

These values are then used to normalize each channel reading to a common 0 to 1000 scale.

### Normalized Sensor Values

After calibration, the sensor driver converts raw ADC readings into normalized values so that the array can be used consistently across different surfaces and lighting conditions.

This makes line detection more reliable and reduces the effect of variations between individual sensor channels.

### Centroid-Based Line Position

The line position is estimated using a centroid-style weighted average of the sensor readings. The sensor array returns a line position in “Pololu units” ranging from:

- 0 at one side of the array  
- to (N−1)×1000 at the other side  

The signed line-follow error is then computed relative to the center of the array.

This provides a continuous error signal that is much more useful for control than a simple binary “line detected / not detected” approach.

---

## Line-Reading Task

The line-reading behavior is integrated into the line-follow framework through sensor sampling and centroid processing.

The line-processing logic performs the following actions:

1. reads normalized values from the QTR sensor array  
2. computes the centroid-based line position  
3. converts that position into a signed line error  
4. checks whether the line signal is strong enough to be valid  
5. publishes line error and line-valid status to shares  

The system also supports calibration commands through shared variables:

- `cal_cmd = 1` → calibrate white
- `cal_cmd = 2` → calibrate black
- after calibration → `cal_done = 1`

This allows calibration to be triggered from the tuning user interface without needing to rewrite the sensor task.

---

## Cascaded Control Structure

The robot uses a cascaded control architecture with:

- an **outer-loop line-follow controller**
- an **inner-loop wheel-speed controller**

### Inner Loop: Wheel PI Control

Each motor task runs its own internal PI controller using encoder velocity feedback.

The inner loop regulates wheel speed in units of **counts per second** and computes motor effort directly.

This is beneficial because:

- wheel speed control stays tightly coupled to encoder measurement
- effort is computed locally inside the wheel task
- the outer loop only needs to command desired wheel setpoints

### Outer Loop: Line-Follow PI Control

The line-follow task acts as the outer loop.

Its purpose is to take the line error and compute a steering correction `dv`. That steering correction is then added to one wheel setpoint and subtracted from the other.

Conceptually:

- left setpoint = nominal speed − steering correction
- right setpoint = nominal speed + steering correction

This differential steering causes the robot to turn back toward the line.

### Anti-Windup and Saturation

The outer-loop controller includes saturation on steering correction and simple anti-windup logic. This helps prevent the integral term from growing excessively when the line error is large or the steering output is clamped.

The final wheel setpoints are also limited to reasonable minimum and maximum values to protect the motor controller from extreme commands.

---

## Motor Task Behavior

The motor task was restructured significantly in this lab.

Each motor task now performs the following steps every cycle when enabled:

1. update encoder position and velocity  
2. publish measured position and omega to shares  
3. compute wheel effort internally using PI control if enabled  
4. apply effort to the motor driver  
5. optionally log measured data into queues  

A key design decision in this lab was to keep **logging separate from motor operation**. If the logging queues fill up, the task stops logging, but the motor continues running normally. This prevents experimental data collection from interfering with actual control performance.

This makes the task safer and more predictable during long runs.

---

## User Interface

Two user interfaces are used in this lab.

### Tuning UI

The tuning interface is active when `start_user == 0`.

It allows the user to:

- set wheel PI gains
- set nominal wheel speed
- toggle line following
- set line-follow gains
- arm and perform sensor calibration
- print line-follow status
- run motors without logging
- stop the motors
- switch into data collection mode

Important tuning commands include:

| Command | Action |
|--------|--------|
| `h` | Print help menu |
| `k` | Set wheel gains `Kp`, `Ki` |
| `s` | Set nominal speed `v_nom` |
| `f` | Toggle line-follow enable |
| `o` | Set line gains `Kp_line`, `Ki_line` |
| `c` | Arm calibration workflow |
| `w` | Calibrate white |
| `B` | Calibrate black |
| `b` | Cancel calibration |
| `p` | Print one-shot line status |
| `q` | Stream line status |
| `n` | Run motors for live test |
| `x` | Stop motors |
| `g` | Switch to data collection UI |

### Data Collection UI

The data collection interface is active when `start_user == 1`.

It allows the user to:

- collect left motor data
- collect right motor data
- collect both simultaneously
- stop motors during collection
- toggle live printing
- return to the tuning menu

Commands include:

| Command | Action |
|--------|--------|
| `l` | Collect and print left motor data |
| `r` | Collect and print right motor data |
| `b` | Collect and print both motors |
| `x` | Stop motors |
| `v` | Toggle live print |
| `h` | Return to tuning menu |

This split-interface design keeps tuning and data collection organized and prevents command conflicts during operation.

---

## Shares and Queues

The lab uses a large number of shares and queues to coordinate the multitasking system.

### Important Shares

| Share | Meaning |
|------|---------|
| `motorLGo`, `motorRGo` | Enable flags for left and right wheel tasks |
| `Kp`, `Ki` | Wheel PI gains |
| `setpointL`, `setpointR` | Per-wheel velocity setpoints |
| `omegaL`, `omegaR` | Measured wheel velocities |
| `posL`, `posR` | Measured wheel positions |
| `effortL`, `effortR` | Motor effort commands |
| `follow_en` | Enable flag for line-follow controller |
| `v_nom` | Nominal robot speed |
| `Kp_line`, `Ki_line` | Outer-loop line-follow gains |
| `line_err` | Signed line-follow error |
| `dv_out` | Steering correction from outer loop |
| `line_ok` | Valid line detected flag |
| `cal_cmd` | Calibration command |
| `cal_done` | Calibration complete flag |
| `start_user` | Chooses between tuning UI and data UI |

### Queues

Queues are used primarily for logging wheel data during experiments:

| Queue | Meaning |
|------|---------|
| `dataValuesL`, `timeValuesL` | Left motor log data and timestamps |
| `dataValuesR`, `timeValuesR` | Right motor log data and timestamps |

These queues allow the system to capture experimental data without disrupting closed-loop operation.

---

## Data Logging and Analysis

Data collected from the system can be exported over serial and analyzed using a host-side Python script.

In this lab, the logging utility was updated so that it no longer behaves like the old step-response collector. Instead, it supports line-follow logging and status monitoring from the updated tuning interface.

This allows experiments such as:

- observing line error over time
- observing steering correction output
- checking whether line following is enabled
- evaluating line-follow controller tuning

The resulting plots can be used to understand how well the robot tracks the line and how the outer-loop controller behaves during turning.

---

## Discussion

This lab required significant restructuring of the previous control framework. The most important improvement was moving the wheel PI controller inside the motor task. This reduced synchronization problems between tasks and made the overall architecture cleaner and easier to tune.

Another major improvement was enforcing more consistent units and sign conventions. Earlier versions of the code mixed units and direction assumptions in a way that made debugging difficult. By cleaning up the encoder interface and keeping the inner loop in counts per second, the behavior became much more predictable.

The addition of the QTR analog driver and centroid-based line error calculation also improved the quality of the feedback signal. Rather than simply detecting whether the robot was on the line, the controller can now estimate how far the robot is from the center of the line, allowing smoother steering corrections.

The cascaded control architecture used in this lab closely reflects real robotics practice. The outer loop focuses on path tracking, while the inner loops regulate the wheel dynamics needed to achieve that path.

---

## Conclusion

This lab successfully extended the Romi control system to perform closed-loop line following using a cascaded control architecture.

A new analog line sensor driver was developed for the QTR-MD-08A array, including white/black calibration and centroid-based line error calculation. The previous wheel-speed control architecture was restructured by moving the PI controller directly inside the motor tasks, which improved timing behavior, code organization, and controller stability.

The final system combines:

- normalized line sensor feedback
- outer-loop steering control
- inner-loop wheel speed regulation
- multitasking coordination through shares and queues
- serial interfaces for calibration, tuning, and data collection

This lab represents a major step toward autonomous navigation on the Romi platform and provides a strong foundation for future work involving higher-speed line following, IMU integration, and more advanced state estimation.

---

## Files in This Folder

| File | Description |
|-----|-----|
| `main.py` | Main program that initializes hardware, shares, tasks, and scheduler |
| `motor_driver.py` | Low-level motor driver interface |
| `encoder.py` | Encoder interface with sign inversion and scaled outputs |
| `sensor_driver.py` | QTR analog line sensor driver with calibration and normalization |
| `task_motor.py` | Motor task with internal PI speed control |
| `task_follow_line.py` | Outer-loop line-follow controller |
| `task_user.py` | Tuning UI and data collection UI |
| `task_share.py` | Shared variables and queues for multitasking |
| `cotask.py` | Cooperative task scheduler |
| `step_collector_loop.py` | Host-side logging and line-follow analysis script |
| `README.md` | Documentation explaining the lab |
