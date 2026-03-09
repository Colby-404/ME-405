# ME 405 – Lab 0x06
## IMU Integration and State Estimator Implementation

Author: Colby Cordoba and Hobbs Hegedus  
Course: ME 405 – Mechatronics  
Institution: California Polytechnic State University, San Luis Obispo  

---

## Overview

The objective of this lab was to extend the Romi robot from a line-following platform into a sensor-fused embedded system by integrating an inertial measurement unit (IMU) and implementing a discrete-time state estimator. The final goal was not only to control the robot using wheel encoders and line sensors, but also to estimate additional robot states such as forward travel, heading, and planar position using a combination of wheel, IMU, and control-input information.

This lab built on the architecture from the previous line-following lab, but it required substantial restructuring to fit within the memory limits of the STM32 running MicroPython. The final software combines:

- internal PI wheel-speed control inside each motor task
- outer-loop line following
- IMU sensing using a BNO055
- a discrete-time observer / state estimator
- multitasking coordination through shares
- a lean serial tuning and data collection interface

A major challenge in this lab was not purely the control theory, but the practical constraint of **limited RAM** on the embedded target. As more subsystems were added, especially the IMU task, estimator task, UI features, and extra diagnostic printing, the system became increasingly vulnerable to memory allocation failures and unstable behavior. Because of this, a large part of the design effort focused on reducing allocation overhead, simplifying the UI, and restructuring the code so the system could still run reliably.

---

## Hardware Setup

The hardware platform used in this lab was the same Pololu Romi robot platform from previous labs, now expanded to include an IMU.

### Main Hardware Components

| Component | Description |
|----------|-------------|
| Romi Chassis | Differential drive mobile robot platform |
| DC Gear Motors | Provide propulsion for left and right wheels |
| DRV8838 Motor Drivers | H-bridge motor drivers for wheel actuation |
| Quadrature Encoders | Measure wheel position and velocity |
| QTR-MD-08A Sensor Array | Analog line-following reflectance sensor array |
| BNO055 IMU | 9-axis inertial measurement unit |
| STM32 Nucleo-L476RG | Main microcontroller running MicroPython |

The BNO055 IMU was connected over a software I2C interface using:

| IMU Signal | MCU Pin |
|-----------|---------|
| SCL | PB13 |
| SDA | PB14 |

The line sensor array remained connected through the analog input pins used in the previous lab, and the motors and encoders continued to use Timer 4 for PWM generation and dedicated hardware timers for quadrature decoding.

---

## System Architecture

The system retained the multitasking architecture from the previous lab, but was expanded to include an IMU task and a state estimator task.

### Main Tasks

| Task | Responsibility |
|-----|-----|
| `task_motor` | Reads encoder data, performs internal PI speed control, actuates motor, publishes measurements |
| `task_follow_line` | Computes steering correction from line error and updates wheel setpoints |
| `task_imu` | Reads BNO055 data and publishes heading, yaw rate, and calibration status |
| `task_state_estimator` | Estimates robot motion and pose using wheel, IMU, and effort information |
| `task_tuning_ui` | Serial interface for tuning, streaming, estimator inspection, and IMU commands |
| `task_user` | Data collection UI for buffered left/right/both logging |
| `cotask` scheduler | Cooperative multitasking scheduler |

The architecture is layered so that low-level hardware drivers feed task-level logic, and task-level outputs are exchanged through shares. This modularity was essential once the estimator and IMU were introduced because it allowed the new functionality to be added without completely rewriting the existing line-following code.

---

## Low-RAM Design Strategy

One of the most important aspects of this lab was the move to a **low-RAM build**.

As the project grew, the addition of the IMU driver, estimator task, extra shares, and multiple user interfaces made memory usage a major issue. Earlier versions of the system had already begun to encounter memory-related problems, and this lab pushed the embedded memory budget even further.

To keep the program running, the final `main.py` uses several RAM-saving strategies:

1. `micropython.opt_level(3)` is enabled to reduce overhead.
2. `gc.collect()` is called repeatedly between imports.
3. imports are performed in a deliberate order so memory is reclaimed as early as possible.
4. long `name=` strings for shares are removed in the low-RAM build to reduce qstr memory usage.
5. the code avoids unnecessary runtime allocations wherever possible.
6. the UI is simplified and split into specialized roles rather than one large dynamic interface.

These changes are significant because they show that the final design was shaped not just by functionality, but by the realities of running a complex robotics program on limited embedded hardware.

---

## IMU Integration

The IMU subsystem is based on the BNO055 driver and a dedicated IMU task.

### IMU Driver

The IMU driver supports:

- chip ID verification on startup
- configuration mode and fusion mode switching
- heading and gyro data access
- calibration state reading
- calibration profile save/load support
- compatibility with a software I2C adapter

The driver is written to support minimal but essential BNO055 functionality so that it remains lightweight enough for the embedded system.

### IMU Task

The IMU task periodically reads the BNO055 and publishes:

- heading in degrees
- yaw rate in degrees per second
- raw calibration status byte

The task also supports runtime commands through shares, including:

- enabling or disabling IMU updates
- switching fusion mode
- zeroing heading
- saving calibration data

This gives the robot access to orientation information that cannot be obtained from encoders alone, especially during curved motion or when wheel slip is present.

---

## State Estimator

A major new component in this lab is the discrete-time state estimator.

### Estimator Purpose

The state estimator combines wheel encoder measurements, IMU measurements, and motor effort inputs to estimate robot states that are not directly measured or that benefit from sensor fusion.

The estimator publishes:

- estimated traveled distance
- estimated heading
- estimated x-position
- estimated y-position
- optional estimated wheel/pose outputs for debugging and proof of operation

### Estimated States

The observer state vector is organized around:

- left wheel angular motion
- right wheel angular motion
- forward travel state
- heading state

The estimator uses observer matrices already embedded in the task implementation. It also supports heading unwrapping and configurable sign handling for pose integration.

### Pose Integration

The estimator integrates heading and travel to reconstruct planar motion:

- `x_pos`
- `y_pos`
- `dist_traveled`

This allows the robot to maintain an internal estimate of where it has moved, rather than only reacting to the line directly.

### Why the Estimator Matters

This is an important conceptual step in the project because it moves the robot from simple feedback control to state-based reasoning. Even if the robot is still following a line, the estimator introduces the idea that the system can maintain an internal model of its motion using multiple sensing sources.

---

## Encoder and Motor Subsystems

The encoder and motor architecture from previous labs was retained, but adapted carefully so it would work with the estimator.

### Encoder Interface

The encoder driver in this lab is a hybrid compatibility version designed to preserve the existing line-follow control structure while also supporting optional scaled outputs.

By default:

- `get_position()` returns raw counts
- `get_velocity()` returns counts per second

This was a deliberate choice because the inner wheel-speed controller and estimator both benefit from staying in consistent count-based units.

The encoder also supports:

- timer object or timer number initialization
- sign inversion through `invert=True`
- optional scaled and rad-based outputs

### Motor Task

The motor task continues to use internal PI control. This was one of the key architectural improvements from the previous lab and was kept in this final build.

Its responsibilities include:

1. update encoder measurement
2. publish wheel position and velocity
3. compute or apply wheel effort
4. actuate the motor driver
5. optionally log data

Because the motor task handles encoder updates and wheel actuation directly, it provides a stable foundation for the higher-level line-follow and estimator logic.

---

## Line Following in the Final Build

Although this lab focuses on the IMU and estimator, the line-following system remains part of the final integrated architecture.

The robot still uses:

- the QTR analog line sensor driver
- a centroid-based line error
- an outer-loop line-follow controller
- nominal speed plus differential steering correction

This is important because the final build is not a separate “estimator-only” system. Instead, it is an integrated robot stack where line following, wheel PI control, IMU sensing, and state estimation all coexist within the same multitasking framework.

---

## User Interface and UI Restructuring

The user interface became one of the most important engineering issues in this lab.

### Why the UI Became a Problem

As more functionality was added, the UI became responsible for:

- wheel gain tuning
- nominal speed tuning
- line-follow gain tuning
- line-follow enable/disable
- IMU enable/disable
- IMU mode toggle
- heading zeroing
- calibration save commands
- line sensor calibration
- line and IMU status streaming
- estimator printing and streaming
- data collection and buffered output

This is a lot of responsibility for a small embedded system, especially when running in MicroPython. The more strings, menus, formatting logic, and serial printing the UI handled, the more memory pressure and CPU overhead it created.

### Practical UI Problems

A major practical issue was that the UI could interfere with control behavior and even contribute to memory instability.

Examples of the UI-related problems addressed in this lab include:

- too much live serial printing consuming CPU time
- excessive UI text increasing RAM pressure
- command handling becoming too large and complicated
- the need to avoid toggling features while motors were actively running
- the risk that debug output would make the line follower appear unstable or “wobbly”

The final code makes this explicit: verbose streaming is kept **off by default** because extra logging can steal CPU time and degrade line-follow performance.

### Final UI Strategy

To manage these issues, the UI was intentionally kept lean.

The tuning UI supports only the commands needed for:

- wheel gain tuning
- line-follow tuning
- IMU interaction
- estimator inspection
- controlled streaming

The data UI is kept separate and is used specifically for buffered data collection.

This separation is important because it avoids having one giant all-purpose interface trying to do everything at once.

### Live Printing Gate

The data UI also includes protections around live printing. For example, live printing is not meant to be toggled while motors are already running or collecting, because doing so could change timing behavior and complicate debugging.

This is a strong example of the final design philosophy in this lab: the UI must serve the experiment without disturbing the control system more than necessary.

---

## Memory Limitations and Their Effect on the Design

The memory issue was not just a minor inconvenience; it directly shaped the design of the final system.

### Why Memory Became Critical

The final build contains:

- motor drivers
- encoder drivers
- line sensor driver
- IMU driver
- motor tasks
- line-follow task
- IMU task
- estimator task
- tuning UI
- data collection UI
- many shares
- scheduler infrastructure

This is a substantial amount of functionality for a MicroPython target with limited RAM.

### Design Consequences

Because of these limitations, the final system had to prioritize:

- low allocation code
- reduced UI text
- minimal share naming
- explicit garbage collection
- carefully ordered imports
- simplified task interactions

In practice, this meant that some features which might be desirable in a desktop or larger embedded environment had to be kept lightweight or omitted entirely.

### Honest Project Status

This lab should be understood as both a successful integration effort and an exercise in embedded resource constraints. The system does integrate the IMU and estimator, but doing so required meaningful tradeoffs in UI complexity, logging verbosity, and memory usage.

That is an important engineering result in itself: the final design is not just the “feature-rich” version, but the version that could actually run reliably on the hardware.

---

## Serial Streaming and Host-Side Logging

The project also includes host-side logging support through `step_collector_loop.py`.

In the final version, this script supports streamed values related to:

- line error
- steering correction
- estimator outputs
- estimated pose
- IMU heading and yaw rate
- optional encoder/estimator-related fields when the firmware stream includes them

This makes it possible to capture real-time robot behavior without forcing all analysis to happen on the embedded system itself.

This separation is useful because plotting and large-scale data handling are better performed on the host computer, while the embedded code focuses on real-time control and estimation.

---

## Design Philosophy

This final lab highlights several important design ideas.

### 1. Embedded Software Must Respect Hardware Limits

The final structure shows that “more features” is not always the same as “better design” on embedded hardware. Features must fit within timing and memory limits.

### 2. Layered Architecture Matters

The code follows a layered structure:

- hardware drivers
- task-level behavior
- scheduler coordination
- serial UI and logging

This makes the system easier to debug and extend.

### 3. UI Should Not Dominate the Robot

The UI is useful for tuning and debugging, but excessive printing and menu complexity can harm control performance. This lab makes that tradeoff very clear.

### 4. State Estimation Extends the Robot Beyond Reactive Control

The estimator adds internal state knowledge to the robot, allowing it to estimate motion and pose rather than only responding to immediate sensor input.

---

## Discussion

This lab represents one of the most complete software integrations in the Romi project. It combines wheel-speed control, line following, IMU integration, and observer-based state estimation inside a single multitasking embedded system.

The most interesting part of the lab is that success did not come only from implementing the estimator equations or reading the IMU correctly. A large part of the work involved restructuring the code so the entire system could coexist in limited RAM without breaking the user interface or destabilizing runtime behavior.

The final result is therefore both a control-system achievement and an embedded software engineering achievement. It demonstrates how advanced robotics functionality must often be balanced against hardware constraints, especially in interpreted embedded environments such as MicroPython.

---

## Conclusion

This lab successfully integrated an IMU and a discrete-time state estimator into the Romi control architecture.

The final system includes:

- internal PI wheel-speed control
- line-follow outer-loop control
- BNO055 heading and yaw-rate sensing
- a discrete-time observer for motion and pose estimation
- host-side streaming and logging
- a lean low-RAM multitasking software structure

A major lesson from this lab was that the system architecture had to be shaped around both functionality and resource constraints. In particular, UI complexity and memory usage became central design issues. The final code reflects those constraints directly through import ordering, garbage collection, reduced qstr usage, lean serial interfaces, and limited live printing.

This lab provides a strong foundation for future work in embedded robotics, especially where sensing, estimation, and control must all operate together on constrained hardware.

---

## Files in This Folder

| File | Description |
|-----|-----|
| `main.py` | Low-RAM integrated main program with motors, line follow, IMU, and estimator |
| `motor_driver.py` | Low-level motor driver interface |
| `encoder.py` | Hybrid encoder interface preserving counts-based control outputs |
| `sensor_driver.py` | QTR analog line sensor driver |
| `imu_driver.py` | BNO055 IMU driver with calibration/profile support |
| `task_motor.py` | Motor task with internal PI control |
| `task_follow_line.py` | Outer-loop line-follow controller |
| `task_imu.py` | Periodic IMU acquisition and publishing task |
| `task_state_estimator.py` | Discrete-time observer / state estimator task |
| `task_tuning_ui.py` | Lean tuning and streaming UI |
| `task_user.py` | Buffered data collection UI |
| `task_share.py` | Shares and queues for task communication |
| `cotask.py` | Cooperative multitasking scheduler |
| `step_collector_loop.py` | Host-side estimator and line-follow logger |
| `ui_help.py` | Help menu text for tuning UI |
| `bno055_calib.bin` | Saved IMU calibration profile |
| `README.md` | Documentation explaining the lab |
