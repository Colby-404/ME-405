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

1. an **in-class robot demonstration and time-trial**, where we present the design and runs the robot on the printed course
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

This term project brings together the major topics of ME 405 into one complete robotic system. The final result is an autonomous Romi robot that uses embedded hardware and software, sensor feedback, and closed-loop control to follow a track and compete in a time-trial event. The project demonstrates the integration of mechatronics concepts, including system assembly, driver development, multitasking control, sensing, estimation, and real-world performance tuning.

The repository also contains `.mpy` files, which are compiled versions of MicroPython `.py` files. The files are precompiled bytecode and exist to reduce the memory usage, and therefore run faster and avoid runtime compilation overhead. 

# Game Track Overview

The game track is a multi-section course designed to test the Romi robot’s ability to follow lines, handle curves, recognize intersections, and navigate between multiple checkpoints. The course includes both fast sections and precision-control sections, requiring the robot to balance speed with accurate tracking.

The robot begins in the **start circle at CP#0 / CP#5** in the upper-left corner of the map. From there, the main route includes a **long straightaway across the top of the course**, followed by a **large right-hand turn** near **CP#1**. This section is useful for testing high-speed line following and straight-line stability.

The left and lower portions of the course include **large-radius curves** and a **wavy S-curve section** leading toward **CP#3** and **CP#2**. These parts of the track test how well the robot can maintain line position through changing curvature and transition smoothly between turns.

The center of the map includes a **branching/dashed path near CP#4**, which appears to represent a secondary route or special navigation segment. This section can be used to test decision-making, alternate routing, or behavior changes between different course features.

Near the right side of the map, the course also includes a **wall region**, a **grid-like zone**, and a **cross intersection** near **CP#2**. These features add complexity beyond simple line following.

Additional **circular marked zones** are placed around the course and may be used for calibration, detection tasks, or checkpoint-based actions. The **IR calibration corner** in the upper-right corner is specifically intended for reflectance sensor setup and calibration before or during testing.

Overall, the track is designed to evaluate:

- line-following accuracy
- speed on straight sections
- turning performance on tight and wide curves
- stability through S-curves
- behavior at intersections and branching paths
- repeatability across multiple checkpoints

This layout makes the course a full-system test of the robot’s sensing, control, and navigation performance.

<img width="5139" height="2554" alt="image" src="https://github.com/user-attachments/assets/a256f24c-b7c3-4547-8ffa-549ae1974432" />


# Final Robot Configuration

As detailed in **HARDWARE SETUP.md**, **HARDWARE DESCRIPTION.md**, and the project **PINOUT.xlsx**, the final term project robot was assembled as a fully integrated Romi-based autonomous robot for line following and time-trial navigation. The platform used a **Pololu Romi chassis** with two independently driven wheels, wheel encoders for motion feedback, an **STM32 Nucleo-L476RG** mounted on a **Shoe of Brian**, and a front-mounted reflectance sensor array for detecting and tracking the course line.

The final hardware arrangement combined the controller stack, power system, sensor wiring, and drive hardware into a compact and testable layout. The Nucleo and interface board were mounted above the chassis, the cables were routed to the motors and encoders through the hardware stack, and the front sensor assembly was positioned to provide reliable line readings during motion. This final setup was the hardware configuration used for testing, tuning, and the final project demonstration.

The image below shows the completed robot assembly used in the term project.


![IMG_2487](https://github.com/user-attachments/assets/a105b7a0-475f-4500-81f6-4237cec445a9)

# System Architecture

The software can be understood as five major parts

1. **Hardware interface**
   `motor_driver.py`, `encoder.py`, and `sensor_driver.py` talk directly to the motor driver, quadrature encoders, and QTR analog array.

2. **Wheel control**
   `task_motor.py` turns speed setpoints into PWM effort using a PI controller for each wheel.

3. **Behavior**
   `task_follow_line.py` computes differential wheel setpoints from line error and also runs scripted encoder-based segments when necessary.
   `task_bumper_recovery.py` overrides normal motion when the bumpers detect contact.

4. **User interaction layer**
   `task_user.py` and `ui_help.py` provide serial commands, streaming status, and calibration controls.

5. **Scheduling infrastructure**
   `cotask.py` and `task_share.py` support cooperative multitasking and safe shared-variable communication.
---

## How the Whole Program Works in Practice

When `main.py` starts, it performs multiple imports to reduce the chance of MicroPython memory issues. Then it configures USB serial output, motor PWM channels, motor driver pins, encoder timers, and the QTR line sensor. After that, it constructs a large collection of shared variables for wheel enable flags, gains, setpoints, measurements, bumper override state, line-follow data, script-debug values, and sensor calibration commands.

The robot then creates these major tasks:

* **Left motor task** and **right motor task** for wheel-level PI speed control
* **Line read task** to acquire and process QTR readings into a line error
* **Line follow task** to convert line error into left/right speed setpoints and manage scripted course segments
* **Bumper recovery task** to back up and turn after a collision
* **Tuning/UI task** to let the user calibrate, tune gains, stream status, and start/stop operation

Finally, the scheduler repeatedly calls the highest-priority task that is ready to run.

---
## Task Diagram

The diagram shows the final task structure used in the term project. Each box represents a scheduled task, labeled with its period and priority, and each dashed connection shows data passed between tasks.

Most of the communication is already shown directly in the diagram, so this section is only meant to briefly explain how to read it.

### Reading the Diagram

- Each task runs at a fixed rate using the cooperative scheduler.
- The labels on the arrows are the shared variables used for communication.
- `(-1)` indicates a **Share** rather than a **Queue**.

### Why the Design Uses Mostly Shares

This system uses mostly **Shares** and very few **Queues** because the tasks generally only need access to the most recent value of each variable. Since the controller is based on the robot’s current state, storing older values is usually unnecessary.

Using Shares keeps communication simpler, reduces memory use, and avoids extra buffering between tasks. This makes the system better suited for real-time control, where the newest available data is what matters most.

### Overall Behavior

The tasks work together to read sensors, compute control actions, drive the motors, handle tuning and user input, and respond to bumper-triggered recovery. Overall, the diagram represents a control system built around sharing the robot’s current state between tasks efficiently.

<img width="2035" height="1317" alt="image" src="https://github.com/user-attachments/assets/aedb0dcd-c51a-4aa5-9ecf-f027e6f3324f" />


# Core Files

## main.py

### Purpose

`main.py` is the integration file for the full robot. It creates the hardware interfaces, instantiates the shared variables, creates the scheduled tasks, and starts the cooperative scheduler.

### How it works

The file begins with repeated `gc.collect()` calls between imports. This is intentional and helps reduce startup RAM problems on MicroPython.

`main.py` then creates the main hardware objects:
- USB serial interface
- left and right motor PWM channels
- two motor driver objects
- two encoder objects
- the QTR analog reflectance sensor array
- bumper input pins

It also creates the shared variables used throughout the project. The final design uses mostly `Share` objects, since most tasks only need the newest available value instead of a history of past values.

The scheduled tasks created here are:
- left motor task
- right motor task
- line-follow task
- bumper-recovery task
- tuning UI task
- an inline `task_read_line()` generator for reading and calibrating the line sensors

The scheduler priorities match the control importance of the system. Bumper recovery is highest priority, line sensing and line following are next, motor tasks are below that, and the UI runs lowest.

### Important calculations and logic

The inline `task_read_line()` function computes the sensor-based line position using a weighted average:

- `weighted_sum += v * (i * 1000)`
- `pos = weighted_sum / total`
- `line_err = pos - center`

This converts the eight sensor readings into one position estimate and then into a centered line error for the outer controller.

### Nuances

- The line-reading task is defined directly in `main.py` instead of a separate file.
- The code creates many shares for control, debug, and scripting, including `line_err`, `line_ok`, `setpointL`, `setpointR`, `script_state`, and encoder-based count tracking shares.
- `effortL` and `effortR` are still created as shares for debug/monitoring, even though the main control structure is centered on speed setpoints rather than raw duty commands.
- The scheduler runs inside a `try/except` block so the motors can be disabled if an exception occurs.

### Reflection

`main.py` shows the final integrated architecture of the project. It ties together sensing, control, scripted behaviors, bumper recovery, and user tuning in one place.

---

## task_motor.py

### Purpose

`task_motor.py` implements the inner wheel-speed control loop for one motor. One instance is created for the left wheel and one for the right wheel.

### How it works

This task has three main states:
- `S0_INIT`
- `S1_WAIT`
- `S2_RUN`

In the run state, the task:
1. updates the encoder
2. reads wheel position and wheel speed
3. writes those measurements to shares
4. optionally resets the PI integrator
5. computes the control effort from the speed error
6. sends that effort to the motor driver

This structure makes the wheel controller an inner loop. Higher-level tasks do not command PWM directly. They command wheel speed setpoints, and `task_motor.py` handles the low-level correction.

### Important calculations

The motor controller uses PI control based on encoder speed:

- `error = setpoint - measured_speed`
- `P = Kp * error`
- `I = Ki * integral(error)`
- `u = P + I`

The code includes anti-windup behavior by only accepting the candidate integral update when the output remains inside the saturation limits.

The output effort is limited to the configured saturation range before being sent to the motor driver.

### Nuances

- The native speed unit is counts per second, since that comes directly from the encoder update.
- `pi_reset_share` allows higher-level tasks to clear the wheel PI integrators during scripted transitions or recovery maneuvers.
- The task supports optional logging queues, but the final integrated robot mainly relies on shares rather than queues.
- On disable, the task sets effort to zero and disables the motor driver.

### Reflection

This file is one of the most important control files in the project because it makes wheel response more repeatable and less sensitive to battery voltage or floor-condition changes.

---

## encoder.py

### Purpose

`encoder.py` provides the quadrature encoder interface and computes both accumulated position and wheel velocity.

### How it works

The constructor configures a timer in encoder mode using two input pins. During each `update()`, the class:
1. reads the current timer count
2. computes the change since the previous reading
3. corrects wraparound
4. accumulates total position
5. computes velocity from count change over elapsed time

<img width="1065" height="370" alt="image" src="https://github.com/user-attachments/assets/d4da1c20-4cf0-4db4-87b4-580ad4bcdfa3" />

### Important calculations

The velocity calculation is based on:

- `delta_counts / delta_time`

The wraparound correction uses the timer half-period rule so that a jump across the timer boundary is interpreted as a small forward or backward movement instead of a huge false jump.

The file also supports conversions to:
- scaled position/velocity
- radians
- raw counts

### Nuances

- The `invert` flag allows the same encoder class to work even if one side is mounted in the opposite direction.
- The class supports both logical zeroing and optional hardware-counter reset.
- This file feeds both the wheel-speed PI controller and the encoder-count-based scripted maneuvers.

### Reflection

This file is the measurement foundation of the robot. Accurate speed and position feedback are required for both closed-loop motor control and encoder-based scripted transitions.

---

## motor_driver.py

### Purpose

`motor_driver.py` is the low-level actuator interface for one motor channel.

### How it works

The class stores the PWM object and uses two digital pins:
- one direction pin
- one sleep/enable pin

`set_effort()` accepts a signed effort command, clamps it to `[-100, 100]`, sets the motor direction from the sign, and applies the corresponding PWM duty cycle.

`enable()` wakes the driver and forces zero PWM.  
`disable()` puts the driver back to sleep.

<img width="2213" height="598" alt="image" src="https://github.com/user-attachments/assets/3bc7621a-5725-4a43-afff-a41b84e620f6" />

### Important calculations

The main control mapping is:

- positive effort -> one direction
- negative effort -> opposite direction
- PWM duty cycle -> absolute value of effort

### Nuances

- Direction is handled by a digital pin, not signed PWM.
- The file stores the most recent effort command for debugging.
- The implementation is intentionally minimal so the higher-level tasks can think in terms of effort instead of pin states.

### Reflection

This file cleanly isolates the hardware actuation details from the rest of the control system.

---

## sensor_driver.py

### Purpose

`sensor_driver.py` provides the driver for the QTR-MD-08A analog line sensor array.

### How it works

The driver creates one ADC object for each sensor channel and optionally controls the emitter pin. It stores:
- raw readings
- normalized readings
- white calibration values
- black calibration values
- the last valid line position

Typical use is:
1. read raw sensor values
2. calibrate white and black references
3. normalize the readings
4. compute line position from a weighted average
5. compute line error relative to the center of the array

<img width="2054" height="373" alt="image" src="https://github.com/user-attachments/assets/dc26d375-c73b-433e-8d90-6f55c0415964" />

### Important calculations

Normalization scales each sensor channel to a usable range based on white and black calibration values.

Line position is computed using a weighted centroid:

- `weighted_sum += reading * sensor_position`
- `position = weighted_sum / total`

Then the line error is found relative to the center of the array.

### Nuances

- `fix_calibration_order()` corrects the calibration if white and black are captured in reverse order.
- If calibration is incomplete, the driver can still fall back to a rough normalization for testing.
- `min_reading` filters out weak values so noise does not distort the centroid.
- If no line is found, the driver returns the last valid position instead of a meaningless jump.

### Reflection

This file is a critical sensing layer. Reliable sensor readings are necessary before any line-following control can work well.

---

## task_follow_line.py

### Purpose

`task_follow_line.py` is the main behavior and outer-loop control file. It performs normal line following, handles scripted encoder-based maneuvers, manages line-loss behavior, and coordinates special course segments.

### How it works

This file combines:
- an outer-loop line-follow controller
- a behavior state machine
- an encoder-count-based course script

The main states in the final version include:
- `S0_IDLE`
- `S1_RUN`
- `S2_TUNE_ZONE`
- `S3_TURN_SMALL`
- `S4_FWD_1`
- `S5_TURN_90`
- `S6_FWD_2`
- `S7_LOST_STOP`
- `S8_SCRIPT_EXIT`
- `S9_POST_FWD`
- `S10_POST_TURN`
- `S11_POST_BUMP_FOLLOW`
- `S12_CP4_TURN_LEFT`
- `S13_CP4_FWD`
- `S14_FINISH_STOP`

During normal operation, the task reads:
- line error
- line-valid flag
- nominal forward speed
- line-control gains
- wheel position feedback
- bumper override state

It then writes:
- left wheel setpoint
- right wheel setpoint
- debug values such as `dv_out`
- script-state and count-tracking shares

<img width="2937" height="541" alt="image" src="https://github.com/user-attachments/assets/113e31fb-35cb-4231-ba58-750002d13a56" />


### Important calculations

The outer-loop control computes a differential correction `dv` from line error:

- `error = desired_line_position - measured_line_position`
- `P = Kp_line * error`
- `I = Ki_line * integral(error)`
- optional `D = Kd_line * derivative(error)`
- `dv = P + I (+ D if enabled)`

That correction is converted into wheel setpoints:

- `spL = v_nom - dv`
- `spR = v_nom + dv`

A small forward trim is also included to compensate for consistent drivetrain bias:

- `spL = fwd_speed - fwd_trim`
- `spR = fwd_speed + fwd_trim`

The file also tracks average encoder counts from the start of the run and from the start of each scripted segment in order to trigger maneuvers at repeatable physical locations.

### Nuances

- This file contains the most advanced control logic in the project.
- It supports both normal sensor-based following and encoder-based scripted segments.
- It includes line-loss confirmation timing so the robot does not falsely stop on brief sensor dropouts.
- The task can reset the wheel PI integrators during scripted transitions.
- `motion_override` prevents this task from fighting the bumper-recovery task for control of the wheel commands.
- The final code includes more states than the helper stage-name map in `task_user.py`, so some late-stage codes can appear as `UNKNOWN` in streamed UI output.

### Reflection

This file is the main behavior controller of the project and contains the final track-specific logic that made the full run possible.

---

## task_bumper_recovery.py

### Purpose

`task_bumper_recovery.py` monitors the bumper switches and temporarily overrides normal line-following when the robot collides with an obstacle.

### How it works

This file implements a recovery state machine with the states:
- `S0_IDLE`
- `S1_DEBOUNCE`
- `S2_BRAKE`
- `S3_REVERSE`
- `S4_TURN_LEFT`
- `S5_RESUME_DELAY`
- `S6_WAIT_RELEASE`

When a bumper hit is confirmed, the task:
1. turns on motion override
2. disables line-follow temporarily
3. commands a stop
4. resets the wheel PI integrators
5. reverses for a set encoder count
6. pivots left for a set encoder count
7. waits briefly
8. re-enables line following
9. waits for bumper release and rearm timing

<img width="2773" height="209" alt="image" src="https://github.com/user-attachments/assets/d2cc9269-2a85-4337-a4a9-faffab6fb062" />


### Important calculations

The recovery distances are measured using encoder counts, not just time. The task computes average segment counts from left and right wheel positions and compares them to the configured reverse and turn thresholds.

This makes the recovery behavior more repeatable across battery conditions and floor conditions.

### Nuances

- The task includes debounce timing, release confirmation, and rearm delay.
- It keeps the wheel-control structure active and overrides setpoints rather than bypassing the control system completely.
- The task uses the `motion_override` share so only one high-level task owns the wheel commands at a time.

### Reflection

This file is very important to the overall robustness of the project because it lets the robot recover instead of getting permanently stuck after contact.

---

## task_user.py

### Purpose

`task_user.py` implements the USB serial tuning and monitoring interface for the robot.

### How it works

This file provides the `task_tuning_ui` class. It reads serial commands from the user and can:
- print the help menu
- set wheel gains
- set nominal speed
- set line-follow gains
- arm and execute calibration
- start and stop the robot
- stream diagnostic information

It also supports a physical user-button shortcut for starting and stopping the run.

### Important calculations and logic

This file is not a control-law file, but it is still important because it exposes the tuning values used by the controllers:
- wheel `Kp` and `Ki`
- line-follow `Kp_line` and `Ki_line`
- nominal speed
- calibration commands
- run/stop enables

The streaming mode also reads and reports current values such as:
- encoder positions
- encoder speeds
- line status
- wheel setpoints
- script stage
- total counts
- segment counts

### Nuances

- Calibration must first be armed before white or black calibration is accepted.
- The task uses mini-modes such as command mode, numeric entry, and stream mode rather than only single-character immediate actions.
- The button behavior is mainly used as a convenient physical run/stop interface.
- IMU-related options still appear in the UI even though the final `main.py` does not wire in IMU shares.

### Reflection

This file was very useful during physical testing because it allowed repeated tuning and live debugging without constantly rewriting code.

---

## ui_help.py

### Purpose

`ui_help.py` prints the serial help menu used by the tuning UI.

### How it works

This file contains a helper function that writes a formatted command table to the USB serial terminal.

### Nuances

- It keeps the help text separate from `task_user.py`, which helps keep the main UI task cleaner.
- The help menu still includes some IMU-related options from earlier development stages.

### Reflection

This is a support file, but it improves usability and makes testing faster.

---

## task_share.py

### Purpose

`task_share.py` provides the inter-task communication objects used throughout the project.

### How it works

This file defines the shared-data infrastructure used by the scheduler:
- `BaseShare`
- `Queue`
- `Share`

These classes allow one task to safely pass data to another without direct function calls.

In the final robot, most communication uses `Share` objects. This matches the structure of the controller, since most tasks only need the newest available value, not a buffered history of older values.

### Important calculations and logic

`Queue` objects use a ring-buffer structure with read and write indices.  
`Share` objects store a single current value.

This distinction is important:
- use a `Share` when only the latest value matters
- use a `Queue` when ordered history must be preserved

### Nuances

- The file includes optional interrupt protection for safer updates.
- It is a foundational support file even though it does not directly implement robot behavior.
- The final design choice to rely mostly on shares helps reduce memory use and simplifies real-time control.

### Reflection

This file is important because it defines how the whole multitasking system exchanges data.

---

## cotask.py

### Purpose

`cotask.py` provides the cooperative multitasking scheduler used by the project.

### How it works

The main class in this file is `Task`. Each scheduled task is created from a generator function, assigned a priority and period, and then managed by the scheduler.

The scheduler repeatedly checks which tasks are ready to run and then advances the appropriate generator.

### Important calculations and logic

A task is considered ready based on timing and scheduling rules:
- its period has elapsed
- it is allowed to run
- it has not completed

This allows the robot to run multiple control and interface tasks in a predictable cooperative structure without preemption.

### Nuances

- This file is the reason the project can separate sensing, wheel control, user interaction, line following, and bumper recovery into different tasks.
- It also supports profiling and transition tracing features.
- Like `task_share.py`, this is a framework file rather than a robot-specific behavior file, but it is still essential to the final system.

### Reflection

This file is the scheduling backbone of the project and makes the entire multitasking architecture possible.

## Robot Performance Video

The video below shows our final ME 405 Romi robot completing the project track using closed-loop line-following control. During the run, the robot uses its reflectance sensor array to detect the line and continuously adjust its motion to navigate the course autonomously.

[![Watch the robot performance video](https://img.youtube.com/vi/Pn7XdVaAzuA/hqdefault.jpg)](https://youtube.com/shorts/Pn7XdVaAzuA)
