# ME 405 Term Project

## Project Overview

This ME 405 term project focused on the design, assembly, programming, and testing of an autonomous **Pololu Romi** mobile robot capable of reliably navigating a printed line track during an in-class **time-trial demonstration**. The project integrated mechanical assembly, electrical integration, embedded programming, sensor interfacing, and closed-loop control into a complete mechatronic system.

The final robot was built around an **STM32 Nucleo-L476RG** microcontroller mounted on a **Shoe of Brian** interface board and installed on a Romi chassis with motors, encoders, and onboard battery power. To perform successfully, the robot needed to detect the line, estimate its motion, respond to changing track conditions, and complete the course as quickly and consistently as possible.

---

## Project Goals

The main goals of the project were to:

- build a fully functional Romi-based autonomous robot
- integrate sensors and actuators into a unified embedded control system
- implement reliable **line-following behavior**
- use feedback control to regulate wheel motion and steering
- improve performance in terms of **speed, repeatability, and robustness**
- document the design process, hardware setup, software architecture, and project results in a professional portfolio

---

## System Description

The robot uses a **differential-drive** configuration with two independently driven wheels and wheel encoders for motion feedback. A **QTR reflectance sensor array** detects the position of the printed line beneath the robot, allowing the control system to compute line error and steer back toward the center of the track. Depending on the final configuration, an **IMU** may also be used for orientation feedback, and bumper switches may be included for contact detection.

The overall system combines:

- **mechanical integration** of the Romi hardware stack
- **electrical integration** of motors, encoders, sensors, and power wiring
- **embedded driver development** for the hardware devices
- **task-based embedded software** running in MicroPython
- **closed-loop motor and steering control**
- **testing and tuning** for reliable time-trial performance

---

## Control Approach

The robot’s control system is based on continuous sensor feedback.

At a high level, the system:

1. reads the reflectance sensors to determine line position
2. computes a line-following error relative to the center of the robot
3. adjusts wheel commands to steer toward the line
4. uses motor and encoder feedback to regulate wheel speed
5. repeats this process continuously while driving the track

This creates a closed-loop system in which the robot can correct its path in real time rather than simply following a fixed command sequence.

---

## Major Hardware Used

The final hardware configuration includes:

- **Pololu Romi chassis**
- **STM32 Nucleo-L476RG**
- **Shoe of Brian** interface board
- **DC motors and quadrature encoders**
- **QTR-MD-08A reflectance sensor array**
- **BNO055/BNO085 IMU**, depending on project configuration
- **bumper switches** for contact sensing
- custom power, motor, encoder, and sensor wiring

---

## Major Software Features

The project software includes:

- motor driver control
- encoder feedback processing
- reflectance sensor reading and calibration
- line position estimation
- closed-loop speed control
- closed-loop steering and line-following control
- cooperative multitasking
- serial user interface for testing and tuning
- optional state estimation and IMU integration

---

## Design Challenges

This project required balancing several competing design goals:

- **speed vs. stability**
- **aggressive steering vs. smooth tracking**
- **sensor sensitivity vs. noise**
- **fast lap times vs. repeatable performance**

In practice, successful performance depended not only on speed, but also on reliability. Small issues in wiring, calibration, sensor placement, control gains, or task timing could significantly affect performance on the track.

---

## Final Deliverables

The project concluded with two major deliverables:

1. an **in-class robot demonstration and time trial**, where the design was presented and the robot was run on the printed course
2. a **project portfolio/repository** containing the source code, hardware documentation, design files, and a written explanation of the project, its challenges, and its results

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

This project brings together the major topics of ME 405 into one complete robotic system. The final result is an autonomous Romi robot that uses embedded hardware, sensor feedback, and closed-loop control to follow a track and compete in a time-trial event. The project demonstrates the integration of mechatronics concepts including system assembly, driver development, multitasking control, sensing, estimation, and real-world performance tuning.

The repository also contains `.mpy` files, which are compiled versions of MicroPython `.py` files. These files are precompiled bytecode used to reduce memory usage, improve execution speed, and avoid runtime compilation overhead.

# Game Track Overview

The game track is a multi-section course designed to test the Romi robot’s ability to follow lines, handle curves, recognize intersections, and navigate between multiple checkpoints. The course includes both high-speed sections and precision-control sections, requiring the robot to balance speed with accurate tracking.

The robot begins in the **start circle at CP#0 / CP#5** in the upper-left corner of the map. From there, the main route includes a **long straightaway across the top of the course**, followed by a **large right-hand turn** near **CP#1**. This section is useful for evaluating straight-line stability and high-speed line-following performance.

The left and lower portions of the course include **large-radius curves** and a **wavy S-curve section** leading toward **CP#3** and **CP#2**. These sections test how well the robot can maintain line position through changing curvature and transition smoothly between turns.

Near the center of the map, the course includes a **branching or dashed path near CP#4**, which appears to represent a secondary route or special navigation segment. This section can be used to test alternate routing behavior, decision-making, or transitions between different course features.

Near the right side of the map, the course also includes a **wall region**, a **grid-like zone**, and a **cross intersection** near **CP#2**. These features add complexity beyond basic line following.

Additional **circular marked zones** are placed throughout the course and may be used for calibration, detection tasks, or checkpoint-based actions. The **IR calibration corner** in the upper-right corner is specifically intended for reflectance sensor setup and calibration before or during testing.

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

As detailed in **HARDWARE SETUP.md**, **HARDWARE DESCRIPTION.md**, and the project **PINOUT.xlsx**, the final robot configuration was assembled as a fully integrated Romi-based autonomous platform for line following and time-trial navigation. The system used a **Pololu Romi chassis** with two independently driven wheels, wheel encoders for motion feedback, an **STM32 Nucleo-L476RG** mounted on a **Shoe of Brian**, and a front-mounted reflectance sensor array for detecting and tracking the course line.

The final hardware arrangement combined the controller stack, power system, sensor wiring, and drivetrain hardware into a compact, organized, and testable layout. The Nucleo and interface board were mounted above the chassis, wiring was routed to the motors and encoders through the hardware stack, and the front sensor assembly was positioned to provide reliable line readings during motion. This configuration was used throughout testing, tuning, and the final project demonstration.

The image below shows the completed robot assembly used for the final project.

![IMG_2487](https://github.com/user-attachments/assets/a105b7a0-475f-4500-81f6-4237cec445a9)

# System Architecture

The software is organized into five main layers:

1. **Hardware interface**  
   `motor_driver.py`, `encoder.py`, and `sensor_driver.py` interface directly with the motor driver, quadrature encoders, and QTR analog reflectance sensor array.

2. **Wheel control**  
   `task_motor.py` converts wheel speed setpoints into PWM effort using a PI controller for each motor.

3. **Behavior layer**  
   `task_follow_line.py` computes differential wheel setpoints from line error and executes scripted encoder-based segments when needed.  
   `task_bumper_recovery.py` temporarily overrides normal motion when bumper contact is detected.

4. **User interaction layer**  
   `task_user.py` and `ui_help.py` provide serial commands, live status streaming, and calibration controls.

5. **Scheduling infrastructure**  
   `cotask.py` and `task_share.py` provide cooperative multitasking and shared-variable communication between tasks.

---

## How the Program Operates

When `main.py` starts, it performs several imports in a staged manner to reduce the risk of MicroPython memory issues. It then configures the USB serial interface, motor PWM channels, motor driver pins, encoder timers, and the QTR line sensor. After initialization, it creates the shared variables used throughout the system, including wheel enable flags, controller gains, speed setpoints, measured feedback values, bumper override state, line-following data, debug values, and sensor calibration commands.

The program then creates the major scheduled tasks:

- **Left motor task** and **right motor task** for wheel-level PI speed control
- **Line read task** to acquire and process QTR sensor readings into a line error value
- **Line follow task** to convert line error into left and right wheel setpoints and manage scripted course segments
- **Bumper recovery task** to reverse and turn after a collision
- **Tuning/UI task** to support calibration, gain tuning, status streaming, and run control

Once the tasks are created, the scheduler repeatedly runs the highest-priority task that is ready to execute.

---

## Task Diagram

The diagram shows the final task structure used in the project. Each box represents a scheduled task labeled by its priority and execution period, and each dashed connection represents data passed between tasks.

Most of the communication is already shown directly in the diagram, so this section only provides a brief explanation of how to interpret it.

### Reading the Diagram

- Each task runs at a fixed rate under the cooperative scheduler.
- Arrow labels indicate the shared variables used for communication.
- `(-1)` indicates a **Share** rather than a **Queue**.

### Why the Design Uses Mostly Shares

This system relies primarily on **Shares** and only a few **Queues** because most tasks only need access to the most recent value of a variable. Since the controller operates on the robot’s current state, storing older values is usually unnecessary.

Using Shares simplifies communication, reduces memory usage, and avoids unnecessary buffering between tasks. This makes the architecture better suited for real-time embedded control, where the newest available data is typically the most important.

### Overall Behavior

Together, the tasks handle sensing, control, actuation, tuning, and recovery behavior. The overall architecture is built around efficiently sharing the robot’s current state between tasks so that control decisions can be updated reliably in real time.

<img width="2035" height="1317" alt="image" src="https://github.com/user-attachments/assets/aedb0dcd-c51a-4aa5-9ecf-f027e6f3324f" />

# Core Files

## `main.py`

### Purpose

`main.py` is the top-level integration file for the robot. It initializes the hardware interfaces, creates the shared variables, instantiates the scheduled tasks, and starts the cooperative scheduler.

### How It Works

The file begins with repeated `gc.collect()` calls between imports. This is intentional and helps reduce startup memory issues in MicroPython.

After initialization, `main.py` creates the primary hardware objects:

- USB serial interface
- left and right motor PWM channels
- two motor driver objects
- two encoder objects
- QTR analog reflectance sensor array
- bumper input pins

It also creates the shared variables used throughout the system. The final design relies mostly on `Share` objects, since most tasks only need the most recent value rather than a history of past data.

The scheduled tasks created in `main.py` include:

- left motor task
- right motor task
- line-follow task
- bumper-recovery task
- tuning/UI task
- an inline `task_read_line()` generator for reading and calibrating the line sensors

Task priorities are assigned based on control importance. Bumper recovery runs at the highest priority, followed by line sensing and line following, then the motor-control tasks, with the user interface running at the lowest priority.

### Important Calculations and Logic

The inline `task_read_line()` function computes line position from the sensor array using a weighted average:

- `weighted_sum += v * (i * 1000)`
- `pos = weighted_sum / total`
- `line_err = pos - center`

This converts the eight sensor readings into a single position estimate and then into a centered line error used by the outer line-following controller.

### Notable Details

- The line-reading task is defined directly in `main.py` rather than placed in a separate file.
- The program creates many shared variables for control, debugging, and scripting, including `line_err`, `line_ok`, `setpointL`, `setpointR`, `script_state`, and encoder-based count-tracking values.
- `effortL` and `effortR` are still maintained as shares for monitoring and debugging, even though the primary control structure is based on wheel speed setpoints rather than raw duty commands.
- The scheduler runs inside a `try/except` block so the motors can be safely disabled if an exception occurs.

### Summary

`main.py` represents the final integrated architecture of the project. It brings together sensing, control, scripted behaviors, bumper recovery, and user interaction in a single top-level program.

---

## `task_motor.py`

### Purpose

`task_motor.py` implements the inner wheel-speed control loop for a single motor. One instance is used for the left wheel and one for the right wheel.

### How It Works

This task is organized around three main states:

- `S0_INIT`
- `S1_WAIT`
- `S2_RUN`

In the run state, the task:

1. updates the encoder
2. reads wheel position and wheel speed
3. writes measured values to shared variables
4. optionally resets the PI integrator
5. computes the control effort from the speed error
6. sends the resulting effort command to the motor driver

This makes `task_motor.py` the inner control loop of the system. Higher-level tasks do not command PWM directly. Instead, they provide wheel speed setpoints, and `task_motor.py` handles the low-level feedback correction needed to achieve them.

<img width="981" height="246" alt="image" src="https://github.com/user-attachments/assets/c849bd0b-69b9-4377-8f2c-ea4b69791164" />

### Important Calculations

The motor controller uses PI control based on encoder-measured wheel speed:

- `error = setpoint - measured_speed`
- `P = Kp * error`
- `I = Ki * integral(error)`
- `u = P + I`

The implementation includes anti-windup behavior by only accepting the candidate integral update when the controller output remains within the saturation limits.

Before the command is sent to the motor driver, the output effort is clamped to the configured saturation range.

### Notable Details

- The native speed unit is counts per second, since that value is computed directly from the encoder update.
- `pi_reset_share` allows higher-level tasks to reset the wheel PI integrators during scripted transitions or recovery maneuvers.
- The task supports optional logging queues, although the final integrated robot primarily relies on shares rather than queues.
- When disabled, the task sets motor effort to zero and disables the motor driver.

### Summary

This file is one of the most important control components in the project because it makes wheel response more repeatable and less sensitive to changes in battery voltage or floor conditions.

---

## `encoder.py`

### Purpose

`encoder.py` provides the quadrature encoder interface and computes both accumulated wheel position and wheel velocity.

### How It Works

The constructor configures a timer in encoder mode using two input pins. During each `update()`, the class:

1. reads the current timer count
2. computes the change since the previous reading
3. corrects for wraparound
4. accumulates total position
5. computes velocity from count change over elapsed time

<img width="1065" height="370" alt="image" src="https://github.com/user-attachments/assets/d4da1c20-4cf0-4db4-87b4-580ad4bcdfa3" />

### Important Calculations

Wheel velocity is computed from the change in encoder counts over elapsed time:

- `delta_counts / delta_time`

Wraparound correction uses the timer half-period rule so that a count jump across the timer boundary is interpreted as a small forward or backward movement rather than a large false jump.

The file also supports conversions to:

- scaled position and velocity
- radians
- raw counts

### Notable Details

- The `invert` flag allows the same encoder class to be used even when one encoder is mounted in the opposite direction.
- The class supports both logical zeroing and optional hardware counter reset.
- This file provides feedback for both the wheel-speed PI controller and the encoder-based scripted maneuvers.

### Summary

This file is the measurement foundation of the robot. Accurate speed and position feedback are essential for both closed-loop motor control and encoder-based motion sequences.

---

## `motor_driver.py`

### Purpose

`motor_driver.py` is the low-level actuator interface for a single motor channel.

### How It Works

The class stores the PWM object and uses two digital pins:

- one direction pin
- one sleep/enable pin

`set_effort()` accepts a signed effort command, clamps it to `[-100, 100]`, determines motor direction from the sign, and applies the corresponding PWM duty cycle.

`enable()` wakes the driver and forces zero PWM.  
`disable()` puts the driver back to sleep.

<img width="2213" height="598" alt="image" src="https://github.com/user-attachments/assets/3bc7621a-5725-4a43-afff-a41b84e620f6" />

### Important Calculations

The main control mapping is straightforward:

- positive effort -> one motor direction
- negative effort -> the opposite motor direction
- PWM duty cycle -> absolute value of the commanded effort

### Notable Details

- Motor direction is controlled by a digital pin rather than signed PWM.
- The file stores the most recent effort command for debugging and monitoring.
- The implementation is intentionally minimal so higher-level tasks can work in terms of effort commands rather than low-level pin states.

### Summary

This file cleanly separates low-level motor actuation from the rest of the control system.

---

## `sensor_driver.py`

### Purpose

`sensor_driver.py` provides the driver for the QTR-MD-08A analog reflectance sensor array.

### How It Works

The driver creates one ADC object for each sensor channel and optionally controls the emitter pin. It stores:

- raw readings
- normalized readings
- white calibration values
- black calibration values
- the last valid line position

Typical operation follows these steps:

1. read raw sensor values
2. capture white and black calibration references
3. normalize the sensor readings
4. compute line position using a weighted average
5. compute line error relative to the center of the array

<img width="2054" height="373" alt="image" src="https://github.com/user-attachments/assets/dc26d375-c73b-433e-8d90-6f55c0415964" />

### Important Calculations

Normalization scales each sensor channel into a usable range based on the recorded white and black calibration values.

Line position is computed using a weighted centroid:

- `weighted_sum += reading * sensor_position`
- `position = weighted_sum / total`

The line error is then computed relative to the center of the sensor array.

### Notable Details

- `fix_calibration_order()` corrects the calibration if white and black references are captured in reverse order.
- If calibration is incomplete, the driver can still fall back to a rough normalization method for testing.
- `min_reading` filters out weak sensor values so noise does not distort the centroid calculation.
- If no line is detected, the driver returns the last valid position instead of producing a meaningless jump.

### Summary

This file is a critical sensing layer in the project. Reliable sensor readings are necessary before any line-following control can perform well.

---

## `task_follow_line.py`

### Purpose

`task_follow_line.py` is the main behavior and outer-loop control file. It handles normal line following, scripted encoder-based maneuvers, line-loss behavior, and special course segments.

### How It Works

This file combines:

- an outer-loop line-following controller
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

### Important Calculations

The outer-loop controller computes a differential correction `dv` from line error:

- `error = desired_line_position - measured_line_position`
- `P = Kp_line * error`
- `I = Ki_line * integral(error)`
- optional `D = Kd_line * derivative(error)`
- `dv = P + I (+ D if enabled)`

That correction is then converted into wheel speed setpoints:

- `spL = v_nom - dv`
- `spR = v_nom + dv`

A small forward trim is also applied to compensate for consistent drivetrain bias:

- `spL = fwd_speed - fwd_trim`
- `spR = fwd_speed + fwd_trim`

The file also tracks average encoder counts from the start of the run and from the start of each scripted segment so that maneuvers can be triggered at repeatable physical locations on the course.

### Notable Details

- This file contains the most advanced control logic in the project.
- It supports both normal sensor-based line following and encoder-based scripted segments.
- It includes line-loss confirmation timing so the robot does not falsely stop on brief sensor dropouts.
- The task can reset the wheel PI integrators during scripted transitions.
- `motion_override` prevents this task from conflicting with `task_bumper_recovery.py` for control of the wheel commands.
- The final code includes more states than the helper stage-name map in `task_user.py`, so some later-stage codes may appear as `UNKNOWN` in the streamed UI output.

### Summary

This file is the main behavior controller of the project and contains the track-specific logic that made the full run possible.

---

## `task_bumper_recovery.py`

### Purpose

`task_bumper_recovery.py` monitors the bumper switches and temporarily overrides normal line-following behavior when the robot collides with an obstacle.

### How It Works

This file implements a recovery state machine with the following states:

- `S0_IDLE`
- `S1_DEBOUNCE`
- `S2_BRAKE`
- `S3_REVERSE`
- `S4_TURN_LEFT`
- `S5_RESUME_DELAY`
- `S6_WAIT_RELEASE`

When a bumper hit is confirmed, the task:

1. enables motion override
2. temporarily disables line following
3. commands the robot to stop
4. resets the wheel PI integrators
5. reverses for a fixed encoder count
6. pivots left for a fixed encoder count
7. waits briefly before resuming
8. re-enables line following
9. waits for bumper release and rearm timing

<img width="2773" height="209" alt="image" src="https://github.com/user-attachments/assets/d2cc9269-2a85-4337-a4a9-faffab6fb062" />

### Important Calculations

Recovery distances are measured using encoder counts rather than time alone. The task computes average segment counts from the left and right wheel positions and compares them to the configured reverse and turn thresholds.

Using encoder-based distance thresholds makes the recovery behavior more repeatable across changes in battery condition and floor surface.

### Notable Details

- The task includes debounce timing, release confirmation, and rearm delay.
- It keeps the wheel-control structure active and overrides wheel setpoints rather than bypassing the control system entirely.
- The task uses the `motion_override` share so that only one high-level task controls the wheel commands at a time.

### Summary

This file is an important part of the robot’s overall robustness because it allows the system to recover from contact instead of becoming permanently stuck.

---

## `task_user.py`

### Purpose

`task_user.py` implements the USB serial tuning and monitoring interface for the robot.

### How It Works

This file provides the `task_tuning_ui` class. It reads serial commands from the user and can:

- print the help menu
- set wheel-control gains
- set nominal speed
- set line-following gains
- arm and execute sensor calibration
- start and stop the robot
- stream diagnostic information

It also supports a physical user-button shortcut for starting and stopping operation.

<img width="1122" height="774" alt="image" src="https://github.com/user-attachments/assets/df936f88-3d10-4403-b32f-205d104bfec2" />

### Important Calculations and Logic

Although this file does not implement a control law directly, it is still an important part of the system because it exposes the tuning parameters used by the controllers, including:

- wheel `Kp` and `Ki`
- line-following `Kp_line` and `Ki_line`
- nominal speed
- calibration commands
- run/stop enables

The streaming mode also reads and reports current system values such as:

- encoder positions
- encoder speeds
- line status
- wheel setpoints
- script stage
- total counts
- segment counts

### Notable Details

- Calibration must first be armed before white or black calibration is accepted.
- The task uses multiple operating modes, including command mode, numeric-entry mode, and stream mode, rather than relying only on single-character immediate actions.
- The button behavior is mainly used as a convenient physical run/stop interface.
- IMU-related options still appear in the UI even though the final `main.py` does not connect IMU shares.

### Summary

This file was especially useful during physical testing because it allowed repeated tuning and live debugging without requiring constant code changes.

---

## `task_read_line`

### Purpose

`task_read_line` is the line-sensor task responsible for reading the QTR reflectance array, handling calibration commands, and computing the current line-tracking error.

### How It Works

This task is implemented as an inline generator in `main.py` rather than as a separate file. It runs continuously and performs three main functions:

- checks whether a calibration command has been requested
- reads and normalizes sensor values
- computes line position and line error

When calibration is requested, the task performs either white or black calibration, stores the updated calibration values, and marks the calibration step as complete.

During normal operation, the task reads the normalized sensor values and uses a weighted average to estimate line position across the array. That position is then shifted relative to the center of the sensor array to produce `line_err`, which is used by the outer line-following controller.

If the line is temporarily lost, the task keeps the most recent valid error value and sets `line_ok = 0` so the rest of the system can detect that the line is no longer being seen.

<img width="1237" height="593" alt="image" src="https://github.com/user-attachments/assets/bd0dde03-96a4-4c2e-9c65-9b0591dfba0c" />

### Important Calculations

The task computes line position using a weighted centroid of the normalized sensor readings:

- `weighted_sum += v * (i * 1000)`
- `total += v`
- `pos = weighted_sum // total`

The line error is then computed by subtracting the center of the sensor array:

- `center = (qtr.count - 1) * 1000 // 2`
- `line_err = pos - center`

Only readings above the sensor threshold are included, which helps reduce the influence of noise.

### Notable Details

- This task is defined inside `main.py`, so it may not appear as a separate file in the repository.
- It retains the most recent valid line error when the line is lost instead of forcing the error to zero.
- It publishes both `line_err` and `line_ok`, allowing the controller to distinguish between a valid reading and a lost-line condition.
- Calibration is handled through the shared variables `cal_cmd` and `cal_done`.

### Summary

This task is the sensing front end of the line-following system. It converts raw reflectance data into a clean line error signal that the rest of the controller can use.

---

## `task_share.py`

### Purpose

`task_share.py` provides the inter-task communication objects used throughout the project.

### How It Works

This file defines the shared-data infrastructure used by the scheduler:

- `BaseShare`
- `Queue`
- `Share`

These classes allow one task to safely pass data to another without direct function calls.

In the final robot, most communication uses `Share` objects. This matches the structure of the controller, since most tasks only need the newest available value rather than a buffered history of older values.

### Important Calculations and Logic

`Queue` objects use a ring-buffer structure with read and write indices.  
`Share` objects store a single current value.

This distinction is important:

- use a `Share` when only the latest value matters
- use a `Queue` when ordered history must be preserved

### Notable Details

- The file includes optional interrupt protection for safer updates.
- It is a foundational support file even though it does not directly implement robot behavior.
- The design choice to rely primarily on shares helps reduce memory usage and simplifies real-time control.

### Summary

This file is important because it defines how the multitasking system exchanges data between tasks.

---

## `cotask.py`

### Purpose

`cotask.py` provides the cooperative multitasking scheduler used by the project.

### How It Works

The main class in this file is `Task`. Each scheduled task is created from a generator function, assigned a priority and period, and then managed by the scheduler.

The scheduler repeatedly checks which tasks are ready to run and advances the appropriate generator.

### Important Calculations and Logic

A task is considered ready to run based on timing and scheduling rules:

- its period has elapsed
- it is allowed to run
- it has not completed

This structure allows the robot to run multiple control and interface tasks in a predictable cooperative framework without preemption.

### Notable Details

- This file is what makes it possible to separate sensing, wheel control, user interaction, line following, and bumper recovery into distinct tasks.
- It also supports profiling and transition-tracing features.
- Like `task_share.py`, this is a framework file rather than a robot-specific behavior file, but it remains essential to the final system.

### Summary

This file is the scheduling backbone of the project and makes the full multitasking architecture possible.

# Robot Performance Video

The video below shows the final ME 405 Romi robot completing the project track using closed-loop line-following control. During the run, the robot uses its reflectance sensor array to detect the line and continuously adjust its motion to navigate the course autonomously.

[![Watch the robot performance video](https://img.youtube.com/vi/Pn7XdVaAzuA/hqdefault.jpg)](https://youtube.com/shorts/Pn7XdVaAzuA)
