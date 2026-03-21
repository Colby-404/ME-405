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

Near the right side of the map, the course also includes a **wall region**, a **grid-like zone**, and a **cross intersection** near **CP#2**. These features add complexity beyond simple line following and may require the robot to slow down, re-center itself, or execute special logic depending on the competition rules.

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

As detailed in **HARDWARE SETUP.md**, **HARDWARE DESCRIPTION.md**, and the project **pinout diagram**, the final term project robot was assembled as a fully integrated Romi-based autonomous robot for line following and time-trial navigation. The platform used a **Pololu Romi chassis** with two independently driven wheels, wheel encoders for motion feedback, an **STM32 Nucleo-L476RG** mounted on a **Shoe of Brian**, and a front-mounted reflectance sensor array for detecting and tracking the course line.

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

## Core Files

## main.py

### Purpose

`main.py` is the integration point for the entire project. It is where hardware is instantiated, shares are created, tasks are configured, and the scheduler is launched.

### How it works

The file begins with deliberate `gc.collect()` calls between imports. That ordering is not cosmetic. It is a direct response to the realities of MicroPython on embedded hardware, where import-time RAM usage can become a source of random startup failures. This is one of the clearest signs that the code was tested on the actual target hardware rather than only reasoned about abstractly.

After importing modules, `main.py` creates:

* a USB serial interface for debugging and command input,
* PWM channels on timer 4 for the left and right motors,
* two motor driver objects,
* two encoder objects,
* the QTR analog sensor array,
* bumper pin assignments,
* many shared variables used to connect tasks.

It also sets initial values for gains, nominal speed, enable flags, debug values, and calibration commands. Then it instantiates the motor tasks, the line-reading generator task, the line-follow task, the bumper-recovery task, and the UI task.

The scheduler configuration is also important. The bumper task is given the highest priority among the listed tasks, the line-reading and line-follow tasks are next, the motor tasks are slightly lower, and the UI is lowest

This file is centered on shared-state coordination instead of direct function calls. It also shows that wheel control, line following, and bumper recovery are treated as separate but cooperating processes.

### Nuances

* The line-reading task is written directly inside `main.py` rather than being split into its own file. That makes `main.py` slightly heavier, but it also keeps all sensor-calibration behavior visible in one place.
* The initial gains and script thresholds are not generic defaults; they are clearly tuned for a specific robot and course.
* The `try/except` wrapper around the scheduler disables motors on exceptions and attempts limited recovery. 

### Reflection

`main.py` demonstrates that our procedure was iterative and hardware-driven. The many constants for trigger counts and speeds suggest repeated track testing, followed by manual adjustment. The result captures the final integration of sensing, control, calibration, and competition-specific scripting. In the future, if given more time, it may have been ideal to use a robust state-space estimator to run the script rather than the less precise encoder ticks. 

**See also:** [task_follow_line.py](#task_follow_linepy), [task_motor.py](#task_motorpy), [task_bumper_recovery.py](#task_bumper_recoverypy)

---

## task_motor.py

### Purpose

`task_motor.py` implements the inner wheel-speed control loop for one motor. Each wheel gets its own instance of this class.

### How it works

The task has three states:

* `S0_INIT`: disable motor, zero encoder, initialize measurements
* `S1_WAIT`: wait until the enable share turns on
* `S2_RUN`: continuously update encoder measurements and drive the motor

In the run state, the task:

1. updates the encoder,
2. reads wheel velocity and position,
3. writes those measurements to shares,
4. optionally resets the PI integrator if commanded,
5. computes the PI control effort from the current setpoint and measured speed,
6. sends that effort to the motor driver.

The PI implementation includes anti-windup by only integrating when the candidate output remains inside saturation bounds. This is a simple but effective way to avoid integral runaway.

This task is the foundation of the robot’s motion quality. The line-follow controller does not directly force PWM values. Instead, it requests left and right wheel speeds, and `task_motor.py` tries to make the wheels achieve them. This separation means outer-loop behavior can be tuned somewhat independently of motor electrical behavior and wheel friction.

### Nuances

* The controller uses counts per second as the native speed unit because that is what the encoder naturally provides.
* There is support for logging velocity and timestamps into queues, but logging is disabled gracefully if queues fill. The code prefers “stop logging” over “stop the motor,” which is the correct priority for a real-time robot.
* On disable, the task requires zero effort and explicitly disables the motor driver, which reduces the chance of residual actuation.
* `pi_reset_share` lets higher-level tasks clear the wheel PI integrators when scripted maneuvers or sharp transitions occur.

### Reflection

The procedure implied here is that wheel controllers were tuned first or at least treated as a stable inner loop before advanced line-follow behavior was layered on top. The resulting system should be more predictable because changes in the line error map to setpoint differences, not raw PWM jumps.

A likely result of this design is smoother line tracking and more reproducible scripted turns. Without this inner loop, the same PWM command could produce different motion as battery voltage or floor friction changes. In the future, though, this may need to include a voltage regulator so that different batteries' power does not affect the system/script. 

**See also:** [encoder.py](#encoderpy), [motor_driver.py](#motor_driverpy), [task_follow_line.py](#task_follow_linepy)

---

## encoder.py

### Purpose

`encoder.py` provides a hardware interface for quadrature encoders and computes both position and velocity.

### How it works

The constructor configures a timer in encoder mode using two pins. During each `update()`, the class:

1. measures elapsed time since the previous update,
2. reads the current timer count,
3. computes the raw count difference,
4. corrects wraparound using the half-period method,
5. accumulates total position,
6. stores the current count for next time.

Velocity is then computed as count delta divided by elapsed time. The class also offers convenient conversions:

* scaled position/velocity,
* radians-based position/velocity,
* raw counts.

This class is small, but it is one of the most technically important pieces of the codebase. A wheel controller is only as good as its measurement quality. The wraparound compensation logic is what makes continuous position tracking possible, even though the hardware timer itself is bounded.

### Nuances

* The `invert` flag allows the same logic to be used even if left and right encoder directions are mounted differently.
* The counts-to-scaled-unit and counts-to-radian constants let the same measurement be interpreted in multiple physical units without reworking the rest of the code.
* `zero(reset_hw_counter=False)` can either logically zero the software position or attempt to reset the hardware timer as well.

### Reflection

This code reconstructs a continuous position by tracking deltas across wraparound, as it should be set up for embedded quadrature measurement.

The result is a measurement layer suitable for both feedback control and motion scripting. In this project, those two uses coexist: velocity feeds the PI loop, while accumulated position enables distance-triggered transitions in the line-follow and bumper-recovery scripts.

**See also:** [task_motor.py](#task_motorpy), [main.py](#mainpy)

---

## motor_driver.py

### Purpose

`motor_driver.py` is the low-level actuator interface for one motor driver channel.

### How it works

The class stores the PWM object and configures two digital pins:

* one for direction,
* one for the sleep/enable line.

`set_effort()` clamps the requested effort to the range `[-100, 100]`. Positive effort drives one direction, negative effort flips the direction pin and uses the absolute duty cycle. `enable()` wakes the driver and zeros PWM, while `disable()` puts the driver to sleep.

### Nuances

* Direction is set by a digital pin rather than signed PWM.
* The code stores the last effort command, which can be useful for debugging.
* `enable()` sets duty to zero immediately, which reduces the chance of waking into a stale nonzero command.

### Reflection

The procedure here is to isolate hardware details so the rest of the code can think in terms of “effort” rather than pin toggles and PWM channels. The result is cleaner control code upstream. Nothing to change in this file, as it is perfectly minimal. 

**See also:** [task_motor.py](#task_motorpy)

---

## sensor_driver.py

### Purpose

`sensor_driver.py` implements support for the QTR-MD-08A analog reflectance sensor array.

### How it works

The driver creates one ADC object per sensor pin and optionally controls an emitter pin. It stores:

* raw ADC readings,
* normalized values,
* white calibration values,
* black calibration values,
* the last computed line position.

The typical flow is:

1. read raw values, optionally with oversampling,
2. calibrate white and black references,
3. normalize each channel to a `0..1000` scale,
4. compute a weighted average position from all active sensors,
5. derive line error relative to the center of the array.

### Nuances

* The code supports calibration even if white and black are acquired in the “wrong” order; `fix_calibration_order()` corrects that.
* If calibration is not yet complete, the driver falls back to a crude normalization using ADC full scale. That is useful during bring-up and debugging.
* `min_reading` acts as a threshold so weak sensor values do not distort the weighted centroid.
* `read_line_position()` returns the last valid position if no line is found, which avoids abrupt nonsense outputs.

### Reflection

Ensuring good values from our line sensor was an issue that needed to be fixed early on. Without solid, good, and proper mounting of the line sensor, the rest of the code is pretty much useless. This, alongside having a robust state estiamtor are porbably most important foundational code for this project.

**See also:** [task_follow_line.py](#task_follow_linepy), [main.py](#mainpy)

---

## task_follow_line.py

### Purpose

`task_follow_line.py` is the main behavior controller. It performs line following during normal operation, handles loss-of-line behavior, schedules gain changes, and executes encoder-based scripted maneuvers for special course segments like the garage and the sharp turns. 

### How it works

This is the most sophisticated file in the project. Conceptually, it combines three roles:

1. **Outer-loop line controller**
   It reads line error, applies PI/PID-style logic, and outputs differential speed commands for the wheels.

2. **Behavior state machine**
   It tracks states such as idle, run, tune zone, lost stop, scripted turns, post-bump follow, checkpoint-4 turn, and finish stop.

3. **Course-progress manager**
   It uses encoder counts to detect when the robot has reached specific regions of the course and then changes gains or behavior accordingly.

In normal follow mode, the task computes a correction `dv` from line error and then produces wheel setpoints of approximately:

* `spL = v_nom - dv`
* `spR = v_nom + dv`

It clamps those setpoints to allowed bounds and writes them to the motor tasks.

The file also contains logic for:

* tune-zone gain scheduling,
* scripted recovery segments triggered after certain count thresholds,
* line-loss confirmation delays to avoid false detection,
* post-bumper scripted motion,
* a checkpoint-4 left turn followed by a final forward drive.

This file is effectively what runs our term project. The code intentionally switches to scripted motion in places where a known course segment may be easier to traverse by encoder counts than by continuously interpreting sensor data.

### Nuances

* The task has many optional features, including derivative action, dynamic saturation, centroid logging, alternative sensor modes, yaw-based straightening, and gain scheduling. Not all are used in `main.py`, but their presence shows the file evolved through experimentation.
* There is a small `fwd_trim` bias that intentionally slows one side and speeds the other during straight motion. That compensates for systematic drift.
* The line-loss logic uses confirmation times rather than immediately declaring failure. This is critical because sensor dropouts may occur momentarily during transitions.
* The task resets the wheel PI integrator at certain scripted transitions, which prevents old control history from contaminating a new maneuver.
* `motion_override` provides a clean handshake with the bumper task so two high-level tasks do not fight over wheel setpoints.

### Reflection

This file was the most important and worked on file of our project. The current code works reliably, but ideally, the encoder values for the script could always be further improved.

**See also:** [sensor_driver.py](#sensor_driverpy), [task_bumper_recovery.py](#task_bumper_recoverypy), [main.py](#mainpy)

---

## task_bumper_recovery.py

### Purpose

`task_bumper_recovery.py` monitors six bumper sensors and temporarily takes control of the robot when contact is detected.

### How it works

The class implements a state machine:

* idle
* debounce
* brake
* reverse
* turn left
* resume delay
* wait release

When a bumper hit is confirmed, the task:

1. turns on motion override,
2. disables line follow,
3. keeps wheel tasks enabled,
4. commands a stop,
5. resets wheel PI,
6. reverses for a specified number of encoder ticks,
7. pivots left for a specified number of ticks,
8. re-enables line follow after a short delay,
9. waits for bumper release and rearm timing.

This task protects the robot from staying stuck after contact. More importantly, it is architected so that bumper recovery is not a separate, disconnected emergency script. It is integrated into the rest of the control stack through shared variables and override logic.

### Nuances

* Debouncing, release confirmation, and rearm delay are all included. That reduces repeated triggers and noisy transitions.
* Recovery distances are measured in encoder ticks, which makes them more repeatable than open-loop timing alone.
* The task does not disable the wheel-control architecture; it simply changes setpoints and uses override ownership.

### Reflection

The complete bump sensor array was probably the most worthwhile purchase for our robot. As seen in our physical demonstrations, the bump sensor recovery task saved our robot from getting stuck in the garage. We would highly recommend complete bump sensors for anyone else attempting this lab, as this autonomy helped to save our robot when the script failed. 

**See also:** [task_follow_line.py](#task_follow_linepy), [main.py](#mainpy)

---

## task_user.py

### Purpose

`task_user.py` implements the serial and button-based tuning interface.

### How it works

This task listens for USB serial commands and optionally for the on-board user button. It can:

* print help,
* set wheel gains,
* set nominal speed,
* set line-follow gains,
* arm and execute sensor calibration,
* start and stop robot motion,
* print status,
* stream diagnostic data,
* toggle some IMU-related settings if those shares exist.

The class is designed as a stateful UI task. Instead of reading one full command line, it often transitions through mini-modes such as `GET_KP`, `GET_KI`, `GET_SP`, and `SENSE_STREAM`.

### Nuances

* Calibration is intentionally “armed” before white/black calibration commands are accepted. That reduces accidental miscalibration.
* The button can start and stop the run directly, which is convenient during physical tests.
* Streaming mode prints line-follow stage, counts, setpoints, gains, and line status. This is extremely helpful for matching observed robot behavior to internal state-machine transitions.
* IMU commands exist even though the present `main.py` does not appear to wire in IMU shares. That suggests the UI code was written to support expansion.

### Reflection

This file reflects how we repeatedly tuned Kp, Ki, line gains, nominal speed, and calibration values while observing streamed telemetry. While a lot of the inputs were left over from previous labs as extra functions, the calibration and debugging capabilities of the UI were extremely useful when figuring out the precise encoder ticks needed for our script. 

**See also:** [ui_help.py](#ui_helppy), [main.py](#mainpy)

---

## ui_help.py

### Purpose

`ui_help.py` prints the command menu shown over USB serial.

### How it works

This file contains a single helper function that writes a formatted help table to the serial interface. It is kept separate from the task user in order to minimize file size. 

### Nuances

* The menu includes IMU-related options, even though the current integration may not use them.
* A short delay is added after printing, likely to help ensure serial output completes cleanly.

### Reflection

This is a support file rather than a control file, but it still contributes to usability. 

**See also:** [task_user.py](#task_userpy)

---

## Provided Infrastructure Files

## cotask.py

This file, provided by Dr. John Ridgely, implements the cooperative multitasking framework. It defines the `Task` abstraction, readiness logic, optional profiling/trace support, and the scheduler that runs tasks by priority or round-robin. In this project, it is the backbone that allows line following, bumper recovery, wheel control, and the UI to coexist without preemptive threading.

**See also:** [main.py](#mainpy)

---

## task_share.py

This file, also provided by Dr. John Ridgely, implements safe shared variables and queues for inter-task communication. In this project, it is used extensively for gains, setpoints, measured wheel velocities, positions, enable flags, line errors, calibration commands, and debug values.

**See also:** [main.py](#mainpy), [task_motor.py](#task_motorpy), [task_follow_line.py](#task_follow_linepy)

The project’s biggest achievement is architectural: it turns a difficult real-world robot task into manageable layers. Its second biggest achievement is procedural: it preserves evidence of iterative testing and course-specific refinement. Those qualities matter because in robotics, the best code is rarely the shortest or most abstract. It is the code that works repeatedly on the real machine, under imperfect conditions, and still remains understandable enough to improve.

