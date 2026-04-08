# ME 405 Project Repository

This repository contains coursework, lab files, support code, and term project material for ME 405.

## Repository Structure

- **LAB 0**  
  Introductory MicroPython lab focused on interrupt callbacks, ADC data collection, and RC step-response analysis.

- **LAB 1**  
  Romi hardware confirmation lab used to verify motor PWM control, encoder readings, and basic assembled robot functionality.

- **LAB 2**  
  Hardware driver development lab for creating reusable Python classes for the Romi motors and quadrature encoders.

- **LAB 3**  
  Closed-loop control lab that integrates the hardware drivers into a cooperative scheduler for velocity control using tasks and shared variables.

- **LAB 4**  
  User-interface and automated data-collection lab for triggering step responses, adjusting gains, and logging/plotting tuning data over serial.

- **LAB 5**  
  Line-following lab focused on building a line sensor driver and using closed-loop control to drive the Romi around a circular path.

- **LAB 6**  
  State-estimation lab that adds an IMU driver and combines sensor data with a system model to estimate the Romi’s motion and orientation.

- **ME405-Support-main**  
  Support files, reference code, and configuration material used across the labs and project.

- **TERM PROJECT**  
  Core materials for the final term project: a mobile robot designed to reliably follow a printed line track during an in-class time-trial demonstration, including all code, documentation, and hardware integration..

## Top-Level Files

- **README.md**  
  Main overview of the repository.

- **HARDWARE DESCRIPTION.md**  
  Summary of the hardware components used in the project.

- **HARDWARE SETUP.md**  
  Basic hardware assembly and setup instructions.

- **PINOUT.xlsx**  
  Pin mapping and wiring reference for the hardware connections.

- **.gitignore**  
  Git ignore rules for unnecessary or generated files.

## Purpose

This repository is used to organize the development, testing, and documentation of ME 405 lab work and the Romi-based term project.
