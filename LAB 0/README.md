# ME 405 – Lab 0x00
## Interrupt Callbacks and ADC Reading

Author: Colby Cordoba and Hobbs Hegedus   
Course: ME 405 – Mechatronics  
Institution: California Polytechnic State University, San Luis Obispo  

---

## Overview

The objective of this lab was to measure the step response of a first-order RC circuit using the analog-to-digital converter (ADC) of an STM32 Nucleo-L476RG microcontroller running MicroPython.

A digital output pin on the microcontroller was used to apply a step input to the RC circuit. The capacitor voltage was then sampled using the ADC at a fixed frequency using a hardware timer callback. The collected data was exported in CSV format and used to generate plots of the experimental step response.

From the measured step response, the time constant of the RC circuit was experimentally determined and compared to the theoretical value calculated from the resistor and capacitor values.

---

## Hardware Setup

The experiment used the STM32 Nucleo-L476RG development board and a simple RC circuit constructed on a breadboard.

### Protection Resistor

A 5.6 kΩ resistor was placed between the RC circuit output and the ADC input.  
This resistor protects the microcontroller in case the circuit is wired incorrectly or the pin is configured improperly.

Because the input impedance of the ADC is very high, this resistor does not significantly affect the measured voltage during normal operation.

### Circuit Components

| Component | Value |
|----------|------|
| Resistor | 65 kΩ |
| Capacitor | 4.7 μF |
| Protection Resistor | 5.6 kΩ |

### Microcontroller Connections

| Pin | Function |
|----|----|
| PC1 | Digital output to apply step input |
| PC0 | ADC input to measure capacitor voltage |
| PC13 | Optional user button trigger |

The RC circuit was connected so that pin **PC1 applied the step input**, while **PC0 measured the capacitor voltage through the protection resistor**.

---

## Theory

An RC circuit exhibits first-order dynamic behavior. When a step voltage is applied, the capacitor voltage increases exponentially toward its steady-state value.

The capacitor voltage as a function of time is

V(t) = V∞(1 − e^(−t/τ))

where

- V(t) = capacitor voltage  
- V∞ = final voltage  
- t = time  
- τ = time constant  

The time constant of an RC circuit is

τ = RC

For this circuit

τ = (65,000)(4.7 × 10⁻⁶)

τ = 0.3055 s

At one time constant, the capacitor reaches approximately **63.2% of its final value**.

---

## Software Implementation

The MicroPython program performs the following steps:

1. Configure pin **PC1** as a digital output to generate the step input.
2. Configure pin **PC0** as an ADC input.
3. Create a timer running at **1000 Hz**.
4. Use a **timer interrupt callback** to sample the ADC.
5. Store ADC samples in an array.
6. Convert raw ADC values to voltage.
7. Print the data in **CSV format** for export and plotting.

---

## Key Code Concepts

### ADC Sampling

The capacitor voltage is measured using the MicroPython ADC class.

The type code `'H'` represents an unsigned 16-bit integer, which is ideal for storing ADC readings because the STM32 ADC produces 12-bit values ranging from 0 to 4095.

Example configuration:

adc = pyb.ADC(pyb.Pin(pyb.Pin.cpu.C0))

Each timer callback reads the ADC and stores the value in an array for later processing.

---

### Timer Configuration

A hardware timer is used to trigger periodic sampling of the ADC.

Example timer setup:

tim7 = pyb.Timer(7, freq=1000)  
tim7.callback(tim_cb)

The timer runs at **1000 Hz**, meaning the ADC samples the capacitor voltage every 1 ms.

---

### Timer Callback Function

The callback function reads the ADC value and stores it in the data array until the array is full.

Example callback function:

def tim_cb(tim):
    global data, idx
    if idx < len(data):
        data[idx] = adc.read()
        idx += 1
    else:
        tim.callback(None)

Once the array is filled, the timer callback is disabled to prevent additional sampling.

---

## Data Collection

To generate the step response, the digital output pin PC1 is initially set low so that the capacitor is discharged. After a short delay, PC1 is set high to apply the step input to the RC circuit.

The timer callback continuously samples the ADC during this transition, recording the capacitor voltage over time.

Sampling was performed at **1000 Hz**, allowing the system to capture the full transient response of the circuit.

---

## Data Export

After data collection finishes, the program converts the raw ADC values into voltage using the 12-bit ADC resolution and reference voltage.

Example conversion:

voltage = (data[i] / max_adc) * V_ref  
time_s = i * period

The data is printed in **CSV format** so that it can be easily copied from the REPL and imported into analysis software such as Excel or MATLAB.

Example output format:

t_s,V  
0.000000,0.012  
0.001000,0.045  
0.002000,0.090  

This CSV data was then used to generate plots of the capacitor voltage versus time.

---

## Results

The experimental data was plotted to produce the step response of the RC circuit.

From the plot, the time constant was determined by identifying the time at which the capacitor voltage reached **63.2% of the final voltage**.

### Experimental Time Constant

τ_exp = 0.315 s

### Theoretical Time Constant

τ_RC = 0.3055 s

---

## Percent Error

The percent error between the experimental and theoretical values was calculated using

%Error = |τ_exp − τ_RC| / τ_RC × 100

%Error = 3.1%

This small error indicates that the experimental results closely matched the expected theoretical behavior.

---

## Discussion

The measured step response followed the expected exponential charging behavior of a first-order RC circuit. The experimental time constant was very close to the theoretical value predicted using the component values.

Minor discrepancies between theoretical and experimental values may be caused by

- component tolerances  
- measurement noise  
- wiring resistance  
- ADC quantization error  

Despite these sources of error, the measured results were in strong agreement with the theoretical model.

---

## Conclusion

This lab demonstrated how a microcontroller can be used to measure the dynamic response of an electrical circuit using timer interrupts and ADC sampling.

By implementing a timer callback in MicroPython, the system was able to collect high-frequency voltage samples from the RC circuit during a step input. The resulting data was exported in CSV format and analyzed to determine the circuit time constant.

The experimental time constant closely matched the theoretical value calculated from the resistor and capacitor values, validating both the measurement method and the circuit model.

This lab also introduced key concepts used throughout the ME 405 course, including

- microcontroller pin configuration  
- timer interrupts  
- analog-to-digital conversion  
- data collection and export  
- embedded system data analysis  

---

## Files in This Folder

| File | Description |
|-----|-----|
| main.py | MicroPython program used to collect ADC step response data |
| README.md | Documentation explaining the lab |
| Lab0_report.pdf | Lab memo containing plots and analysis |
