# ME-405 Lab 0x00: Timer callback ADC sampling on Nucleo L476RG (MicroPython)
# Hardware per lab:
#   - Step input pin:  PC1  (digital output to RC input)
#   - ADC input pin:   PC0  (reads capacitor voltage through 5k resistor)
#   - Optional trigger: Blue user button on PC13 (press to start a capture)

from array import array                 # Imports the 'array' module to create an array of unsigned short integers
import pyb
from pyb import Timer                   # Imports the 'Timer' class from the 'pyb' module to work with hardware timers
from pyb import Pin

PC1 = Pin(Pin.cpu.C1, mode=Pin.OUT_PP)  # Configures pin PC1 as a push-pull output pin
adc = pyb.ADC(pyb.Pin(pyb.Pin.cpu.C0))  # PC0 is ADC input per lab
tim7 = pyb.Timer(7, freq=1000)           # Create the timer object

idx = 0                                 # Initializes an index variable to 0
data = array('H', 300*[0])             # Creates an array of unsigned short integers with 300 elements initialized to 0

def tim_cb(tim):                        # Defines a function that takes in a single input argument representing a timer object
    global data,idx                     # Declares that the function will use the global variables 'data' and 'idx'
    if idx < len(data):                 # Checks if the current index is less than the length of the data array
        data[idx] = adc.read()          # Reads the ADC value and stores it in the data array at the current index
        idx += 1                        # Increment the index variable by 1
    else:                               # If the index is not less than the length of the data array
        tim.callback(None)              # Disable the timer callback to stop further sampling

PC1.low()                               # Set pin PC1 low
pyb.delay(500)                          # Wait for 500 milliseconds to allow data collection to complete

idx = 0                                 # Initializes an index variable to 0
tim7.callback(tim_cb)                   # Assign the callback

pyb.delay(2)                            # Wait for 200 milliseconds to allow timer to start ensuring at least one sample is taken

PC1.high()                              # Set pin PC1 high to create a step input

while idx<len(data):
    pass

tim7.callback(None)                     # Disable the timer callback to stop further sampling
PC1.low()                               # Set pin PC1 low

freq = 1000
period = 1/freq                         # Calculate the period in seconds
V_ref = 3.3                             # Reference voltage for ADC
max_adc = 4095                          # Maximum ADC value for 12-bit ADC

print("t_s,V")
for i in range(idx):
    voltage = (data[i] / max_adc) * V_ref          
    time_s = i * period
    print("{:.6f},{:.6f}".format(time_s, voltage))

print("idx =", idx)
print("last sample =", data[idx-1])
print("first 10 samples =", data[:10])