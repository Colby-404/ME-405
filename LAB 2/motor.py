
# Motor driver class for DRV8838-based motor drivers such as those on the Pololu Romi chassis.

# Assumes use of pyb module for Pin control.

# Handles the pins and timer channels needed to operate the motor driver.

from pyb import Pin

class Motor:

    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''
    
    def __init__(self, PWM, DIR, nSLP): 

        '''Initializes a Motor object'''
         # PWM   : pyb.Timer.channel(...)
         # DIR_pin  : pin name string like 'PC0'
         # nSLP_pin : pin name string like 'PC1

        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)  # SLP
        self.nDIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)   # DIR

        self.pwm = PWM # Stores PWM Channel
        self.pwm.pulse_width_percent(0)  # no effort
        self.nDIR_pin.low()                   
        self.nSLP_pin.low()
        
        self.effort = 0  # Current effort value

    def set_effort(self, effort):

        ''' Sets the present effort requested from the motor based on an input value
            between -100 and 100 for the motor output.'''
        
        if effort > 100:
            effort = 100
        elif effort < -100:
            effort = -100

        self.effort =  effort

        if effort >= 0:
            self.nDIR_pin.low() # Set direction pin high for reverse
            duty = effort
        else:
            self.nDIR_pin.high() # Set direction pin low for forword
            duty = -effort

        self.pwm.pulse_width_percent(duty)
            
    def enable(self):

        '''Enables the motor driver by taking it out of sleep mode into brake mode'''

        self.nSLP_pin.high()
        self.pwm.pulse_width_percent(0)  # no effort

    def disable(self): 

        '''Disables the motor driver by taking it into sleep mode'''

        self.nSLP_pin.low()