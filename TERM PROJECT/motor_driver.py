from pyb import Pin

class motor_driver:

    def __init__(self, PWM, DIR, nSLP):
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.nDIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)
        self.pwm = PWM
        self.pwm.pulse_width_percent(0)
        self.nDIR_pin.low()
        self.nSLP_pin.low()
        self.effort = 0

    def set_effort(self, effort):
        if effort > 100:
            effort = 100
        elif effort < -100:
            effort = -100

        self.effort = effort

        if effort >= 0:
            self.nDIR_pin.low()
            duty = effort
        else:
            self.nDIR_pin.high()
            duty = -effort

        self.pwm.pulse_width_percent(duty)

    def enable(self):
        self.nSLP_pin.high()
        self.pwm.pulse_width_percent(0)

    def disable(self):
        self.nSLP_pin.low()
