import pyb

try:
    tim2 = pyb.Timer(2, freq=20000)
    pwmR = tim2.channel(1, pyb.Timer.PWM, pin=pyb.Pin('PA15'))
    pwmL = tim2.channel(2, pyb.Timer.PWM, pin=pyb.Pin('PB3'))
    pwmR.pulse_width_percent(0)
    pwmL.pulse_width_percent(0)
    print("PWM forced to 0% on PA15 (CH1) and PB3 (CH2).")
except Exception as e:
    print("PWM stop error:", e)

