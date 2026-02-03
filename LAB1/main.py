import pyb, time

# ============================================================
# MOTOR PINS (GPIO)
# ============================================================

DIR_L = pyb.Pin('PC0', pyb.Pin.OUT_PP)
SLP_L = pyb.Pin('PC1', pyb.Pin.OUT_PP)

DIR_R = pyb.Pin('PC2', pyb.Pin.OUT_PP)
SLP_R = pyb.Pin('PC3', pyb.Pin.OUT_PP)

# ============================================================
# PWM (TIM4 @ 20 kHz)
# ============================================================
PWM_FREQ = 20000
tim4 = pyb.Timer(4, freq=PWM_FREQ)

# Left PWM: PB9 = TIM4_CH4
pwmL = tim4.channel(4, pyb.Timer.PWM, pin=pyb.Pin('PB9'))

# Right PWM: PB8 = TIM4_CH3
pwmR = tim4.channel(3, pyb.Timer.PWM, pin=pyb.Pin('PB8'))

pwmL.pulse_width_percent(0)
pwmR.pulse_width_percent(0)

# ============================================================
# ENCODERS
# ============================================================
# Left encoder: TIM2 on PA0 / PA1

ENC_FLIP_L = -1  
ENC_FLIP_R = 1 

PA0 = pyb.Pin('PA0', pyb.Pin.AF_PP, af=pyb.Pin.AF1_TIM2)
PA1 = pyb.Pin('PA1', pyb.Pin.AF_PP, af=pyb.Pin.AF1_TIM2)
timL = pyb.Timer(2, prescaler=0, period=0xFFFF)
timL.channel(1, mode=pyb.Timer.ENC_AB, pin=PA0)
timL.channel(2, mode=pyb.Timer.ENC_AB, pin=PA1)

# Right encoder: TIM1 on PA8 / PA9
PA8 = pyb.Pin('PA8', pyb.Pin.AF_PP, af=pyb.Pin.AF1_TIM1)
PA9 = pyb.Pin('PA9', pyb.Pin.AF_PP, af=pyb.Pin.AF1_TIM1)
timR = pyb.Timer(1, prescaler=0, period=0xFFFF)
timR.channel(1, mode=pyb.Timer.ENC_AB, pin=PA8)
timR.channel(2, mode=pyb.Timer.ENC_AB, pin=PA9)

# ============================================================
# HELPERS
# ============================================================
def signed_delta_u16(new, old):    # UNDERSTAND
    d = (new - old) & 0xFFFF
    if d >= 0x8000:
        d -= 0x10000
    return d

def reset_encoders():
    timL.counter(0)
    timR.counter(0)

def wake_drivers():
    pwmL.pulse_width_percent(0)
    pwmR.pulse_width_percent(0)
    SLP_L.low()
    SLP_R.low()
    time.sleep_ms(150)
    SLP_L.high()
    SLP_R.high()
    time.sleep_ms(150)

def stop(ms=200):
    pwmL.pulse_width_percent(0)
    pwmR.pulse_width_percent(0)
    time.sleep_ms(ms)

def set_dir(forward=True):
    if forward:
        DIR_L.value(0)
        DIR_R.value(0)
    else:
        DIR_L.value(1)
        DIR_R.value(1)
  
posL = 0
posR = 0

def show_positions(duration_s=3, dt_ms=100, forward=True, duty=50):
    global posL, posR 

    prevL = timL.counter()
    prevR = timR.counter()

    sign = 1 if forward else -1

    print("posL   posR")
    print("------------")

    set_dir(forward)
    pwmL.pulse_width_percent(duty)
    pwmR.pulse_width_percent(duty)
    time.sleep_ms(200)

    t_end = time.ticks_add(time.ticks_ms(), int(duration_s * 1000))
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        curL = timL.counter()
        curR = timR.counter()

        posL += sign * ENC_FLIP_L * signed_delta_u16(curL, prevL)
        posR += ENC_FLIP_R * signed_delta_u16(curR, prevR)

        print("{:6d} {:6d}".format(posL, posR))

        prevL = curL
        prevR = curR
        time.sleep_ms(dt_ms)

    stop()

# ============================================================
# MAIN
# ============================================================

wake_drivers()
reset_encoders()

print("FORWARD (counts go up)")
show_positions(duration_s=1, dt_ms=100, forward=True, duty=10)


time.sleep(1)

print("\nBACKWARD (counts go down)")
show_positions(duration_s=1, dt_ms=100, forward=False, duty=10)

time.sleep(1)

print("FORWARD (counts go up)")
show_positions(duration_s=1, dt_ms=100, forward=True, duty=20)

time.sleep(1)

print("\nBACKWARD (counts go down)")
show_positions(duration_s=1, dt_ms=100, forward=False, duty=20)

# Sleep drivers

SLP_L.low()
SLP_R.low()
print("DONE")
