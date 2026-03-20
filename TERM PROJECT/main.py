# main.py

import gc
import pyb

gc.collect()

# ---- Low-RAM import order (largest first) ----
from task_follow_line import task_follow_line
gc.collect()
from task_user import task_tuning_ui
gc.collect()
from task_share import Share
gc.collect()
from cotask import Task, task_list
gc.collect()
from task_bumper_recovery import task_bumper_recovery
gc.collect()
from task_motor import task_motor
gc.collect()
from sensor_driver import QTRSensorsAnalog
gc.collect()
from motor_driver import motor_driver
gc.collect()
from encoder import encoder
gc.collect()


# -----------------------------
# USB
# -----------------------------
ser = pyb.USB_VCP()
ser.write(b"\r\n*** main.py started (two-trigger encoder tuning + scripted follow) ***\r\n")
pyb.delay(50)

# -----------------------------
# Hardware setup (Motors/Encoders)
# -----------------------------
PWM_FREQ = 20000
tim4 = pyb.Timer(4, freq=PWM_FREQ)

pwmL = tim4.channel(4, pyb.Timer.PWM, pin=pyb.Pin('PB9'))
pwmR = tim4.channel(3, pyb.Timer.PWM, pin=pyb.Pin('PB8'))

motorL = motor_driver(pwmL, 'PC0', 'PC1')
motorR = motor_driver(pwmR, 'PC2', 'PC3')

encL = encoder(2, 'PA0', 'PA1', invert=False)
encR = encoder(1, 'PA8', 'PA9', invert=False)


# -----------------------------
# Line sensor (QTR-MD-08A Analog)
# -----------------------------
ADC_PINS = ['PC5', 'PC4', 'PB1', 'PB0', 'PA7', 'PA6', 'PA5', 'PA4']
EMITTER_PIN = 'PC8'

qtr = QTRSensorsAnalog(ADC_PINS, emitter_pin=EMITTER_PIN, invert=False)
qtr.min_reading = 10
if hasattr(qtr, 'force_full_brightness'):
    qtr.force_full_brightness()
if hasattr(qtr, 'emitters_on'):
    qtr.emitters_on()

# -----------------------------
# Pololu bumper pins
# -----------------------------
BMP0_RIGHT_OUTER = 'PA10'
BMP1_RIGHT_MID   = 'PB3'
BMP2_RIGHT_INNER = 'PB4'

BMP5_LEFT_OUTER  = 'PB5'
BMP4_LEFT_MID    = 'PB10'
BMP3_LEFT_INNER  = 'PC7'

# -----------------------------
# Shares
# -----------------------------
motorLGo   = Share("B", name="Left Enable")
motorRGo   = Share("B", name="Right Enable")
Kp         = Share("f", name="Wheel Kp")
Ki         = Share("f", name="Wheel Ki")
setpointL  = Share("f", name="Setpoint L")
setpointR  = Share("f", name="Setpoint R")
omegaL     = Share("f", name="Omega L")
omegaR     = Share("f", name="Omega R")
posL       = Share("l", name="Pos L")
posR       = Share("l", name="Pos R")
effortL    = Share("h", name="Left Effort")
effortR    = Share("h", name="Right Effort")
pi_reset   = Share("B", name="PI Reset")
start_user = Share("B", name="Start User Task")

# Outer-loop shares
follow_en     = Share("B", name="Follow Enable")
bump_override = Share("B", name="Bump Override")
v_nom         = Share("f", name="V_nom")
Kp_line       = Share("f", name="Line Kp")
Ki_line       = Share("f", name="Line Ki")
line_err      = Share("f", name="Line Error")
line_ok       = Share("B", name="Line OK")
dv_out        = Share("f", name="dv_out")

# Script debug shares
script_state          = Share("B", name="Script State")
script_total_counts   = Share("f", name="Script Total Counts")
script_segment_counts = Share("f", name="Script Segment Counts")

# Calibration command shares
cal_cmd  = Share("B", name="Cal Cmd")
cal_done = Share("B", name="Cal Done")


# -----------------------------
# Initial values
# -----------------------------
motorLGo.put(0)
motorRGo.put(0)

Kp.put(0.04)
Ki.put(0.00)

v_nom.put(700.0)
setpointL.put(v_nom.get())
setpointR.put(v_nom.get())

follow_en.put(0)
bump_override.put(0)

Kp_line.put(0.5)
Ki_line.put(0.0)

line_err.put(0.0)
line_ok.put(1)
dv_out.put(0.0)

script_state.put(0)
script_total_counts.put(0.0)
script_segment_counts.put(0.0)

cal_cmd.put(0)
cal_done.put(0)

effortL.put(0)
effortR.put(0)
pi_reset.put(0)

start_user.put(0)


# -----------------------------
# Motor tasks
# -----------------------------
leftMotorTask = task_motor(
    motorL, encL,
    motorLGo, effortL, posL, omegaL,
    setpoint=setpointL,
    Kp=Kp,
    Ki=Ki,
    use_internal_pi=True,
    effort_sat=100,
    pi_reset_share=pi_reset
)

rightMotorTask = task_motor(
    motorR, encR,
    motorRGo, effortR, posR, omegaR,
    setpoint=setpointR,
    Kp=Kp,
    Ki=Ki,
    use_internal_pi=True,
    effort_sat=100,
    pi_reset_share=pi_reset
)


# -----------------------------
# Sensor read task
# -----------------------------
def task_read_line():
    OVERSAMPLE = 4
    CAL_SAMPLES = 80
    last_valid_err = 0.0

    while True:
        try:
            cmd = int(cal_cmd.get())

            if cmd == 1:
                qtr.calibrate_white(
                    samples=CAL_SAMPLES,
                    oversample=OVERSAMPLE,
                    emitters=True,
                    settle_ms=100
                )
                cal_cmd.put(0)
                cal_done.put(1)
                print("WHITE:", qtr.white)

            elif cmd == 2:
                qtr.calibrate_black(
                    samples=CAL_SAMPLES,
                    oversample=OVERSAMPLE,
                    emitters=True,
                    settle_ms=100
                )
                if hasattr(qtr, "fix_calibration_order"):
                    qtr.fix_calibration_order()
                cal_cmd.put(0)
                cal_done.put(1)
                print("BLACK:", qtr.black)

            qtr.read_normalized(oversample=OVERSAMPLE, emitters=True)

            weighted_sum = 0
            total = 0
            for i in range(qtr.count):
                v = qtr.norm[i]
                if v < qtr.min_reading:
                    v = 0
                weighted_sum += v * (i * 1000)
                total += v

            if total > 0:
                pos = weighted_sum // total
                center = (qtr.count - 1) * 1000 // 2
                last_valid_err = float(pos - center)
                line_err.put(last_valid_err)
                line_ok.put(1)
            else:
                line_err.put(last_valid_err)
                line_ok.put(0)

        except Exception as e:
            line_err.put(last_valid_err)
            line_ok.put(0)
            print("task_read_line error:", e)

        yield 0

# -----------------------------
# Line-follow tuning constants
# -----------------------------
FOLLOW_SAT_DV = 450.0
FOLLOW_SP_MIN = -3000.0
FOLLOW_SP_MAX = 3000.0

TUNE_TRIGGER   = 10000.0
SCRIPT_TRIGGER = 10600.0

SEG_SMALL_RIGHT = 135.0
SEG_FWD_1       = 950.0
SEG_TURN_90     = 555.0
SEG_FWD_2       = 5000.0


SEG_POST_BUMP_FOLLOW = 1500
SEG_FWD_3        = 500.0
SEG_TURN_FULL_90 = 700.0
POST_BUMP_SLOW_SPD = 200.0   # slower v_nom after turn, for curvy section

SCRIPT_FWD_SPD  = 300.0
SCRIPT_TURN_SPD = 400.0

LINE_LOST_MS    = 120.0
LINE_FOUND_MS   = 120.0

BASE_KP,   BASE_KI   = 0.04, 0.05
TUNE_KP_LINE, TUNE_KI_LINE = 0.5, 0.0
SCRIPT_KP, SCRIPT_KI = 0.04, 0.05

# -----------------------------
# Line-follow task
# -----------------------------
gc.collect()
followTask = task_follow_line(
    enable_follow=follow_en,
    v_nom=v_nom,
    Kp_line=Kp_line,
    Ki_line=Ki_line,
    line_err=line_err,
    line_ok=line_ok,
    spL=setpointL,
    spR=setpointR,
    dv_out=dv_out,
    sat_dv=FOLLOW_SAT_DV,
    sp_min=FOLLOW_SP_MIN,
    sp_max=FOLLOW_SP_MAX,
    posL_meas=posL,
    posR_meas=posR,
    enable_encoder_script=True,
    tune_trigger_counts=TUNE_TRIGGER,
    stage0_trigger_counts=SCRIPT_TRIGGER,
    small_right_counts=SEG_SMALL_RIGHT,
    stage0_forward1_counts=SEG_FWD_1,
    stage0_turn2_counts=SEG_TURN_90,
    stage0_forward2_counts=SEG_FWD_2,
    recovery_fwd_speed=SCRIPT_FWD_SPD,
    recovery_turn_speed=SCRIPT_TURN_SPD,
    wheel_Kp=Kp,
    wheel_Ki=Ki,
    base_wheel_Kp=BASE_KP,     base_wheel_Ki=BASE_KI,
    script_wheel_Kp=SCRIPT_KP, script_wheel_Ki=SCRIPT_KI,
    tune_kp_line=TUNE_KP_LINE, tune_ki_line=TUNE_KI_LINE,
    line_lost_confirm_ms=LINE_LOST_MS,
    line_found_confirm_ms=LINE_FOUND_MS,
    script_state_share=script_state,
    script_total_counts_share=script_total_counts,
    script_segment_counts_share=script_segment_counts,
    pi_reset_cmd=pi_reset,
    motion_override=bump_override,
    post_bump_follow_counts=SEG_POST_BUMP_FOLLOW,
    post_bump_fwd_counts=SEG_FWD_3,
    post_bump_turn_counts=SEG_TURN_FULL_90,
    post_bump_slow_speed=POST_BUMP_SLOW_SPD,
)

# -----------------------------
# Bumper recovery task
# -----------------------------
bumperTask = task_bumper_recovery(
    bmp_r_outer=BMP0_RIGHT_OUTER,
    bmp_r_mid=BMP1_RIGHT_MID,
    bmp_r_inner=BMP2_RIGHT_INNER,

    bmp_l_outer=BMP5_LEFT_OUTER,
    bmp_l_mid=BMP4_LEFT_MID,
    bmp_l_inner=BMP3_LEFT_INNER,

    follow_en=follow_en,
    motorLGo=motorLGo,
    motorRGo=motorRGo,
    setpointL=setpointL,
    setpointR=setpointR,
    posL=posL,
    posR=posR,
    pi_reset=pi_reset,
    motion_override=bump_override,

    reverse_speed=180.0,
    reverse_ticks=180.0,
    turn_speed=300.0,
    turn_ticks=400.0,
    brake_ms=120,
    resume_ms=80,
    debounce_ms=40,
    release_ms=60,
    rearm_ms=250,
    active_low=True
)

# -----------------------------
# UI task
# -----------------------------
tuningTask = task_tuning_ui(
    start_user,
    v_nom, Kp, Ki,
    ser=ser,
    follow_en=follow_en,
    Kp_line=Kp_line,
    Ki_line=Ki_line,
    cal_cmd=cal_cmd,
    cal_done=cal_done,
    line_err=line_err,
    dv_out=dv_out,
    leftMotorGo=motorLGo,
    rightMotorGo=motorRGo,
    posL=posL,
    posR=posR,
    omegaL=omegaL,
    omegaR=omegaR,
    setpointL=setpointL,
    setpointR=setpointR,
    script_state=script_state,
    script_total_counts=script_total_counts,
    script_segment_counts=script_segment_counts,
    line_ok=line_ok,
)

# -----------------------------
# Scheduler
# -----------------------------

task_list.append(Task(leftMotorTask.run,  name="Left Motor",    priority=1, period=20))
task_list.append(Task(rightMotorTask.run, name="Right Motor",   priority=1, period=20))

task_list.append(Task(task_read_line,     name="Line Read",     priority=3, period=20))
task_list.append(Task(followTask.run,     name="Line Follow",   priority=3, period=20))
task_list.append(Task(bumperTask.run,     name="Bumper",      priority=4, period=20))

task_list.append(Task(tuningTask.run,     name="Tuning UI",     priority=0, period=50))

# -----------------------------
# RUN
# -----------------------------
_err_count = 0
while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        motorL.disable()
        motorR.disable()
        print("Program Terminated")
        break
    except Exception as e:
        motorL.disable()
        motorR.disable()
        _err_count += 1
        print("Scheduler error:", e)
        if _err_count > 5:
            print("Too many errors, stopping.")
            break
        motorL.enable()
        motorR.enable()