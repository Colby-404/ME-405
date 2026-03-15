# main.py

import gc
import pyb

gc.collect()

# ---- Low-RAM import order ----
from task_user import task_tuning_ui
gc.collect()

from motor_driver import motor_driver
gc.collect()
from encoder import encoder
gc.collect()
from task_motor import task_motor
gc.collect()
from task_share import Share
gc.collect()
from cotask import Task, task_list
gc.collect()
from task_follow_line import task_follow_line
gc.collect()
from sensor_driver import QTRSensorsAnalog
gc.collect()

from imu_driver import BNO055
gc.collect()
from task_imu import task_imu
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
# IMU (BNO055, I2C2: SCL=PB13, SDA=PB14)
# -----------------------------
imu = BNO055.from_softi2c('PB13', 'PB14', freq=100000, addr=0x28)
IMU_FUSION_MODE = BNO055.MODE_IMUPLUS  # accel+gyro only (no mag)

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
# Shares
# -----------------------------
motorLGo = Share("B", name="Left Enable")
motorRGo = Share("B", name="Right Enable")
Kp       = Share("f", name="Wheel Kp")
Ki       = Share("f", name="Wheel Ki")
setpointL = Share("f", name="Setpoint L")
setpointR = Share("f", name="Setpoint R")
omegaL   = Share("f", name="Omega L")
omegaR   = Share("f", name="Omega R")
posL     = Share("l", name="Pos L")
posR     = Share("l", name="Pos R")
effortL  = Share("h", name="Left Effort")
effortR  = Share("h", name="Right Effort")
pi_reset = Share("B", name="PI Reset")
start_user = Share("B", name="Start User Task")

# Outer-loop shares
follow_en = Share("B", name="Follow Enable")
v_nom     = Share("f", name="V_nom")      # counts/s
Kp_line   = Share("f", name="Line Kp")
Ki_line   = Share("f", name="Line Ki")
line_err  = Share("f", name="Line Error")
line_ok   = Share("B", name="Line OK")
dv_out    = Share("f", name="dv_out")

# Script debug shares (for encoder-based scripted tuning)
script_state = Share("B", name="Script State")
script_total_counts = Share("f", name="Script Total Counts")
script_segment_counts = Share("f", name="Script Segment Counts")

# Calibration command shares (line sensor)
cal_cmd  = Share("B", name="Cal Cmd")
cal_done = Share("B", name="Cal Done")

# -----------------------------
# IMU shares
# -----------------------------
imu_en        = Share("B", name="IMU Enable")
imu_mode      = Share("B", name="IMU Mode")
imu_zero_cmd  = Share("B", name="IMU Zero Cmd")
imu_save_cmd  = Share("B", name="IMU Save Cal Cmd")

imu_heading = Share("f", name="IMU Heading deg")
imu_yawrate = Share("f", name="IMU YawRate dps")
imu_calraw  = Share("B", name="IMU Cal Raw")

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

imu_en.put(1)
imu_mode.put(int(IMU_FUSION_MODE) & 0xFF)
imu_zero_cmd.put(0)
imu_save_cmd.put(0)

imu_heading.put(0.0)
imu_yawrate.put(0.0)
imu_calraw.put(0)

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
# IMU task
# -----------------------------
imuTask = task_imu(
    imu=imu,
    enable_share=imu_en,
    mode_share=imu_mode,
    zero_cmd_share=imu_zero_cmd,
    save_cal_cmd_share=imu_save_cmd,
    heading_deg=imu_heading,
    yaw_rate_dps=imu_yawrate,
    calib_raw=imu_calraw,
    require_mag=False,
    calib_file="bno055_calib.bin",
    try_load_calib=True,
    fusion_mode=IMU_FUSION_MODE
)

# -----------------------------
# Sensor read task: updates line_err and handles calibration
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
FOLLOW_SAT_DV    = 450.0
FOLLOW_SP_MIN    = -3000.0
FOLLOW_SP_MAX    = 3000.0

TUNE_TRIGGER     = 8000.0   # counts to enter tune zone
SCRIPT_TRIGGER   = 10600.0   # counts to start scripted maneuver

SEG_SMALL_RIGHT  = 185.0    # small right turn distance
SEG_FWD_1        = 1200.0   # straight after small right
SEG_TURN_90      = 595.0    # 90-deg right turn distance
SEG_FWD_2        = 5000.0   # final straight

SCRIPT_FWD_SPD   = 300.0   # reduced from 600 to limit back-EMF spike at S5 reversal
SCRIPT_TURN_SPD  = 400.0

LINE_LOST_MS     = 120.0
LINE_FOUND_MS    = 120.0
STRAIGHT_YAW_KP  = 1.5     # deg/s -> counts/s: tune sign if robot corrects wrong way

# Wheel PI gains: (Kp, Ki) per zone
BASE_KP,   BASE_KI   = 0.04, 0.05   # 0 to TUNE_TRIGGER
TUNE_KP,   TUNE_KI   = 0.04, 0.05   # TUNE_TRIGGER to SCRIPT_TRIGGER
SCRIPT_KP, SCRIPT_KI = 0.04, 0.05   # SCRIPT_TRIGGER onwards

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
    tune_wheel_Kp=TUNE_KP,     tune_wheel_Ki=TUNE_KI,
    script_wheel_Kp=SCRIPT_KP, script_wheel_Ki=SCRIPT_KI,
    line_lost_confirm_ms=LINE_LOST_MS,
    line_found_confirm_ms=LINE_FOUND_MS,
    script_state_share=script_state,
    script_total_counts_share=script_total_counts,
    script_segment_counts_share=script_segment_counts,
    imu_yawrate_share=None,
    straight_yaw_kp=0.0,
    pi_reset_cmd=pi_reset,
)
# -----------------------------
# UI tasks
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
    imu_heading=imu_heading,
    imu_yawrate=imu_yawrate,
    imu_calraw=imu_calraw,
    imu_en=imu_en,
    imu_mode=imu_mode,
    imu_zero_cmd=imu_zero_cmd,
    imu_save_cmd=imu_save_cmd,
)

# -----------------------------
# Scheduler
# -----------------------------
task_list.append(Task(leftMotorTask.run,  name="Left Motor",    priority=1, period=20))
task_list.append(Task(rightMotorTask.run, name="Right Motor",   priority=1, period=20))

task_list.append(Task(task_read_line,     name="Line Read",     priority=3, period=20))
task_list.append(Task(followTask.run,     name="Line Follow",   priority=3, period=20))

task_list.append(Task(imuTask.run,        name="IMU",           priority=2, period=20))
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