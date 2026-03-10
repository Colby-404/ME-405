# main.py (internal PI inside task_motor, line-follow architecture preserved)
# IMU integrated into tuning UI (enable/mode/zero/save) and published via task_imu
# NOTE: State estimator is intentionally omitted until IMU + line-follow are stable.

import gc
import pyb

gc.collect()

# ---- Low-RAM import order ----
# Import the largest modules first while the heap is clean, and GC between imports.

# Original import kept here for later if you want to restore the old data-collection UI:
# from task_user import task_user, task_tuning_ui
from task_user import task_tuning_ui
gc.collect()

from motor_driver import motor_driver
gc.collect()
from encoder import encoder
gc.collect()
from task_motor import task_motor
gc.collect()
from task_share import Share, Queue
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

# -----------------------------
# USB
# -----------------------------
ser = pyb.USB_VCP()
ser.write(b"\r\n*** main.py started (Internal PI in task_motor build) ***\r\n")
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

# NOTE: If a wheel "runs away" or oscillates badly, flip the corresponding invert flag.
encL = encoder(2, 'PA0', 'PA1', invert=False)
encR = encoder(1, 'PA8', 'PA9', invert=False)

# -----------------------------
# (A) IMU setup (BNO055 over I2C2)
# Wiring per your table:
#   SCL -> PB13
#   SDA -> PB14
# -----------------------------

imu = BNO055.from_softi2c('PB13', 'PB14', freq=100000, addr=0x28)

# print("I2C scan:", i2c.scan())

# Default fusion mode (can be overridden at runtime from UI via imu_mode share)
# - IMUPLUS: accel+gyro only (no magnetometer) -> often more stable indoors
# - NDOF: accel+gyro+mag -> absolute heading, but mag can be noisy near motors
IMU_FUSION_MODE = BNO055.MODE_IMUPLUS

# -----------------------------
# Line sensor setup (QTR-MD-08A Analog)
# -----------------------------
ADC_PINS = [
    'PC5',  # OUT7
    'PC4',  # OUT6
    'PB1',  # OUT5
    'PB0',  # OUT4
    'PA7',  # OUT3
    'PA6',  # OUT2
    'PA5',  # OUT1
    'PA4',  # OUT0
]

# If QTR emitter/CTRL pin is wired, set it here (must NOT be one of ADC_PINS)
EMITTER_PIN = None

qtr = QTRSensorsAnalog(ADC_PINS, emitter_pin=EMITTER_PIN, invert=False)
qtr.min_reading = 10

# -----------------------------
# Shares
# -----------------------------
motorLGo = Share("B", name="Left Enable")
motorRGo = Share("B", name="Right Enable")

# Wheel PI gains (used directly by task_motor internal PI)
Kp = Share("f", name="Wheel Kp")
Ki = Share("f", name="Wheel Ki")

# Per-wheel velocity setpoints (counts/s)
setpointL = Share("f", name="Setpoint L")
setpointR = Share("f", name="Setpoint R")

# Measured wheel states
omegaL = Share("f", name="Omega L")
omegaR = Share("f", name="Omega R")
posL = Share("l", name="Pos L")
posR = Share("l", name="Pos R")

# Effort shares kept for visibility/debug/UI compatibility
effortL = Share("h", name="Left Effort")
effortR = Share("h", name="Right Effort")

# Kept for compatibility with the current task_tuning_ui implementation.
# Old data-collection UI used this to switch menus.
start_user = Share("B", name="Start User Task")

# Outer-loop (line-follow) shares
follow_en = Share("B", name="Follow Enable")
v_nom     = Share("f", name="V_nom")      # counts/s
Kp_line   = Share("f", name="Line Kp")
Ki_line   = Share("f", name="Line Ki")
line_err  = Share("f", name="Line Error")
dv_out    = Share("f", name="dv_out")
line_ok   = Share("B", name="Line OK")

# Calibration command shares (line sensor)
cal_cmd  = Share("B", name="Cal Cmd")
cal_done = Share("B", name="Cal Done")

# -----------------------------
# IMU shares
# -----------------------------
imu_en        = Share("B", name="IMU Enable")         # 0/1
imu_mode      = Share("B", name="IMU Mode")           # BNO055 mode byte (0x08, 0x0C, ...)
imu_zero_cmd  = Share("B", name="IMU Zero Cmd")       # write 1 to zero heading
imu_save_cmd  = Share("B", name="IMU Save Cal Cmd")   # write 1 to save calib profile to file

imu_heading = Share("f", name="IMU Heading deg")      # published (zeroed heading)
imu_yawrate = Share("f", name="IMU YawRate dps")      # published
imu_calraw  = Share("B", name="IMU Cal Raw")          # published

# -----------------------------
# Initial values
# -----------------------------
motorLGo.put(0)
motorRGo.put(0)

Kp.put(0.09)
Ki.put(0.0)

v_nom.put(800.0)
setpointL.put(v_nom.get())
setpointR.put(v_nom.get())

follow_en.put(1)
Kp_line.put(0.7)
Ki_line.put(0.0)

line_err.put(0.0)
dv_out.put(0.0)
line_ok.put(0)

cal_cmd.put(0)
cal_done.put(0)

effortL.put(0)
effortR.put(0)

start_user.put(0)

imu_en.put(1)
imu_mode.put(int(IMU_FUSION_MODE) & 0xFF)
imu_zero_cmd.put(0)
imu_save_cmd.put(0)

imu_heading.put(0.0)
imu_yawrate.put(0.0)
imu_calraw.put(0)

# -----------------------------
# Queues (wheel logging)
# -----------------------------
# Kept for now because task_motor may still expect these buffers.
# If you later confirm task_motor does not need them, these are a good place to save RAM.
dataValuesL = Queue("f", 100, name="Left Data Buffer")
timeValuesL = Queue("L", 100, name="Left Time Buffer")

dataValuesR = Queue("f", 100, name="Right Data Buffer")
timeValuesR = Queue("L", 100, name="Right Time Buffer")

# -----------------------------
# Motor tasks (INTERNAL PI ENABLED)
# -----------------------------
leftMotorTask = task_motor(
    motorL, encL,
    motorLGo, effortL, posL, omegaL,
    dataValuesL, timeValuesL,
    setpoint=setpointL,
    Kp=Kp,
    Ki=Ki,
    use_internal_pi=True,
    effort_sat=100
)

rightMotorTask = task_motor(
    motorR, encR,
    motorRGo, effortR, posR, omegaR,
    dataValuesR, timeValuesR,
    setpoint=setpointR,
    Kp=Kp,
    Ki=Ki,
    use_internal_pi=True,
    effort_sat=100
)

# -----------------------------
# IMU task: updates imu_heading + imu_yawrate + imu_calraw
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
# Sensor read task: updates line_err + line_ok + handles calibration
# -----------------------------
def task_read_line():
    OVERSAMPLE = 4
    CAL_SAMPLES = 80

    STRENGTH_MIN = 400
    ACTIVE_MIN   = 2
    ACTIVE_TH    = 80

    while True:
        try:
            cmd = int(cal_cmd.get())

            if cmd == 1:
                qtr.calibrate_white(samples=CAL_SAMPLES, oversample=OVERSAMPLE,
                                    emitters=True, settle_ms=100)
                cal_cmd.put(0)
                cal_done.put(1)
                print("WHITE:", qtr.white)

            elif cmd == 2:
                qtr.calibrate_black(samples=CAL_SAMPLES, oversample=OVERSAMPLE,
                                    emitters=True, settle_ms=100)
                if hasattr(qtr, "fix_calibration_order"):
                    qtr.fix_calibration_order()
                cal_cmd.put(0)
                cal_done.put(1)
                print("BLACK:", qtr.black)

            qtr.read_normalized(oversample=OVERSAMPLE, emitters=True)

            strength = 0
            active = 0
            for v in qtr.norm:
                strength += v
                if v > ACTIVE_TH:
                    active += 1

            if (strength >= STRENGTH_MIN) and (active >= ACTIVE_MIN):
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
                    line_err.put(float(pos - center))
                    line_ok.put(1)
                else:
                    line_ok.put(0)
            else:
                line_ok.put(0)

        except Exception as e:
            line_ok.put(0)
            print("task_read_line error:", e)

        yield 0

# -----------------------------
# Outer-loop line-follow task (writes setpointL/setpointR)
# -----------------------------
followTask = task_follow_line(
    enable_follow=follow_en,
    v_nom=v_nom,
    Kp_line=Kp_line,
    Ki_line=Ki_line,
    line_err=line_err,
    spL=setpointL,
    spR=setpointR,
    dv_out=dv_out,
    sat_dv=600.0,
    period_ms=50,
    line_ok=line_ok,
    sp_min=-3000.0,
    sp_max=3000.0,

    heading_deg=imu_heading,
    posL_meas=posL,
    posR_meas=posR,
    use_line_recovery=True,

    # First scripted section:
    #   Robot drives forward this many encoder counts after the first line loss
    #   before starting the 90-degree right turn.
    lost_forward_counts=150.0,

    # First scripted section:
    #   Right-turn angle in degrees for the first special maneuver.
    turn_right_deg=90.0,

    # Second scripted section:
    #   Robot drives forward this many encoder counts before beginning the 360.
    stage1_forward1_counts=150.0,

    # Second scripted section:
    #   Half-turn angle used twice to make a full 360 (180 + 180).
    stage1_turn_half_deg=180.0,

    # Second scripted section:
    #   Robot drives forward this many encoder counts after the 360
    #   before continuing straight until the line is found again.
    stage1_forward2_counts=150.0,

    # Shared recovery tuning:
    #   Allowed heading error band in degrees when deciding a turn is complete.
    heading_tol_deg=3.0,

    # Shared recovery tuning:
    #   Forward speed used during scripted recovery.
    #   If set to None, recovery uses the current v_nom value instead.
    recovery_fwd_speed=700.0,

    # Shared recovery tuning:
    #   Turn speed used during scripted recovery turns.
    recovery_turn_speed=450.0
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
    imu_heading=imu_heading,
    imu_yawrate=imu_yawrate,
    imu_calraw=imu_calraw,
    imu_en=imu_en,
    imu_mode=imu_mode,
    imu_zero_cmd=imu_zero_cmd,
    imu_save_cmd=imu_save_cmd,
)

# Old data-collection UI task kept commented out for later restore.
# userTask = task_user(
#     start_user,
#     motorLGo, motorRGo,
#     dataValuesL, timeValuesL,
#     dataValuesR, timeValuesR,
#     v_nom, Kp, Ki,
#     ser,
#     follow_en=follow_en,
#     Kp_line=Kp_line,
#     Ki_line=Ki_line,
#     line_err=line_err,
#     dv_out=dv_out,
#     imu_heading=imu_heading,
#     imu_yawrate=imu_yawrate,
#     imu_calraw=imu_calraw,
# )

# -----------------------------
# Scheduler
# -----------------------------
task_list.append(Task(leftMotorTask.run,  name="Left Motor",    priority=1, period=20))
task_list.append(Task(rightMotorTask.run, name="Right Motor",   priority=1, period=20))

task_list.append(Task(task_read_line,     name="Line Read",     priority=3, period=20))
task_list.append(Task(followTask.run,     name="Line Follow",   priority=3, period=20))

task_list.append(Task(imuTask.run,        name="IMU",           priority=2, period=20))

task_list.append(Task(tuningTask.run,     name="Tuning UI",     priority=0, period=50))

# Old data-collection UI task kept commented out for later restore.
# task_list.append(Task(userTask.run,       name="User",          priority=0, period=50))

# -----------------------------
# RUN
# -----------------------------
while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        motorL.disable()
        motorR.disable()
        print("Program Terminated")
        break