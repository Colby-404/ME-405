# main.py (low-RAM build)
# - Internal PI inside task_motor
# - Line-follow outer loop
# - IMU task
# - State estimator task
# - Serial tuning UI (task_tuning_ui)

import gc
import micropython
import pyb

# Compile subsequent imports with higher optimization to save RAM
try:
    micropython.opt_level(3)
except Exception:
    pass

gc.collect()

# ---- Low-RAM import order ----
from task_tuning_ui import task_tuning_ui
import gc; gc.collect()

from motor_driver import motor_driver
import gc; gc.collect()
from encoder import encoder
import gc; gc.collect()
from task_motor import task_motor
import gc; gc.collect()
from task_share import Share
import gc; gc.collect()
from cotask import Task, task_list
import gc; gc.collect()
from task_follow_line import task_follow_line
import gc; gc.collect()
from sensor_driver import QTRSensorsAnalog
import gc; gc.collect()

from imu_driver import BNO055
import gc; gc.collect()
from task_imu import task_imu
import gc; gc.collect()

from task_state_estimator import task_state_estimator
import gc; gc.collect()
# -----------------------------

# USB
ser = pyb.USB_VCP()
ser.write(b"\r\nmain start\r\n")
pyb.delay(20)

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
# IMU (BNO055) over I2C2 (soft)
#   SCL -> PB13, SDA -> PB14
# -----------------------------
imu = BNO055.from_softi2c('PB13', 'PB14', freq=100000, addr=0x28)
IMU_FUSION_MODE = BNO055.MODE_IMUPLUS

# -----------------------------
# Line sensor (QTR-MD-08A Analog)
# -----------------------------
ADC_PINS = ['PC5', 'PC4', 'PB1', 'PB0', 'PA7', 'PA6', 'PA5', 'PA4']
qtr = QTRSensorsAnalog(ADC_PINS, emitter_pin=None, invert=False)
qtr.min_reading = 10

# -----------------------------
# Shares (no long 'name=' strings to save qstr RAM)
# -----------------------------
motorLGo = Share('B')
motorRGo = Share('B')

Kp = Share('f')
Ki = Share('f')

setpointL = Share('f')
setpointR = Share('f')

omegaL = Share('f')
omegaR = Share('f')
posL = Share('l')
posR = Share('l')

effortL = Share('h')
effortR = Share('h')

follow_en = Share('B')
v_nom     = Share('f')
Kp_line   = Share('f')
Ki_line   = Share('f')
line_err  = Share('f')
dv_out    = Share('f')

cal_cmd  = Share('B')
cal_done = Share('B')

# IMU shares
imu_en        = Share('B')
imu_mode      = Share('B')
imu_zero_cmd  = Share('B')
imu_save_cmd  = Share('B')
imu_heading   = Share('f')
imu_yawrate   = Share('f')
imu_calraw    = Share('B')

# Estimator shares (minimal set used by UI + step_collector)
est_en   = Share('B')
xhat_s   = Share('f')
xhat_psi = Share('f')
x_pos    = Share('f')
y_pos    = Share('f')
dist_traveled = Share('f')

# Optional estimator outputs for proof / logging
yhat_sL     = Share('f')
yhat_sR     = Share('f')
yhat_psi    = Share('f')
yhat_psidot = Share('f')

# -----------------------------
# Initial values
# -----------------------------
motorLGo.put(0)
motorRGo.put(0)

Kp.put(0.09)
Ki.put(0.0)

v_nom.put(1000.0)
setpointL.put(v_nom.get())
setpointR.put(v_nom.get())

follow_en.put(0)
Kp_line.put(0.7)
Ki_line.put(0.0)

line_err.put(0.0)
dv_out.put(0.0)

cal_cmd.put(0)
cal_done.put(0)

effortL.put(0)
effortR.put(0)

imu_en.put(1)
imu_mode.put(int(IMU_FUSION_MODE) & 0xFF)
imu_zero_cmd.put(0)
imu_save_cmd.put(0)
imu_heading.put(0.0)
imu_yawrate.put(0.0)
imu_calraw.put(0)

est_en.put(1)
xhat_s.put(0.0)
xhat_psi.put(0.0)
x_pos.put(0.0)
y_pos.put(0.0)
dist_traveled.put(0.0)

yhat_sL.put(0.0)
yhat_sR.put(0.0)
yhat_psi.put(0.0)
yhat_psidot.put(0.0)

# -----------------------------
# Motor tasks (internal PI)
# -----------------------------
leftMotorTask = task_motor(
    motorL, encL,
    motorLGo, effortL, posL, omegaL,
    data_q=None, time_q=None,
    setpoint=setpointL, Kp=Kp, Ki=Ki,
    use_internal_pi=True,
    effort_sat=100
)

rightMotorTask = task_motor(
    motorR, encR,
    motorRGo, effortR, posR, omegaR,
    data_q=None, time_q=None,
    setpoint=setpointR, Kp=Kp, Ki=Ki,
    use_internal_pi=True,
    effort_sat=100
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
    calib_file='bno055_calib.bin',
    try_load_calib=True,
    fusion_mode=IMU_FUSION_MODE
)

# -----------------------------
# State estimator task (minimal publishes)
# -----------------------------
estTask = task_state_estimator(
    est_en,
    posL, posR,
    effortL, effortR,
    imu_heading, imu_yawrate,
    xhat_s, xhat_psi,
    x_pos=x_pos, y_pos=y_pos, dist_traveled=dist_traveled,
    yhat_sL=yhat_sL, yhat_sR=yhat_sR, yhat_psi=yhat_psi, yhat_psidot=yhat_psidot,
    wheel_radius_mm=35.0,
    track_width_mm=141.0,
    counts_per_rad=229.1831,
    v_batt=7.4,
    unwrap_heading=True,
    pose_heading_sign=-1.0
)

# -----------------------------
# Line sensor read task (updates line_err)
# -----------------------------
def task_read_line():
    OVERSAMPLE = 4
    CAL_SAMPLES = 80
    STRENGTH_MIN = 400
    ACTIVE_MIN = 2
    ACTIVE_TH = 80

    while True:
        try:
            cmd = int(cal_cmd.get())

            if cmd == 1:
                qtr.calibrate_white(samples=CAL_SAMPLES, oversample=OVERSAMPLE,
                                    emitters=True, settle_ms=100)
                cal_cmd.put(0)
                cal_done.put(1)

            elif cmd == 2:
                qtr.calibrate_black(samples=CAL_SAMPLES, oversample=OVERSAMPLE,
                                    emitters=True, settle_ms=100)
                if hasattr(qtr, 'fix_calibration_order'):
                    qtr.fix_calibration_order()
                cal_cmd.put(0)
                cal_done.put(1)

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
            
        except Exception as e:
            # keep running even if sensor hiccups
            try:
                print('line err', e)
            except Exception:
                pass

        yield 0

# -----------------------------
# Line-follow task (writes wheel setpoints)
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
    line_ok=None,
    sp_min=-3000.0,
    sp_max=3000.0
)

# -----------------------------
# UI task
# -----------------------------
uiTask = task_tuning_ui(
    v_nom, Kp, Ki,
    ser=ser,
    motors=(motorLGo, motorRGo),
    line=(follow_en, Kp_line, Ki_line, line_err, dv_out),
    cal=(cal_cmd, cal_done),
    imu=(imu_en, imu_mode, imu_zero_cmd, imu_save_cmd, imu_heading, imu_yawrate, imu_calraw),
    enc=(posL, posR),
    est=(est_en, xhat_s, xhat_psi, x_pos, y_pos, dist_traveled, yhat_sL, yhat_sR, yhat_psi, yhat_psidot)
)

# -----------------------------
# Scheduler
# -----------------------------
task_list.append(Task(leftMotorTask.run,  name='L', priority=1, period=20))
task_list.append(Task(rightMotorTask.run, name='R', priority=1, period=20))

task_list.append(Task(task_read_line,     name='S', priority=3, period=20))
task_list.append(Task(followTask.run,     name='F', priority=3, period=20))

task_list.append(Task(imuTask.run,        name='I', priority=2, period=20))
task_list.append(Task(estTask.run,        name='E', priority=2, period=20))

task_list.append(Task(uiTask.run,         name='U', priority=0, period=50))

# -----------------------------
# RUN
# -----------------------------
while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        motorL.disable(); motorR.disable()
        break
