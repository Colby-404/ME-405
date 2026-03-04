from motor_driver import motor_driver
from encoder import encoder
from task_motor import task_motor
from task_user import task_user, task_tuning_ui
from task_share import Share, Queue
from cotask import Task, task_list
from task_control_loop import task_control
from gc import collect
import pyb

# -----------------------------
# USB
# -----------------------------
ser = pyb.USB_VCP()
ser.write(b"\r\n*** main.py started ***\r\n")
pyb.delay(50)

# -----------------------------
# Hardware setup
# -----------------------------
PWM_FREQ = 20000
tim4 = pyb.Timer(4, freq=PWM_FREQ)

pwmL = tim4.channel(4, pyb.Timer.PWM, pin=pyb.Pin('PB9'))
pwmR = tim4.channel(3, pyb.Timer.PWM, pin=pyb.Pin('PB8'))

motorL = motor_driver(pwmL, 'PC0', 'PC1')
motorR = motor_driver(pwmR, 'PC2', 'PC3')

encL = encoder(2, 'PA0', 'PA1')
encR = encoder(1, 'PA8', 'PA9')

# -----------------------------
# Shares
# -----------------------------
motorLGo = Share("B", name="Left Enable")
motorRGo = Share("B", name="Right Enable")

setpoint = Share("f", name="Setpoint")
Kp = Share("f", name="Kp Gain")
Ki = Share("f", name="Ki Gain")

omegaL = Share("f", name="Omega L")
omegaR = Share("f", name="Omega R")

effortL = Share("h", name="Left Effort")
effortR = Share("h", name="Right Effort")

start_user = Share("B", name="Start User Task")

posL = Share("l", name="Pos L")
posR = Share("l", name="Pos R")

# -----------------------------
# Initial values
# -----------------------------
motorLGo.put(0)
motorRGo.put(0)

setpoint.put(1200.0)   # counts/sec
Kp.put(0.02)
Ki.put(0.0)


effortL.put(0)
effortR.put(0)

start_user.put(0)   # tuning UI will set this to 1

# -----------------------------
# Queues
# -----------------------------
dataValuesL = Queue("f", 100, name="Left Data Buffer")
timeValuesL = Queue("L", 100, name="Left Time Buffer")

dataValuesR = Queue("f", 100, name="Right Data Buffer")
timeValuesR = Queue("L", 100, name="Right Time Buffer")

# -----------------------------
# Motor tasks
# -----------------------------
leftMotorTask = task_motor(
    motorL,
    encL,
    motorLGo,
    effortL,
    posL,
    omegaL,
    dataValuesL,
    timeValuesL
)

rightMotorTask = task_motor(
    motorR,
    encR,
    motorRGo,
    effortR,
    posR,
    omegaR,
    dataValuesR,
    timeValuesR
)

# -----------------------------
# Control tasks
# -----------------------------
def cl_left():
    yield from task_control(
        motorLGo,
        setpoint,
        Kp,
        Ki,
        omegaL,
        effortL
    )

def cl_right():
    yield from task_control(
        motorRGo,
        setpoint,
        Kp,
        Ki,
        omegaR,
        effortR
    )

# -----------------------------
# UI tasks
# -----------------------------
tuningTask = task_tuning_ui(
    start_user,
    setpoint,
    Kp,
    Ki,
    ser=ser
)

userTask = task_user(
    start_user,
    motorLGo,
    motorRGo,
    dataValuesL,
    timeValuesL,
    dataValuesR,
    timeValuesR,
    setpoint,
    Kp,
    Ki,
    ser
)

# -----------------------------
# Scheduler
# -----------------------------
task_list.append(Task(leftMotorTask.run,  name="Left Motor",  priority=1, period=50))
task_list.append(Task(rightMotorTask.run, name="Right Motor", priority=1, period=50))

task_list.append(Task(cl_left,  name="Left Control",  priority=2, period=50))
task_list.append(Task(cl_right, name="Right Control", priority=2, period=50))

task_list.append(Task(tuningTask.run, name="Tuning UI", priority=0, period=50))
task_list.append(Task(userTask.run,   name="User",      priority=0, period=50))

collect()

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