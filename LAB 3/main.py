from motor_driver import motor_driver
from encoder      import encoder
from task_motor   import task_motor
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from task_control_loop import task_control_fun, task_control_loop
from gc           import collect
import pyb


# ============================
# MODE SWITCH
# ============================
RUN_STEP_TEST = False   # True = run open-loop motor/encoder test
                        # False = run multitasking system

# -----------------------------
# Hardware setup
# -----------------------------
PWM_FREQ = 20000
tim4 = pyb.Timer(4, freq=PWM_FREQ)

pwmL = tim4.channel(4, pyb.Timer.PWM, pin=pyb.Pin('PB9'))
pwmR = tim4.channel(3, pyb.Timer.PWM, pin=pyb.Pin('PB8'))

motorL = motor_driver(pwmL, 'PC0', 'PC1')  # DIR_L, nSLP_L
motorR = motor_driver(pwmR, 'PC2', 'PC3')  # DIR_R, nSLP_R

encL = encoder(2, 'PA0', 'PA1')
encR = encoder(1, 'PA8', 'PA9')


# -----------------------------
# Shares (type codes)
# -----------------------------
# B = uint8 enable flags
# h = int16 motor effort (-100..100)
# f = float values (omega, setpoint, gains)
# L = uint32 timestamps (ticks_us)

motorLGo = Share("B", name="Left Enable")
motorRGo = Share("B", name="Right Enable")

setpoint = Share("f", name="Setpoint")
Kp       = Share("f", name="Kp Gain")

omegaL   = Share("f", name="Omega L")
omegaR   = Share("f", name="Omega R")

effortL  = Share("h", name="Left Effort")
effortR  = Share("h", name="Right Effort")

# Initial values
motorLGo.put(0)
motorRGo.put(0)
setpoint.put(6)
Kp.put(10)
effortL.put(0)
effortR.put(0)


# -----------------------------
# Queues (separate per motor)
# -----------------------------
dataValuesL = Queue("f", 100, name="Left Data Buffer")     # omega log
timeValuesL = Queue("L", 100, name="Left Time Buffer")     # uint32

dataValuesR = Queue("f", 100, name="Right Data Buffer")
timeValuesR = Queue("L", 100, name="Right Time Buffer")


# -----------------------------
# Task objects
# -----------------------------
leftMotorTask  = task_motor(motorL, encL, motorLGo, effortL, omegaL, dataValuesL, timeValuesL)
rightMotorTask = task_motor(motorR, encR, motorRGo, effortR, omegaR, dataValuesR, timeValuesR)

def cl_left():
    yield from task_control_fun(motorLGo, setpoint, Kp, omegaL, effortL)

def cl_right():
    yield from task_control_fun(motorRGo, setpoint, Kp, omegaR, effortR)

# If your task_user only supports ONE queue set, pass left for now
# (upgrade later to accept both sets)
#userTask = task_user(motorLGo, motorRGo, dataValuesL, timeValuesL)
userTask = task_user(motorLGo, motorRGo, dataValuesL, timeValuesL)

# -----------------------------
# Add tasks to scheduler
# -----------------------------

task_list.append(Task(leftMotorTask.run,  name="Left Motor",  priority=2, period=10, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Motor", priority=2, period=10, profile=True))

task_list.append(Task(cl_left,  name="Left Control",  priority=1, period=10, profile=True))
task_list.append(Task(cl_right, name="Right Control", priority=1, period=10, profile=True))


task_list.append(Task(userTask.run, name="User", priority=0, period=50, profile=False))



collect()


# ============================
# STEP TEST MODE
# ============================
if RUN_STEP_TEST:
    ENC_FLIP_L = 1
    ENC_FLIP_R = 1

    motorL.enable()
    motorR.enable()
    motorL.set_effort(0)
    motorR.set_effort(0)

    def run_step(effort, duration_ms=1000, sample_ms=50):
        motorL.set_effort(effort)
        motorR.set_effort(effort)

        t0 = pyb.millis()
        last = t0
        while pyb.millis() - t0 < duration_ms:
            now = pyb.millis()
            if now - last >= sample_ms:
                last = now
                encL.update()
                encR.update()
                posL = ENC_FLIP_L * encL.get_position()
                posR = ENC_FLIP_R * encR.get_position()
                velL = ENC_FLIP_L * encL.get_velocity()
                velR = ENC_FLIP_R * encR.get_velocity()
                print("{:6d} e={:4d} posL={:7.0f} velL={:8.5f} posR={:7.0f} velR={:8.5f}"
                      .format(now - t0, effort, posL, velL, posR, velR))

        motorL.set_effort(0)
        motorR.set_effort(0)
        pyb.delay(500)

    for e in [10, 20, 30, -10, -20, -30]:
        print("\nSTEP TEST effort =", e)
        run_step(e, duration_ms=1000, sample_ms=50)

    motorL.disable()
    motorR.disable()


# ============================
# SCHEDULER MODE
# ============================
else:
    while True:
        try:
            task_list.pri_sched()
        except KeyboardInterrupt:
            print("Program Terminating")
            motorL.disable()
            motorR.disable()
            break

    print("\n")
    print(task_list)
    print(show_all())
