from motor_driver import motor_driver
from encoder      import encoder
from task_motor   import task_motor
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from gc           import collect
import pyb 

# Build all driver objects first

PWM_FREQ = 20000
tim4 = pyb.Timer(4, freq=PWM_FREQ) 

pwmL = tim4.channel(4, pyb.Timer.PWM, pin=pyb.Pin('PB9'))
pwmR = tim4.channel(3, pyb.Timer.PWM, pin=pyb.Pin('PB8'))

motorL = motor_driver(pwmL, 'PC0', 'PC1')  # DIR_L, SLP_L
motorR = motor_driver(pwmR, 'PC2', 'PC3')  # DIR_R, SLP_R
encL = encoder(2, 'PA0', 'PA1')
encR = encoder(1, 'PA8', 'PA9')

# Build shares and queues
motorLGo = Share("B",     name="Left Mot. Go Flag")
motorRGo = Share("B",     name="Right Mot. Go Flag")
dataValues    = Queue("f", 30, name="Data Collection Buffer")
timeValues    = Queue("L", 30, name="Time Buffer")

# Build task class objects
leftMotorTask  = task_motor(motorL,  encL,
                            motorLGo, dataValues, timeValues)
rightMotorTask = task_motor(motorR, encR,
                            motorRGo, dataValues, timeValues)
userTask = task_user(motorLGo, motorRGo, dataValues, timeValues)

# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))

# Run the garbage collector preemptively
collect()

if __name__ == "__main__":
    
    ENC_FLIP_L = 1
    ENC_FLIP_R = 1

    # ============================================================

    motorL.enable()
    motorR.enable()
    motorL.set_effort(0)
    motorR.set_effort(0)

while True:
        # Forward
        motorL.set_effort(20)
        motorR.set_effort(20)


        t_end = pyb.millis() + 10000
        while pyb.millis() < t_end:
            encL.update()
            encR.update()
            print("{:6d} {:6d}".format(ENC_FLIP_L * encL.get_position(),
                                       ENC_FLIP_R * encR.get_position()))
            pyb.delay(50)

        # Backward
        motorL.set_effort(-20)
        motorR.set_effort(-20)

        t_end = pyb.millis() + 10000
        while pyb.millis() < t_end:
            encL.update()
            encR.update()
            print("{:6d} {:6d}".format(ENC_FLIP_L * encL.get_position(),
                                       ENC_FLIP_R * encR.get_position()))
            pyb.delay(50)

        # Stop
        motorL.set_effort(0)
        motorR.set_effort(0)
        pyb.delay(1000)

# Run the scheduler until the user quits the program with Ctrl-C
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


