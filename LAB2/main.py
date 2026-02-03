
if __name__ == "__main__":
    
    import pyb, time
    from motor import Motor
    from encoder import Encoder

    # ============================================================

    PWM_FREQ = 20000

    tim4 = pyb.Timer(4, freq=PWM_FREQ) 

    pwmL = tim4.channel(4, pyb.Timer.PWM, pin=pyb.Pin('PB9'))
    pwmR = tim4.channel(3, pyb.Timer.PWM, pin=pyb.Pin('PB8'))

    motorL = Motor(pwmL, 'PC0', 'PC1')  # DIR_L, SLP_L
    motorR = Motor(pwmR, 'PC2', 'PC3')  # DIR_R, SLP_R

    # ============================================================

    encL = Encoder(2, 'PA0', 'PA1')
    encR = Encoder(1, 'PA8', 'PA9')

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