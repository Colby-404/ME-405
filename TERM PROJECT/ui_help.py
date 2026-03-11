import pyb

def print_help(s):
    s.write(b"+------------------------------------------------------------------------------+\r\n")
    s.write(b"| ME 405 Romi Tuning Interface (Lab 0x05)                                      |\r\n")
    s.write(b"+---+--------------------------------------------------------------------------+\r\n")
    s.write(b"| h | Help menu                                                                |\r\n")
    s.write(b"| k | Enter wheel gains (Kp, Ki)                                               |\r\n")
    s.write(b"| s | Set V_nom (forward speed setpoint)                                       |\r\n")
    s.write(b"+------------------------------------------------------------------------------+\r\n")
    s.write(b"| c | Arm calibration (w=white, b=black, a=cancel)                             |\r\n")
    s.write(b"| o | Enter line-follow gains (Kp_line, Ki_line)                               |\r\n")
    s.write(b"| n | Run motors (line-follow + stream to PC with IMU data)                    |\r\n")
    s.write(b"| x | Stop motors (also stops stream)                                          |\r\n")
    s.write(b"+------------------------------------------------------------------------------+\r\n")
    s.write(b"| p | Print line + IMU status (err, dv, follow, heading, yawrate, calib)       |\r\n")
    s.write(b"| f | Toggle line follow enable                                                |\r\n")  
    s.write(b"| i | Toggle IMU enable (imu_en)                                               |\r\n") 
    s.write(b"| m | Toggle IMU fusion mode (IMUPLUS<-> NDOF)                                 |\r\n") 
    s.write(b"| z | Zero IMU heading (sets current heading to 0)                             |\r\n") 
    s.write(b"| u | Save IMU calibration profile to bno055_calib.bin                         |\r\n")

    # NEED to make it so line not okay does not imediately trigger event or add specifc line not okay error because as it is now it is simply just doing it everytime line not okay. 
        # After this change it should do all of the major run tasks in one sweep however during the second sequence may need to disable line following until ending sequence in order to keep it from detecting false line.
    # Bump sensor ISR that does a 200 and enables line following again 
    # push forwords
    # Program stops when encoder reach a specific rotation. 
    s.write(b"+---+--------------------------------------------------------------------------+\r\n")
    s.write(b"\r\n>: ")
    pyb.delay(10)
