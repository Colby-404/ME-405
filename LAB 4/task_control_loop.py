'''
Closed-loop control task. Uses a simple P/PID controller to compute effort
commands for the motor task based on a setpoint and measured omega. The
controller gains and setpoint can be updated in real time by the user task.
'''

'''
Closed loop control task is implemented in task_motor via the effort share. 
The user task updates the setpoint and controller gains in real time based on user commands, 
and the motor task reads those values and implements the control algorithm to compute the 
appropriate effort command to send to the motor driver.
'''

import time

class task_control_loop:

    """Simple P/PID controller. Outputs effort command."""

    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, sat=100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sat = sat                      
        self.integral = 0.0                 # The integral term accumulates the error over time to eliminate steady-state error
        self.prev_err = 0.0                 # The previous error value is stored to compute the derivative term, which predicts future error based on the current rate of change of the error
        self.prev_meas = 0.0                # The previous measurement value is stored to compute the derivative term based on the rate of change of the measurement, which can help reduce derivative kick when the setpoint changes suddenly
        self.prev_t = time.ticks_ms()

    def set_gains(self, Kp=None, Ki=None, Kd=None):     # A method to update the controller gains in real time
        if Kp is not None: self.Kp = Kp                 # Update proportional gain if a new value is provided
        if Ki is not None: self.Ki = Ki
        if Kd is not None: self.Kd = Kd

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_meas = 0.0                            # Reset previous measurement to prevent derivative kick from sudden changes in setpoint or measurement that occur while the controller is disabled
        self.prev_t = time.ticks_ms()

    def update(self, ref, meas):
        now = time.ticks_ms()
        dt_ms = time.ticks_diff(now, self.prev_t)       # Compute time difference in milliseconds since the last update
        self.prev_t = now
        dt = dt_ms / 1000.0 if dt_ms > 0 else 0.0       # Convert time difference to seconds for use in integral and derivative calculations, ensuring we don't divide by zero

        err = ref - meas                                # Compute the current error between the reference setpoint and the measured value

        if dt > 0:
            self.integral += err * dt                   # Update the integral term by accumulating the error over time, scaled by the time difference
            derr = -(meas - self.prev_meas) / dt        # Derivative on measurement (D-on-y): provides damping and avoids derivative kick on setpoint steps.
        else:
            derr = 0.0                                  # If no time has passed, we can't compute a derivative, so we set it to zero

        self.prev_meas = meas                           # Update the previous measurement for the next derivative calculation
        self.prev_err  = err                            # Update the previous error for the next derivative calculation

        u = self.Kp*err + self.Ki*self.integral + self.Kd*derr

        if u > self.sat:  u = self.sat                  # Saturate the control output to the specified limits to prevent commanding the motor beyond its capabilities
        if u < -self.sat: u = -self.sat                 # Ensure the control output is not less than the negative saturation limit

        return int(u)                                   # Return the computed control effort as an integer, which can be used to command the motor driver. 
                                                        #The effort will be in the range [-sat, sat] based on the controller gains and the current error.
    
def task_control(enable, setpoint, Kp, Ki, omega_meas, effort):             # A generator function that implements the control loop task. 
                                                                            # It continuously checks if the control loop is enabled, and if so,
                                                                            # it updates the controller gains and setpoint based on the current values from the user task, 
                                                                            # computes the control effort using the controller, 
                                                                            # and updates the effort share for the motor task to command the motors accordingly. 
                                                                            # If the control loop is not enabled, it sets the effort to zero to stop the motors.
    ctrl = task_control_loop(Kp=Kp.get(), Ki=Ki.get(), Kd=0.0, sat=100)
    enabled_last = False

    while True:                                                             # The task runs indefinitely, continuously checking for updates to the enable flag, setpoint, 
                                                                            # and controller gains, and computing the control effort accordingly.
        en = enable.get()                                                   # Check if the control loop is enabled by reading the enable share.

        if en and not enabled_last:                                         # If we have just transitioned from disabled to enabled, reset the controller state to prevent 
                                                                            # integrator windup and derivative kick from sudden changes in setpoint or measurement that occur while the controller is disabled.
            ctrl.reset()
            ctrl.prev_meas = omega_meas.get()                               # prevents first-step derivative spike
        enabled_last = en

        if en:
            # apply live gain updates
            ctrl.set_gains(Kp=Kp.get(), Ki=Ki.get())                        # Update the controller gains in real time based on the current values from the user task.

            u = ctrl.update(setpoint.get(), omega_meas.get())
            effort.put(u)                                                   # Update the effort share for the motor task to command the motors with the computed control effort.
        else:
            effort.put(0)

        yield 0                                                            # Yield control back to the scheduler after each iteration, allowing other tasks to run.
