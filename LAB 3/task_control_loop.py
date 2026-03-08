'''
Closed-loop control task. Uses a simple P/PID controller to compute effort
commands for the motor task based on a setpoint and measured omega. The
controller gains and setpoint can be updated in real time by the user task.
'''
import time

class task_control_loop:
    """Simple P/PID controller. Outputs effort command."""

    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, sat=100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sat = sat
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = time.ticks_ms()

    def set_gains(self, Kp=None, Ki=None, Kd=None):
        if Kp is not None: self.Kp = Kp
        if Ki is not None: self.Ki = Ki
        if Kd is not None: self.Kd = Kd

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = time.ticks_ms()

    def update(self, ref, meas):
        now = time.ticks_ms()
        dt_ms = time.ticks_diff(now, self.prev_t)
        self.prev_t = now
        dt = dt_ms / 1000.0 if dt_ms > 0 else 0.0

        err = ref - meas

        if dt > 0:
            self.integral += err * dt
            derr = (err - self.prev_err) / dt
        else:
            derr = 0.0

        self.prev_err = err

        u = self.Kp*err + self.Ki*self.integral + self.Kd*derr

        if u > self.sat:  u = self.sat
        if u < -self.sat: u = -self.sat

        return int(u)
    

def task_control_fun(enable, setpoint, Kp, omega_meas, effort):
    ctrl = task_control_loop(Kp=0.0, Ki=0.0, Kd=0.0, sat=100)
    enabled_last = False

    while True:
        en = enable.get()

        if en and not enabled_last:
            ctrl.reset()
        enabled_last = en

        if en:
            ref  = setpoint.get()
            meas = omega_meas.get()
            ctrl.set_gains(Kp=Kp.get())
            effort.put(ctrl.update(ref, meas))
        else:
            effort.put(0)
        yield 0
