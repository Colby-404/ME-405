'''
Closed-loop control task. Uses a simple P/PID controller to compute effort
commands for the motor task based on a setpoint and measured omega. The
controller gains and setpoint can be updated in real time by the user task.

Closed loop control task is implemented in task_motor via the effort share. 
The user task updates the setpoint and controller gains in real time based on user commands, 
and the motor task reads those values and implements the control algorithm to compute the 
appropriate effort command to send to the motor driver.
'''

# Use MicroPython-compatible timing
try:
    from time import ticks_us, ticks_diff
except ImportError:
    # Fallback for some ports
    from utime import ticks_us, ticks_diff


class task_control_loop:
    """PID controller for wheel speed. Outputs effort command."""

    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, sat=100,
                 deriv_on_measurement=True, output_as_int=True):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Saturation limit (None or <=0 means "no saturation")
        self.sat = sat

        # Behavior options
        self.deriv_on_measurement = deriv_on_measurement
        self.output_as_int = output_as_int

        # Controller state
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_meas = 0.0
        self.prev_t = ticks_us()

        # Debug/diagnostic state (handy later if you want to log internals)
        self.dt = 0.0
        self.unsat = 0.0
        self.output = 0.0

    # ------------------------------------------------------------
    # Gain / config update methods (kept + added for compatibility)
    # ------------------------------------------------------------

    def set_gains(self, Kp=None, Ki=None, Kd=None):
        """Live gain update (compatible with your current task_control loop)."""
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd

    # Methods modeled after pid_controller.py API
    def update_Kp(self, kp):
        self.Kp = kp

    def update_Ki(self, ki):
        self.Ki = ki

    def update_Kd(self, kd):
        self.Kd = kd

    def update_saturation(self, saturation):
        self.sat = saturation

    # ------------------------------------------------------------
    # Reset / zero
    # ------------------------------------------------------------

    def reset(self):
        """Reset controller state (safe to call on enable edge)."""
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_meas = 0.0
        self.prev_t = ticks_us()
        self.dt = 0.0
        self.unsat = 0.0
        self.output = 0.0

    # Alias for compatibility with pid_controller style
    def zero(self):
        self.reset()

    # ------------------------------------------------------------
    # Core PID update
    # ------------------------------------------------------------

    def update(self, ref, meas):
        """
        Compute next control effort.

        Returns:
            int by default (for your effort Share("h"))
            float if output_as_int=False
        """

        # Time step in seconds (microsecond resolution)
        now = ticks_us()
        dt_us = ticks_diff(now, self.prev_t)
        self.prev_t = now
        self.dt = (dt_us / 1_000_000.0) if dt_us > 0 else 0.0

        # Error
        err = ref - meas

        # Derivative term
        if self.dt > 0:
            if self.deriv_on_measurement:
                # D on measurement (your current behavior, avoids setpoint kick)
                derr = -(meas - self.prev_meas) / self.dt
            else:
                # Optional D on error (pid_controller-style derivative source)
                derr = (err - self.prev_err) / self.dt
        else:
            derr = 0.0

        # P and D with current state
        P = self.Kp * err
        D = self.Kd * derr

        # Current unsaturated output using present integral state
        self.unsat = P + (self.Ki * self.integral) + D

        # ---- Conditional integration anti-windup (from pid_controller idea) ----
        # Candidate integral update
        if self.dt > 0:
            integ_candidate = self.integral + err * self.dt
        else:
            integ_candidate = self.integral

        # Candidate output if we accept the integral update
        u_candidate = P + (self.Ki * integ_candidate) + D

        # Saturation logic
        sat_active = (self.sat is not None) and (self.sat > 0)

        if sat_active:
            # Only integrate if candidate output remains within saturation
            if -self.sat <= u_candidate <= self.sat:
                self.integral = integ_candidate
                u = u_candidate
            else:
                # Do not integrate further into saturation
                u = self.unsat

            # Final clamp (always enforce output bounds)
            if u > self.sat:
                u = self.sat
            elif u < -self.sat:
                u = -self.sat
        else:
            # No saturation mode
            self.integral = integ_candidate
            u = u_candidate

        # Save state for next iteration
        self.prev_meas = meas
        self.prev_err = err
        self.output = u

        # Match your current motor-effort behavior
        if self.output_as_int:
            return int(u)
        else:
            return u


def task_control(enable, setpoint, Kp, Ki, omega_meas, effort, Kd=None, sat=100):
    """
    Share-based control task generator (drop-in compatible).

    Required shares (same as your current code):
        enable, setpoint, Kp, Ki, omega_meas, effort

    Optional:
        Kd  -> Share for derivative gain (if you add one later)
        sat -> numeric saturation (default 100)
    """
    # Build controller using current share values
    ctrl = task_control_loop(
        Kp=Kp.get(),
        Ki=Ki.get(),
        Kd=(Kd.get() if Kd is not None else 0.0),
        sat=sat,
        deriv_on_measurement=True,   # keeps your current better behavior
        output_as_int=True           # effort Share("h") friendly
    )

    enabled_last = False

    while True:
        en = enable.get()

        # Rising edge: reset controller cleanly
        if en and not enabled_last:
            ctrl.reset()

            # Preload previous measurement/error to prevent startup derivative spike
            meas0 = omega_meas.get()
            ctrl.prev_meas = meas0
            ctrl.prev_err = setpoint.get() - meas0
            ctrl.prev_t = ticks_us()

        enabled_last = en

        if en:
            # Live gain updates from shares
            ctrl.update_Kp(Kp.get())
            ctrl.update_Ki(Ki.get())

            if Kd is not None:
                ctrl.update_Kd(Kd.get())

            ctrl.update_saturation(sat)

            # Compute and publish effort
            u = ctrl.update(setpoint.get(), omega_meas.get())
            effort.put(u)
        else:
            # Safe stop + optional anti-windup hygiene while disabled
            effort.put(0)
            # (Optional) uncomment if you want integrator to clear every disabled cycle:
            # ctrl.integral = 0.0

        yield 0