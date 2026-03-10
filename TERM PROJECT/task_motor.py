# task_motor.py (Internal PI supported)
# - Default mode: same as legacy (reads effort share from external controller)
# - Optional mode: internal PI controller using encoder velocity (counts/s) + Kp/Ki shares
#
# For your new setup, use_internal_pi=True in main.py and DO NOT run task_control_loop
# for the same motor.

from time import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)


class task_motor:
    def __init__(self,
                 mot,
                 enc,
                 enable,
                 effort,
                 pos_meas,
                 omega_meas,
                 data_q=None,
                 time_q=None,
                 # -------- Optional internal PI mode --------
                 setpoint=None,      # Share("f") in counts/s
                 Kp=None,            # Share("f")
                 Ki=None,            # Share("f")
                 use_internal_pi=False,
                 effort_sat=100,
                 pi_output_as_int=True):
        """
        Motor + encoder task.

        Default behavior (legacy-compatible):
            - reads effort from effort share (external controller writes effort)
            - publishes omega_meas / pos_meas
            - logs omega + timestamps

        Internal PI mode:
            - computes effort inside this task using setpoint, Kp, Ki, omega_meas
            - still publishes omega_meas / pos_meas
            - still writes computed effort to effort share (for visibility/debug)
            - uses encoder velocity in counts/s as PI measurement (matches your enc.get_velocity())
            - effort_sat is applied to PI output if > 0 (symmetric +/- saturation)
        """
        self._mot = mot
        self._enc = enc
        self._en = enable
        self._effort = effort
        self._pos_meas = pos_meas
        self._omega_meas = omega_meas

        self._data_q = data_q
        self._time_q = time_q

    # existing queue put() logging code
        # Logging behavior: stop logging only, never stop motor due to full queue
        self._logging_enabled = True

        # Runtime latch
        self._motor_enabled = False

        # Time bases
        self._t0 = ticks_us()          # logging timestamp zero
        self._prev_ctrl_t = ticks_us() # PI dt timing

        # -------- Internal PI mode config --------
        self._setpoint = setpoint
        self._Kp = Kp
        self._Ki = Ki
        self._use_internal_pi = bool(use_internal_pi)

        # Fail-safe to external mode if required shares are missing
        if self._use_internal_pi:
            if (self._setpoint is None) or (self._Kp is None) or (self._Ki is None):
                self._use_internal_pi = False
                print("task_motor: internal PI disabled (missing setpoint/Kp/Ki share)")

        

        self._effort_sat = int(abs(effort_sat))
        self._pi_output_as_int = bool(pi_output_as_int)

        # PI state
        self._i_term = 0.0
        self._prev_meas = 0.0
        self._prev_err = 0.0

        # Start safe
        try:
            self._mot.disable()
        except Exception:
            pass

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------

    def _reset_pi(self, meas0=0.0, ref0=0.0):
        self._i_term = 0.0
        self._prev_meas = float(meas0)
        self._prev_err = float(ref0 - meas0)
        self._prev_ctrl_t = ticks_us()

    def _clamp(self, x, lo, hi):
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def _compute_internal_pi_effort(self, ref_counts_per_s, meas_counts_per_s):
        """
        PI controller in COUNTS/S units (matches your encoder.get_velocity()).
        Includes conditional integration anti-windup.
        """
        now = ticks_us()
        dt_us = ticks_diff(now, self._prev_ctrl_t)
        self._prev_ctrl_t = now
        dt = (dt_us / 1_000_000.0) if dt_us > 0 else 0.0

        ref = float(ref_counts_per_s)
        meas = float(meas_counts_per_s)
        err = ref - meas

        kp = float(self._Kp.get()) if self._Kp is not None else 0.0
        ki = float(self._Ki.get()) if self._Ki is not None else 0.0

        P = kp * err

        # Candidate integral update
        if dt > 0:
            i_candidate = self._i_term + err * dt
        else:
            i_candidate = self._i_term

        # Candidate and unsaturated outputs
        u_unsat = P + ki * self._i_term
        u_candidate = P + ki * i_candidate

        sat = self._effort_sat if self._effort_sat > 0 else None

        if sat is not None:
            # Anti-windup: only integrate if candidate output stays within limits
            if -sat <= u_candidate <= sat:
                self._i_term = i_candidate
                u = u_candidate
            else:
                u = u_unsat

            u = self._clamp(u, -sat, sat)
        else:
            self._i_term = i_candidate
            u = u_candidate

        self._prev_meas = meas
        self._prev_err = err

        if self._pi_output_as_int:
            return int(u)
        return u

    # ------------------------------------------------------------
    # Main task generator
    # ------------------------------------------------------------

    def run(self):
        state = S0_INIT

        while True:

            # ---------------- S0: INIT ----------------
            if state == S0_INIT:
                try:
                    self._mot.disable()
                except Exception:
                    pass

                self._motor_enabled = False
                self._logging_enabled = True
                self._enc.zero()

                # Initialize measurement shares
                self._omega_meas.put(0.0)
                self._pos_meas.put(0)

                # Reset PI state if using internal PI
                if self._use_internal_pi:
                    ref0 = float(self._setpoint.get()) if self._setpoint is not None else 0.0
                    self._reset_pi(meas0=0.0, ref0=ref0)

                state = S1_WAIT

            # ---------------- S1: WAIT ----------------
            elif state == S1_WAIT:
                if self._en.get():
                    self._enc.zero()
                    self._t0 = ticks_us()
                    self._logging_enabled = True

                    # Prime PI at run start to avoid startup kick
                    if self._use_internal_pi:
                        self._enc.update()
                        omega0 = self._enc.get_velocity()    # counts/s
                        pos0 = self._enc.get_position()      # counts
                        self._omega_meas.put(float(omega0))
                        self._pos_meas.put(int(pos0))

                        ref0 = float(self._setpoint.get()) if self._setpoint is not None else 0.0
                        self._reset_pi(meas0=omega0, ref0=ref0)

                    state = S2_RUN

            # ---------------- S2: RUN ----------------
            elif state == S2_RUN:
                if not self._en.get():
                    # Stop safely
                    try:
                        self._mot.set_effort(0)
                    except Exception:
                        pass
                    try:
                        self._mot.disable()
                    except Exception:
                        pass

                    self._motor_enabled = False

                    if self._use_internal_pi:
                        self._reset_pi(meas0=0.0, ref0=0.0)

                    state = S1_WAIT

                else:
                    # Enable motor once
                    if not self._motor_enabled:
                        try:
                            self._mot.enable()
                        except Exception:
                            pass
                        self._motor_enabled = True

                    # Encoder update + publish measurements
                    self._enc.update()
                    omega = self._enc.get_velocity()   # counts/s
                    pos = self._enc.get_position()     # counts

                    self._omega_meas.put(float(omega))
                    self._pos_meas.put(int(pos))

                    # Compute or read effort
                    if self._use_internal_pi:
                        ref = float(self._setpoint.get())  # counts/s
                        u = self._compute_internal_pi_effort(ref, omega)

                        # Publish computed effort for visibility / UI
                        try:
                            self._effort.put(int(u))
                        except Exception:
                            self._effort.put(float(u))

                        effort_cmd = u
                    else:
                        # Legacy external-control mode
                        effort_cmd = self._effort.get()

                    # Apply effort to motor
                    self._mot.set_effort(effort_cmd)

                    # Logging (omega + timestamp in microseconds)
                    if self._logging_enabled and (self._data_q is not None) and (self._time_q is not None):
                        if (not self._data_q.full()) and (not self._time_q.full()):
                            t_us = ticks_diff(ticks_us(), self._t0)
                            self._data_q.put(float(omega))
                            self._time_q.put(t_us)
                        else:
                            self._logging_enabled = False

            yield state