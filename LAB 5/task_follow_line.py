# task_follow_line.py
# Outer-loop line-following task for ME405 Romi
#
# Default mode (drop-in compatible with your current main.py):
#   - reads line_err Share
#   - writes spL / spR
#   - optional line_ok gating
#
# Optional upgrades:
#   - Kd_line (PID instead of PI)
#   - microsecond timing + anti-windup
#   - dynamic turn saturation (fraction of |v_nom|)
#   - centroid logging queues
#   - direct weighted 2x3 sensor mode (llf/rlf) inspired by task_line_follower

import micropython

# MicroPython timing compatibility
try:
    import time
    ticks_us = time.ticks_us
    ticks_diff = time.ticks_diff
except AttributeError:
    from utime import ticks_us, ticks_diff

from task_share import Share

S0_IDLE = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_follow_line:
    def __init__(self,
                 enable_follow: Share,
                 v_nom: Share,
                 Kp_line: Share,
                 Ki_line: Share,
                 line_err: Share,
                 spL: Share,
                 spR: Share,
                 dv_out: Share = None,    
                 sat_dv: float = 400.0,
                 period_ms: int = 20,     # kept for compatibility/documentation
                 # Existing optionals
                 line_ok: Share = None,   # Share("B"): 1=line valid, 0=line lost
                 sp_min: float = -3000.0,
                 sp_max: float =  3000.0,


                 # ---- OPTIONALS ----
                 Kd_line: Share = None,   # optional line derivative gain share
                 deriv_on_measurement: bool = True,
                 output_sign: int = 1,    # +1 keeps current mixing: spL=v-dv, spR=v+dv
                 # saturation mode
                 dynamic_sat: bool = False,
                 sat_ratio: float = 0.75, # if dynamic_sat=True, sat = sat_ratio*abs(v_nom)
                 # optional logging (centroid/error)
                 centroid_q = None,       # Queue for centroid OR line_err proxy
                 centroid_t_q = None,     # Queue for timestamps (s)
                 centroid_out: Share = None,  # optional debug share for centroid
                 # optional direct 2x3 line sensor mode (from task_line_follower style)
                 llf = None,              # left line sensor object (expects get_values())
                 rlf = None,              # right line sensor object (expects get_values())
                 inner_weight: float = 8.0,
                 middle_weight: float = 16.0,
                 outer_weight: float = 24.0,
                 sensor_total_min: float = 1.0,     # validity threshold
                 sensor_mean_min: float = 1.0,      # validity threshold
                 sensor_error_scale_with_speed: bool = True):

        # Required shares (current architecture)
        self._en   = enable_follow
        self._vnom = v_nom
        self._Kp   = Kp_line
        self._Ki   = Ki_line
        self._Kd   = Kd_line
        self._err  = line_err
        self._spL  = spL
        self._spR  = spR
        self._dv_out = dv_out
        self._line_ok = line_ok

        # Output shaping / limits
        self._sat_dv = float(sat_dv) if sat_dv is not None else None
        self._dynamic_sat = bool(dynamic_sat)
        self._sat_ratio = float(sat_ratio)
        self._sp_min = float(sp_min)
        self._sp_max = float(sp_max)
        self._output_sign = 1 if output_sign >= 0 else -1

        # Timing / state
        self._state = S0_IDLE
        self._period_ms = int(period_ms)   # scheduler usually controls actual rate
        self._was_enabled = False
        self._start_t_us = ticks_us()
        self._prev_t_us = ticks_us()

        # PID state (line loop)
        self._i_term = 0.0
        self._prev_err = 0.0
        self._prev_meas = 0.0
        self._deriv_on_measurement = bool(deriv_on_measurement)

        # Optional logging
        # centroid is a general term for the "line position" metric that the controller uses internally; 
        # In our current architecture this is line_err, but if you use the optional weighted 2x3 sensor mode it will be a different computed value.
        self._centroid_q = centroid_q
        self._centroid_t_q = centroid_t_q
        self._centroid_out = centroid_out

        # Optional direct 2x3 sensor mode 
        # If llf/rlf provided, will compute line error directly from weighted 2x3 sensor arrays instead of line_err share
        # Note: not using a formal interface here since this is just an optional upgrade, but expects llf/rlf objects with a get_values() method that returns (in, mid, out) readings
        self._llf = llf
        self._rlf = rlf
        self._use_sensor_mode = (llf is not None and rlf is not None)

        self._w_in = float(inner_weight)
        self._w_mid = float(middle_weight)
        self._w_out = float(outer_weight)
        self._w_total = self._w_in + self._w_mid + self._w_out

        self._sensor_total_min = float(sensor_total_min)
        self._sensor_mean_min = float(sensor_mean_min)
        self._sensor_error_scale_with_speed = bool(sensor_error_scale_with_speed)

        print("task_follow_line 2x3 instantiated")

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------

    def _reset(self):
        self._i_term = 0.0
        self._prev_err = 0.0
        self._prev_meas = 0.0
        self._prev_t_us = ticks_us()
        self._start_t_us = self._prev_t_us

    def _clamp(self, x, lo, hi):
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def _active_sat(self, v_nom_value):
        """Compute active steering saturation (dv limit)."""
        if self._dynamic_sat:
            return abs(float(v_nom_value)) * self._sat_ratio
        return self._sat_dv

    def _log_centroid(self, centroid_value):
        """Optional centroid/error logging to queues and/or share."""
        if self._centroid_out is not None:
            try:
                self._centroid_out.put(float(centroid_value))
            except Exception:
                pass

        if self._centroid_q is not None:
            try:
                if not self._centroid_q.full():
                    self._centroid_q.put(float(centroid_value))
            except Exception:
                pass

        if self._centroid_t_q is not None:
            try:
                if not self._centroid_t_q.full():
                    t_s = ticks_diff(ticks_us(), self._start_t_us) / 1_000_000.0
                    self._centroid_t_q.put(float(t_s))
            except Exception:
                pass

    def _compute_error_from_weighted_sensors(self, v_nom_value):
        """
        Compute line error from two 3-sensor arrays (llf/rlf), inspired by task_line_follower.

        Returns:
            (valid, err, centroid)
        """
        try:
            Lin, Lmid, Lout = self._llf.get_values()
            Rin, Rmid, Rout = self._rlf.get_values()
        except Exception:
            return (False, 0.0, 0.0)

        # Weighted left/right sums
        left = self._w_in * Lin + self._w_mid * Lmid + self._w_out * Lout
        right = self._w_in * Rin + self._w_mid * Rmid + self._w_out * Rout

        raw_total = (Lin + Lmid + Lout + Rin + Rmid + Rout)
        if raw_total <= 0:
            return (False, 0.0, 0.0)

        left_mean = left / self._w_total
        right_mean = right / self._w_total
        mean = (left_mean + right_mean) * 0.5

        # Centroid metric (same form as the monolithic task)
        centroid = (left + right) / raw_total

        # Validity check
        valid = (raw_total >= self._sensor_total_min) and (mean >= self._sensor_mean_min)
        if not valid:
            return (False, 0.0, centroid)

        # Error signal (same spirit as task_line_follower)
        # Base raw error
        error = left - right

        # Normalize by brightness; optionally scale with speed so error is already in "turn command units"
        if mean <= 0:
            err = 0.0
        else:
            if self._sensor_error_scale_with_speed:
                err = (error * float(v_nom_value)) / mean
            else:
                err = error / mean

        return (True, float(err), float(centroid))

    def _get_line_measurement(self, v_nom_value):
        """
        Get line error + validity.

        Priority:
          1) weighted sensor mode (if llf/rlf provided)
          2) line_err share + optional line_ok share
        """
        # Direct sensor mode (optional)
        if self._use_sensor_mode:
            valid, err, centroid = self._compute_error_from_weighted_sensors(v_nom_value)

            # Publish computed error to line_err share so rest of system can inspect it
            try:
                self._err.put(float(err))
            except Exception:
                pass

            # Publish validity if line_ok share exists
            if self._line_ok is not None:
                try:
                    self._line_ok.put(1 if valid else 0)
                except Exception:
                    pass

            # Optional centroid logging/share
            self._log_centroid(centroid)

            return valid, float(err)

        # Default modular mode (your current architecture)
        if self._line_ok is not None:
            try:
                valid = bool(self._line_ok.get())
            except Exception:
                valid = True
        else:
            valid = True

        try:
            err = float(self._err.get())
        except Exception:
            err = 0.0

        # In modular mode, if centroid queues were provided, log line_err as a proxy
        # (still useful for tuning/debugging)
        if (self._centroid_q is not None) or (self._centroid_t_q is not None) or (self._centroid_out is not None):
            self._log_centroid(err)

        return valid, err

    # ------------------------------------------------------------
    # Main task generator
    # ------------------------------------------------------------

    def run(self):
        while True:
            enabled = bool(self._en.get())

            # Rising edge: reset controller/timing
            if enabled and not self._was_enabled:
                self._reset()

                # Preload previous measurement to reduce first-step derivative kick
                try:
                    e0 = float(self._err.get())
                except Exception:
                    e0 = 0.0
                self._prev_meas = e0
                self._prev_err = e0

            self._was_enabled = enabled

            v = float(self._vnom.get())

            if not enabled:
                # Follow OFF -> symmetric setpoints (straight)
                self._spL.put(v)
                self._spR.put(v)

                if self._dv_out is not None:
                    self._dv_out.put(0.0)

                # Keep line loop integrator clean while disabled
                self._i_term = 0.0
                self._state = S0_IDLE
                yield self._state
                continue

            self._state = S1_RUN

            # Get line error + validity (share mode OR optional weighted-sensor mode)
            line_valid, err = self._get_line_measurement(v)

            if not line_valid:
                # Lost line -> reset I and drive straight
                self._i_term = 0.0
                self._spL.put(v)
                self._spR.put(v)
                if self._dv_out is not None:
                    self._dv_out.put(0.0)
                yield self._state
                continue

            # Time step in seconds (microsecond timing)
            now_us = ticks_us()
            dt_us = ticks_diff(now_us, self._prev_t_us)
            self._prev_t_us = now_us
            dt = (dt_us / 1_000_000.0) if dt_us > 0 else 0.0

            # Gains
            kp = float(self._Kp.get())
            ki = float(self._Ki.get())
            kd = float(self._Kd.get()) if self._Kd is not None else 0.0

            # Derivative term
            if dt > 0:
                if self._deriv_on_measurement:
                    # D on measurement (measurement here is line error signal)
                    derr = -(err - self._prev_meas) / dt
                else:
                    derr = (err - self._prev_err) / dt
            else:
                derr = 0.0

            P = kp * err
            D = kd * derr

            # Active saturation (fixed or dynamic)
            sat = self._active_sat(v)
            sat_enabled = (sat is not None) and (sat > 0)

            # Candidate integral update
            if dt > 0:
                i_candidate = self._i_term + err * dt
            else:
                i_candidate = self._i_term

            # Conditional integration anti-windup
            u_unsat = P + ki * self._i_term + D
            u_candidate = P + ki * i_candidate + D

            if sat_enabled:
                if -sat <= u_candidate <= sat:
                    self._i_term = i_candidate
                    dv = u_candidate
                else:
                    # Don't integrate deeper into saturation
                    dv = u_unsat

                # Final clamp
                if dv > sat:
                    dv = sat
                elif dv < -sat:
                    dv = -sat
            else:
                self._i_term = i_candidate
                dv = u_candidate

            # Mix to wheel setpoints (preserve your current convention by default)
            if self._output_sign >= 0:
                spL = v - dv
                spR = v + dv
            else:
                # Optional sign flip if you ever need it
                spL = v + dv
                spR = v - dv

            # Clamp setpoints
            spL = self._clamp(spL, self._sp_min, self._sp_max)
            spR = self._clamp(spR, self._sp_min, self._sp_max)

            # Publish
            self._spL.put(float(spL))
            self._spR.put(float(spR))
            if self._dv_out is not None:
                self._dv_out.put(float(dv))

            # Save state for next iteration
            self._prev_err = err
            self._prev_meas = err  # "measurement" here is the line-error signal

            yield self._state