# task_follow_line.py
# Outer-loop line-following task for ME405 Romi
#
# Normal mode:
#   - uses line_err for steering
#   - writes spL / spR
#
# Scripted mode:
#   - line_ok is NOT used to trigger scripts
#   - stage 0 starts when encoder travel since follow-enable reaches stage0_trigger_counts
#   - stage 1 starts when encoder travel since follow-enable reaches stage1_trigger_counts
#
# Scripted actions:
#   Stage 0: go forward -> turn right -> return to normal line follow
#   Stage 1: go forward -> 180 -> 180 -> go forward -> return to normal line follow

import micropython

try:
    import time
    ticks_us = time.ticks_us
    ticks_diff = time.ticks_diff
except AttributeError:
    from utime import ticks_us, ticks_diff

from task_share import Share

S0_IDLE           = micropython.const(0)
S1_RUN            = micropython.const(1)
S2_STAGE0_FWD     = micropython.const(2)
S3_STAGE0_TURN90  = micropython.const(3)
S4_STAGE0_EXIT    = micropython.const(4)
S5_STAGE1_FWD1    = micropython.const(5)
S6_STAGE1_TURN_A  = micropython.const(6)
S7_STAGE1_TURN_B  = micropython.const(7)
S8_STAGE1_FWD2    = micropython.const(8)
S9_STAGE1_EXIT    = micropython.const(9)


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
                 period_ms: int = 20,

                 line_ok: Share = None,
                 sp_min: float = -3000.0,
                 sp_max: float = 3000.0,

                 heading_deg: Share = None,
                 posL_meas: Share = None,
                 posR_meas: Share = None,
                 use_line_recovery: bool = False,

                 lost_forward_counts: float = 150.0,
                 turn_right_deg: float = 90.0,

                 stage1_forward1_counts: float = 150.0,
                 stage1_turn_half_deg: float = 180.0,
                 stage1_forward2_counts: float = 150.0,

                 heading_tol_deg: float = 3.0,
                 recovery_fwd_speed: float = None,
                 recovery_turn_speed: float = 450.0,
                 line_lost_confirm_ms: float = 150.0,
                 line_found_confirm_ms: float = 120.0,

                 stage0_trigger_counts: float = 3000.0,
                 stage1_trigger_counts: float = 7000.0,

                 Kd_line: Share = None,
                 deriv_on_measurement: bool = True,
                 output_sign: int = 1,
                 dynamic_sat: bool = False,
                 sat_ratio: float = 0.75,
                 centroid_q=None,
                 centroid_t_q=None,
                 centroid_out: Share = None,
                 llf=None,
                 rlf=None,
                 inner_weight: float = 8.0,
                 middle_weight: float = 16.0,
                 outer_weight: float = 24.0,
                 sensor_total_min: float = 1.0,
                 sensor_mean_min: float = 1.0,
                 sensor_error_scale_with_speed: bool = True):

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

        self._sat_dv = float(sat_dv) if sat_dv is not None else None
        self._dynamic_sat = bool(dynamic_sat)
        self._sat_ratio = float(sat_ratio)
        self._sp_min = float(sp_min)
        self._sp_max = float(sp_max)
        self._output_sign = 1 if output_sign >= 0 else -1

        self._state = S0_IDLE
        self._period_ms = int(period_ms)
        self._was_enabled = False
        self._start_t_us = ticks_us()
        self._prev_t_us = ticks_us()

        self._i_term = 0.0
        self._prev_err = 0.0
        self._prev_meas = 0.0
        self._deriv_on_measurement = bool(deriv_on_measurement)

        self._centroid_q = centroid_q
        self._centroid_t_q = centroid_t_q
        self._centroid_out = centroid_out

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

        self._heading = heading_deg
        self._posL_meas = posL_meas
        self._posR_meas = posR_meas
        self._use_line_recovery = bool(use_line_recovery)

        self._lost_forward_counts = float(lost_forward_counts)
        self._turn_right_deg = float(turn_right_deg)
        self._stage1_forward1_counts = float(stage1_forward1_counts)
        self._stage1_turn_half_deg = float(stage1_turn_half_deg)
        self._stage1_forward2_counts = float(stage1_forward2_counts)
        self._heading_tol_deg = float(heading_tol_deg)
        self._recovery_fwd_speed = recovery_fwd_speed
        self._recovery_turn_speed = float(recovery_turn_speed)

        # accepted for compatibility, not used for script triggering anymore
        self._line_lost_confirm_ms = float(line_lost_confirm_ms)
        self._line_found_confirm_ms = float(line_found_confirm_ms)

        self._stage0_trigger_counts = float(stage0_trigger_counts)
        self._stage1_trigger_counts = float(stage1_trigger_counts)

        self._course_stage = 0
        self._lost_heading = 0.0
        self._lost_posL = 0
        self._lost_posR = 0
        self._target_heading = 0.0
        self._start_posL = 0
        self._start_posR = 0
        self._stage0_started = False
        self._stage1_started = False

    def _reset(self):
        self._i_term = 0.0
        self._prev_err = 0.0
        self._prev_meas = 0.0
        self._prev_t_us = ticks_us()
        self._start_t_us = self._prev_t_us
        self._lost_heading = 0.0
        self._lost_posL = 0
        self._lost_posR = 0
        self._target_heading = 0.0
        self._start_posL = self._get_posL()
        self._start_posR = self._get_posR()
        self._stage0_started = False
        self._stage1_started = False
        self._course_stage = 0

    def _clamp(self, x, lo, hi):
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def _wrap_deg(self, ang):
        while ang > 180.0:
            ang -= 360.0
        while ang < -180.0:
            ang += 360.0
        return ang

    def _get_heading_deg(self):
        if self._heading is None:
            return 0.0
        try:
            return float(self._heading.get())
        except Exception:
            return 0.0

    def _get_posL(self):
        if self._posL_meas is None:
            return 0
        try:
            return int(self._posL_meas.get())
        except Exception:
            return 0

    def _get_posR(self):
        if self._posR_meas is None:
            return 0
        try:
            return int(self._posR_meas.get())
        except Exception:
            return 0

    def _latch_script_start(self):
        self._lost_heading = self._get_heading_deg()
        self._lost_posL = self._get_posL()
        self._lost_posR = self._get_posR()

    def _avg_counts_since_loss(self):
        dL = abs(self._get_posL() - self._lost_posL)
        dR = abs(self._get_posR() - self._lost_posR)
        return 0.5 * (dL + dR)

    def _avg_counts_since_start(self):
        dL = self._get_posL() - self._start_posL
        dR = self._get_posR() - self._start_posR
        return abs(0.5 * (dL + dR))

    def _recovery_ready(self):
        return (
            self._use_line_recovery and
            (self._heading is not None) and
            (self._posL_meas is not None) and
            (self._posR_meas is not None)
        )

    def _active_sat(self, v_nom_value):
        if self._dynamic_sat:
            return abs(float(v_nom_value)) * self._sat_ratio
        return self._sat_dv

    def _log_centroid(self, centroid_value):
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
        try:
            Lin, Lmid, Lout = self._llf.get_values()
            Rin, Rmid, Rout = self._rlf.get_values()
        except Exception:
            return (False, 0.0, 0.0)
        left = self._w_in * Lin + self._w_mid * Lmid + self._w_out * Lout
        right = self._w_in * Rin + self._w_mid * Rmid + self._w_out * Rout
        raw_total = (Lin + Lmid + Lout + Rin + Rmid + Rout)
        if raw_total <= 0:
            return (False, 0.0, 0.0)
        left_mean = left / self._w_total
        right_mean = right / self._w_total
        mean = (left_mean + right_mean) * 0.5
        centroid = (left + right) / raw_total
        valid = (raw_total >= self._sensor_total_min) and (mean >= self._sensor_mean_min)
        if not valid:
            return (False, 0.0, centroid)
        error = left - right
        if mean <= 0:
            err = 0.0
        else:
            if self._sensor_error_scale_with_speed:
                err = (error * float(v_nom_value)) / mean
            else:
                err = error / mean
        return (True, float(err), float(centroid))

    def _get_line_measurement(self, v_nom_value):
        if self._use_sensor_mode:
            valid, err, centroid = self._compute_error_from_weighted_sensors(v_nom_value)
            try:
                self._err.put(float(err))
            except Exception:
                pass
            if self._line_ok is not None:
                try:
                    self._line_ok.put(1 if valid else 0)
                except Exception:
                    pass
            self._log_centroid(centroid)
            return valid, float(err)
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
        if (self._centroid_q is not None) or (self._centroid_t_q is not None) or (self._centroid_out is not None):
            self._log_centroid(err)
        return valid, err

    def run(self):
        while True:
            enabled = bool(self._en.get())
            if enabled and not self._was_enabled:
                self._reset()
                try:
                    e0 = float(self._err.get())
                except Exception:
                    e0 = 0.0
                self._prev_meas = e0
                self._prev_err = e0
            self._was_enabled = enabled
            v = float(self._vnom.get())

            if not enabled:
                self._spL.put(v)
                self._spR.put(v)
                if self._dv_out is not None:
                    self._dv_out.put(0.0)
                self._i_term = 0.0
                self._state = S0_IDLE
                yield self._state
                continue

            _raw_line_valid, err = self._get_line_measurement(v)
            counts_from_start = self._avg_counts_since_start()

            if self._state == S4_STAGE0_EXIT:
                self._course_stage = 1
                self._state = S1_RUN
                self._i_term = 0.0
                self._prev_err = err
                self._prev_meas = err
                self._prev_t_us = ticks_us()
                yield self._state
                continue

            if self._state == S9_STAGE1_EXIT:
                self._course_stage = 2
                self._state = S1_RUN
                self._i_term = 0.0
                self._prev_err = err
                self._prev_meas = err
                self._prev_t_us = ticks_us()
                yield self._state
                continue

            if self._state == S2_STAGE0_FWD:
                fwd = abs(float(v)) if self._recovery_fwd_speed is None else abs(float(self._recovery_fwd_speed))
                self._spL.put(fwd)
                self._spR.put(fwd)
                if self._dv_out is not None:
                    self._dv_out.put(0.0)
                if self._avg_counts_since_loss() >= self._lost_forward_counts:
                    self._target_heading = self._wrap_deg(self._lost_heading - self._turn_right_deg)
                    self._state = S3_STAGE0_TURN90
                yield self._state
                continue

            if self._state == S3_STAGE0_TURN90:
                turn = abs(self._recovery_turn_speed)
                cur_h = self._get_heading_deg()
                hdg_err = self._wrap_deg(self._target_heading - cur_h)
                self._spL.put(+turn)
                self._spR.put(-turn)
                if self._dv_out is not None:
                    self._dv_out.put(-turn)
                if abs(hdg_err) <= self._heading_tol_deg:
                    self._state = S4_STAGE0_EXIT
                yield self._state
                continue

            if self._state == S5_STAGE1_FWD1:
                fwd = abs(float(v)) if self._recovery_fwd_speed is None else abs(float(self._recovery_fwd_speed))
                self._spL.put(fwd)
                self._spR.put(fwd)
                if self._dv_out is not None:
                    self._dv_out.put(0.0)
                if self._avg_counts_since_loss() >= self._stage1_forward1_counts:
                    self._target_heading = self._wrap_deg(self._lost_heading - self._stage1_turn_half_deg)
                    self._state = S6_STAGE1_TURN_A
                yield self._state
                continue

            if self._state == S6_STAGE1_TURN_A:
                turn = abs(self._recovery_turn_speed)
                cur_h = self._get_heading_deg()
                hdg_err = self._wrap_deg(self._target_heading - cur_h)
                self._spL.put(+turn)
                self._spR.put(-turn)
                if self._dv_out is not None:
                    self._dv_out.put(-turn)
                if abs(hdg_err) <= self._heading_tol_deg:
                    self._target_heading = self._wrap_deg(self._target_heading - self._stage1_turn_half_deg)
                    self._state = S7_STAGE1_TURN_B
                yield self._state
                continue

            if self._state == S7_STAGE1_TURN_B:
                turn = abs(self._recovery_turn_speed)
                cur_h = self._get_heading_deg()
                hdg_err = self._wrap_deg(self._target_heading - cur_h)
                self._spL.put(+turn)
                self._spR.put(-turn)
                if self._dv_out is not None:
                    self._dv_out.put(-turn)
                if abs(hdg_err) <= self._heading_tol_deg:
                    self._lost_posL = self._get_posL()
                    self._lost_posR = self._get_posR()
                    self._state = S8_STAGE1_FWD2
                yield self._state
                continue

            if self._state == S8_STAGE1_FWD2:
                fwd = abs(float(v)) if self._recovery_fwd_speed is None else abs(float(self._recovery_fwd_speed))
                self._spL.put(fwd)
                self._spR.put(fwd)
                if self._dv_out is not None:
                    self._dv_out.put(0.0)
                if self._avg_counts_since_loss() >= self._stage1_forward2_counts:
                    self._state = S9_STAGE1_EXIT
                yield self._state
                continue

            if self._recovery_ready():
                if (self._course_stage == 0) and (not self._stage0_started):
                    if counts_from_start >= self._stage0_trigger_counts:
                        self._stage0_started = True
                        self._i_term = 0.0
                        self._latch_script_start()
                        self._state = S2_STAGE0_FWD
                        yield self._state
                        continue
                elif (self._course_stage == 1) and (not self._stage1_started):
                    if counts_from_start >= self._stage1_trigger_counts:
                        self._stage1_started = True
                        self._i_term = 0.0
                        self._latch_script_start()
                        self._state = S5_STAGE1_FWD1
                        yield self._state
                        continue

            if self._state != S1_RUN:
                self._i_term = 0.0
                self._prev_err = err
                self._prev_meas = err
                self._prev_t_us = ticks_us()

            self._state = S1_RUN
            now_us = ticks_us()
            dt_us = ticks_diff(now_us, self._prev_t_us)
            self._prev_t_us = now_us
            dt = (dt_us / 1_000_000.0) if dt_us > 0 else 0.0

            kp = float(self._Kp.get())
            ki = float(self._Ki.get())
            kd = float(self._Kd.get()) if self._Kd is not None else 0.0

            if dt > 0:
                if self._deriv_on_measurement:
                    derr = -(err - self._prev_meas) / dt
                else:
                    derr = (err - self._prev_err) / dt
            else:
                derr = 0.0

            P = kp * err
            D = kd * derr
            sat = self._active_sat(v)
            sat_enabled = (sat is not None) and (sat > 0)

            if dt > 0:
                i_candidate = self._i_term + err * dt
            else:
                i_candidate = self._i_term
            u_unsat = P + ki * self._i_term + D
            u_candidate = P + ki * i_candidate + D

            if sat_enabled:
                if -sat <= u_candidate <= sat:
                    self._i_term = i_candidate
                    dv = u_candidate
                else:
                    dv = u_unsat
                if dv > sat:
                    dv = sat
                elif dv < -sat:
                    dv = -sat
            else:
                self._i_term = i_candidate
                dv = u_candidate

            if self._output_sign >= 0:
                spL = v - dv
                spR = v + dv
            else:
                spL = v + dv
                spR = v - dv

            spL = self._clamp(spL, self._sp_min, self._sp_max)
            spR = self._clamp(spR, self._sp_min, self._sp_max)
            self._spL.put(float(spL))
            self._spR.put(float(spR))
            if self._dv_out is not None:
                self._dv_out.put(float(dv))
            self._prev_err = err
            self._prev_meas = err
            yield self._state
