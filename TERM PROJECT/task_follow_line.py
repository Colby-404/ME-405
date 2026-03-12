# task_follow_line.py
# Encoder-triggered scripted line follow for ME405 Romi
#
# Normal mode:
#   - uses line_err for steering
#   - if the line is lost for more than line_lost_confirm_ms, it stops safely
#   - when the line is found again for line_found_confirm_ms, it resumes
#   - when total encoder travel reaches tune_trigger_counts, the state changes
#     to TUNE_ZONE so you can adjust wheel Kp/Ki while still line-following
#
# Scripted mode (encoder-only, runs once):
#   1) small right turn
#   2) straight
#   3) 90-ish right turn
#   4) straight

import micropython

try:
    import time
    ticks_us = time.ticks_us
    ticks_diff = time.ticks_diff
except AttributeError:
    from utime import ticks_us, ticks_diff

from task_share import Share

S0_IDLE        = micropython.const(0)
S1_RUN         = micropython.const(1)
S2_TUNE_ZONE   = micropython.const(2)
S3_TURN_SMALL  = micropython.const(3)
S4_FWD_1       = micropython.const(4)
S5_TURN_90     = micropython.const(5)
S6_FWD_2       = micropython.const(6)
S7_LOST_STOP   = micropython.const(7)
S8_SCRIPT_EXIT = micropython.const(8)


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

                 # legacy / compatibility args
                 heading_deg: Share = None,
                 posL_meas: Share = None,
                 posR_meas: Share = None,
                 use_line_recovery: bool = False,
                 enable_encoder_script: bool = True,
                 lost_forward_counts: float = 150.0,
                 turn_right_deg: float = 90.0,
                 stage1_forward1_counts: float = 150.0,
                 stage1_turn_half_deg: float = 180.0,
                 stage1_forward2_counts: float = 150.0,
                 heading_tol_deg: float = 3.0,

                 # encoder-script tuning
                 recovery_fwd_speed: float = None,
                 recovery_turn_speed: float = 450.0,
                 line_lost_confirm_ms: float = 120.0,
                 line_found_confirm_ms: float = 120.0,
                 tune_trigger_counts: float = 8000.0,
                 stage0_trigger_counts: float = 10000.0,
                 stage1_trigger_counts: float = 7000.0,
                 small_right_counts: float = 120.0,
                 small_right2_counts=None,
                 stage0_forward1_counts: float = 500.0,
                 stage0_turn2_counts: float = 260.0,
                 stage0_forward2_counts: float = 600.0,

                 # debug
                 script_state_share: Share = None,
                 script_total_counts_share: Share = None,
                 script_segment_counts_share: Share = None,

                 # optional PID extras
                 Kd_line: Share = None,
                 deriv_on_measurement: bool = True,
                 output_sign: int = 1,
                 dynamic_sat: bool = False,
                 sat_ratio: float = 0.75,

                 # optional sensor centroid mode
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

        self._en = enable_follow
        self._vnom = v_nom
        self._Kp = Kp_line
        self._Ki = Ki_line
        self._Kd = Kd_line
        self._err = line_err
        self._spL = spL
        self._spR = spR
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
        self._prev_t_us = self._start_t_us
        self._start_sync_cycles = 0

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

        self._posL_meas = posL_meas
        self._posR_meas = posR_meas
        self._enable_encoder_script = bool(enable_encoder_script)

        self._recovery_fwd_speed = recovery_fwd_speed
        self._recovery_turn_speed = float(recovery_turn_speed)

        self._line_lost_confirm_us = int(float(line_lost_confirm_ms) * 1000.0)
        self._line_found_confirm_us = int(float(line_found_confirm_ms) * 1000.0)

        self._tune_trigger_counts = float(tune_trigger_counts)
        self._stage0_trigger_counts = float(stage0_trigger_counts)
        self._small_right_counts = float(small_right_counts)
        self._stage0_forward1_counts = float(stage0_forward1_counts)
        self._stage0_turn2_counts = float(stage0_turn2_counts)
        self._stage0_forward2_counts = float(stage0_forward2_counts)

        self._script_state_share = script_state_share
        self._script_total_counts_share = script_total_counts_share
        self._script_segment_counts_share = script_segment_counts_share

        self._start_posL = 0
        self._start_posR = 0
        self._segment_start_posL = 0
        self._segment_start_posR = 0

        self._script_started = False
        self._script_done = False

        self._lost_since_us = None
        self._found_since_us = None
        self._last_valid_err = 0.0

    def _reset(self):
        self._i_term = 0.0
        self._prev_err = 0.0
        self._prev_meas = 0.0
        self._last_valid_err = 0.0
        self._prev_t_us = ticks_us()
        self._start_t_us = self._prev_t_us

        self._start_posL = self._get_posL()
        self._start_posR = self._get_posR()
        self._segment_start_posL = self._start_posL
        self._segment_start_posR = self._start_posR

        self._script_started = False
        self._script_done = False

        self._lost_since_us = None
        self._found_since_us = None
        self._start_sync_cycles = 2

    def _clamp(self, x, lo, hi):
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

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

    def _set_segment_start_here(self):
        self._segment_start_posL = self._get_posL()
        self._segment_start_posR = self._get_posR()

    def _avg_counts_from(self, refL, refR):
        dL = abs(self._get_posL() - refL)
        dR = abs(self._get_posR() - refR)
        return 0.5 * (dL + dR)

    def _avg_counts_since_start(self):
        return self._avg_counts_from(self._start_posL, self._start_posR)

    def _avg_counts_since_segment(self):
        return self._avg_counts_from(self._segment_start_posL, self._segment_start_posR)

    def _script_ready(self):
        return self._enable_encoder_script and (self._posL_meas is not None) and (self._posR_meas is not None)

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
        mean = 0.5 * (left_mean + right_mean)
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

    def _update_debug(self, total_counts, segment_counts):
        if self._script_state_share is not None:
            try:
                self._script_state_share.put(int(self._state))
            except Exception:
                pass

        if self._script_total_counts_share is not None:
            try:
                self._script_total_counts_share.put(float(total_counts))
            except Exception:
                pass

        if self._script_segment_counts_share is not None:
            try:
                self._script_segment_counts_share.put(float(segment_counts))
            except Exception:
                pass

    def _command_stop(self):
        self._spL.put(0.0)
        self._spR.put(0.0)
        if self._dv_out is not None:
            self._dv_out.put(0.0)

    def _command_turn_right(self, turn_speed):
        self._spL.put(+turn_speed)
        self._spR.put(-turn_speed)
        if self._dv_out is not None:
            self._dv_out.put(-turn_speed)

    def _command_forward(self, fwd_speed):
        self._spL.put(+fwd_speed)
        self._spR.put(+fwd_speed)
        if self._dv_out is not None:
            self._dv_out.put(0.0)

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
                self._last_valid_err = e0

            self._was_enabled = enabled
            v = float(self._vnom.get())

            if not enabled:
                self._command_stop()
                self._i_term = 0.0
                self._state = S0_IDLE
                self._update_debug(0.0, 0.0)
                yield self._state
                continue

            if self._start_sync_cycles > 0:
                self._start_posL = self._get_posL()
                self._start_posR = self._get_posR()
                self._segment_start_posL = self._start_posL
                self._segment_start_posR = self._start_posR
                self._start_sync_cycles -= 1

            raw_valid, err = self._get_line_measurement(v)
            now_us = ticks_us()

            counts_from_start = self._avg_counts_since_start()
            counts_in_segment = self._avg_counts_since_segment()

            tune_zone_active = (counts_from_start >= self._tune_trigger_counts) and (not self._script_started)

            # -----------------------------
            # Script exit: return to normal follow
            # -----------------------------
            if self._state == S8_SCRIPT_EXIT:
                self._script_done = True
                self._state = S1_RUN
                self._i_term = 0.0
                self._prev_err = err
                self._prev_meas = err
                self._prev_t_us = now_us
                self._update_debug(counts_from_start, 0.0)
                yield self._state
                continue

            # -----------------------------
            # Scripted encoder-only sequence
            # -----------------------------
            if self._state == S3_TURN_SMALL:
                turn = abs(float(self._recovery_turn_speed))
                self._command_turn_right(turn)
                self._update_debug(counts_from_start, counts_in_segment)

                if counts_in_segment >= self._small_right_counts:
                    self._set_segment_start_here()
                    self._state = S4_FWD_1

                yield self._state
                continue

            if self._state == S4_FWD_1:
                fwd = abs(float(v)) if self._recovery_fwd_speed is None else abs(float(self._recovery_fwd_speed))
                self._command_forward(fwd)
                self._update_debug(counts_from_start, counts_in_segment)

                if counts_in_segment >= self._stage0_forward1_counts:
                    self._set_segment_start_here()
                    self._state = S5_TURN_90

                yield self._state
                continue

            if self._state == S5_TURN_90:
                turn = abs(float(self._recovery_turn_speed))
                self._command_turn_right(turn)
                self._update_debug(counts_from_start, counts_in_segment)

                if counts_in_segment >= self._stage0_turn2_counts:
                    self._set_segment_start_here()
                    self._state = S6_FWD_2

                yield self._state
                continue

            if self._state == S6_FWD_2:
                fwd = abs(float(v)) if self._recovery_fwd_speed is None else abs(float(self._recovery_fwd_speed))
                self._command_forward(fwd)
                self._update_debug(counts_from_start, counts_in_segment)

                if counts_in_segment >= self._stage0_forward2_counts:
                    self._state = S8_SCRIPT_EXIT

                yield self._state
                continue

            # -----------------------------
            # Start scripted sequence once, at encoder trigger
            # -----------------------------
            if self._script_ready() and (not self._script_started) and (not self._script_done):
                if counts_from_start >= self._stage0_trigger_counts:
                    self._script_started = True
                    self._i_term = 0.0
                    self._set_segment_start_here()
                    self._state = S3_TURN_SMALL
                    self._update_debug(counts_from_start, 0.0)
                    yield self._state
                    continue

            # -----------------------------
            # Normal line-follow / tune-zone
            # -----------------------------
            normal_state = S2_TUNE_ZONE if tune_zone_active else S1_RUN

            if raw_valid:
                self._last_valid_err = err
                self._lost_since_us = None

                if self._state == S7_LOST_STOP:
                    if self._found_since_us is None:
                        self._found_since_us = now_us
                    elif ticks_diff(now_us, self._found_since_us) >= self._line_found_confirm_us:
                        self._state = normal_state
                        self._i_term = 0.0
                        self._prev_err = err
                        self._prev_meas = err
                        self._prev_t_us = now_us
                        self._found_since_us = None
                else:
                    self._found_since_us = None

                ctrl_err = err

            else:
                self._found_since_us = None

                if self._lost_since_us is None:
                    self._lost_since_us = now_us

                lost_us = ticks_diff(now_us, self._lost_since_us)

                if (self._state == S7_LOST_STOP) or (lost_us >= self._line_lost_confirm_us):
                    self._state = S7_LOST_STOP
                    self._i_term = 0.0
                    self._prev_err = self._last_valid_err
                    self._prev_meas = self._last_valid_err
                    self._prev_t_us = now_us
                    self._command_stop()
                    self._update_debug(counts_from_start, 0.0)
                    yield self._state
                    continue

                # short dropout: hold last valid error briefly
                ctrl_err = self._last_valid_err

            if self._state != normal_state:
                self._i_term = 0.0
                self._prev_err = ctrl_err
                self._prev_meas = ctrl_err
                self._prev_t_us = now_us

            self._state = normal_state

            dt_us = ticks_diff(now_us, self._prev_t_us)
            self._prev_t_us = now_us
            dt = (dt_us / 1_000_000.0) if dt_us > 0 else 0.0

            kp = float(self._Kp.get())
            ki = float(self._Ki.get())
            kd = float(self._Kd.get()) if self._Kd is not None else 0.0

            if dt > 0:
                if self._deriv_on_measurement:
                    derr = -(ctrl_err - self._prev_meas) / dt
                else:
                    derr = (ctrl_err - self._prev_err) / dt
            else:
                derr = 0.0

            P = kp * ctrl_err
            D = kd * derr
            sat = self._active_sat(v)
            sat_enabled = (sat is not None) and (sat > 0)

            if dt > 0:
                i_candidate = self._i_term + ctrl_err * dt
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

            self._prev_err = ctrl_err
            self._prev_meas = ctrl_err
            self._update_debug(counts_from_start, 0.0)

            yield self._state