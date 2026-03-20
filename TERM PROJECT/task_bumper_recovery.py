# task_bumper_recovery.py

import micropython
import pyb

S0_IDLE         = micropython.const(0)
S1_DEBOUNCE     = micropython.const(1)
S2_BRAKE        = micropython.const(2)
S3_REVERSE      = micropython.const(3)
S4_TURN_LEFT    = micropython.const(4)
S5_RESUME_DELAY = micropython.const(5)
S6_WAIT_RELEASE = micropython.const(6)


class task_bumper_recovery:
    def __init__(self,
                 bmp_r_outer, bmp_r_mid, bmp_r_inner,
                 bmp_l_outer, bmp_l_mid, bmp_l_inner,
                 follow_en,
                 motorLGo, motorRGo,
                 setpointL, setpointR,
                 posL, posR,
                 pi_reset=None,
                 motion_override=None,
                 reverse_speed=180.0,
                 reverse_ticks=940.0,
                 turn_speed=160.0,
                 turn_ticks=970.0,
                 brake_ms=120,
                 resume_ms=80,
                 debounce_ms=40,
                 release_ms=60,
                 rearm_ms=250,
                 active_low=True):

        self._r0 = pyb.Pin(bmp_r_outer, pyb.Pin.IN, pyb.Pin.PULL_UP)
        self._r1 = pyb.Pin(bmp_r_mid,   pyb.Pin.IN, pyb.Pin.PULL_UP)
        self._r2 = pyb.Pin(bmp_r_inner, pyb.Pin.IN, pyb.Pin.PULL_UP)

        self._l0 = pyb.Pin(bmp_l_outer, pyb.Pin.IN, pyb.Pin.PULL_UP)
        self._l1 = pyb.Pin(bmp_l_mid,   pyb.Pin.IN, pyb.Pin.PULL_UP)
        self._l2 = pyb.Pin(bmp_l_inner, pyb.Pin.IN, pyb.Pin.PULL_UP)

        self._active_low = active_low

        self._follow_en = follow_en
        self._motorLGo = motorLGo
        self._motorRGo = motorRGo
        self._spL = setpointL
        self._spR = setpointR
        self._posL = posL
        self._posR = posR
        self._pi_reset = pi_reset
        self._motion_override = motion_override

        self._reverse_speed = abs(float(reverse_speed))
        self._reverse_ticks = abs(float(reverse_ticks))
        self._turn_speed = abs(float(turn_speed))
        self._turn_ticks = abs(float(turn_ticks))

        self._brake_ms = int(brake_ms)
        self._resume_ms = int(resume_ms)
        self._debounce_ms = int(debounce_ms)
        self._release_ms = int(release_ms)
        self._rearm_ms = int(rearm_ms)

        self._state = S0_IDLE
        self._t0 = pyb.millis()
        self._release_t0 = pyb.millis()
        self._rearm_until = pyb.millis()

        self._seg_start_L = 0
        self._seg_start_R = 0

    def _pressed(self, pin_obj):
        val = pin_obj.value()
        return (val == 0) if self._active_low else (val == 1)

    def _right_hit(self):
        return (self._pressed(self._r0) or
                self._pressed(self._r1) or
                self._pressed(self._r2))

    def _left_hit(self):
        return (self._pressed(self._l0) or
                self._pressed(self._l1) or
                self._pressed(self._l2))

    def _any_hit_raw(self):
        return self._left_hit() or self._right_hit()

    def _set_state(self, new_state):
        self._state = new_state
        self._t0 = pyb.millis()

    def _elapsed_ms(self):
        return pyb.elapsed_millis(self._t0)

    def _get_posL(self):
        try:
            return int(self._posL.get())
        except Exception:
            return 0

    def _get_posR(self):
        try:
            return int(self._posR.get())
        except Exception:
            return 0

    def _start_segment(self):
        self._seg_start_L = self._get_posL()
        self._seg_start_R = self._get_posR()

    def _avg_segment_counts(self):
        dL = abs(self._get_posL() - self._seg_start_L)
        dR = abs(self._get_posR() - self._seg_start_R)
        return 0.5 * (dL + dR)

    def _stop_robot(self):
        self._spL.put(0.0)
        self._spR.put(0.0)

    def _reverse_robot(self):
        self._spL.put(-self._reverse_speed)
        self._spR.put(-self._reverse_speed)

    def _turn_left_robot(self):
        # pivot left
        self._spL.put(-self._turn_speed)
        self._spR.put(+self._turn_speed)

    def _reset_pi(self):
        if self._pi_reset is not None:
            try:
                self._pi_reset.put(1)
            except Exception:
                pass

    def _override_on(self):
        if self._motion_override is not None:
            try:
                self._motion_override.put(1)
            except Exception:
                pass

    def _override_off(self):
        if self._motion_override is not None:
            try:
                self._motion_override.put(0)
            except Exception:
                pass

    def run(self):
        while True:
            follow_is_on = False
            try:
                follow_is_on = int(self._follow_en.get()) == 1
            except Exception:
                follow_is_on = False

            if self._state == S0_IDLE:
                if follow_is_on and pyb.elapsed_millis(self._rearm_until) >= 0:
                    if self._any_hit_raw():
                        self._set_state(S1_DEBOUNCE)

            elif self._state == S1_DEBOUNCE:
                if not self._any_hit_raw():
                    self._set_state(S0_IDLE)
                elif self._elapsed_ms() >= self._debounce_ms:
                    self._override_on()
                    self._follow_en.put(0)
                    self._motorLGo.put(1)
                    self._motorRGo.put(1)
                    self._stop_robot()
                    self._reset_pi()
                    self._set_state(S2_BRAKE)

            elif self._state == S2_BRAKE:
                self._stop_robot()
                if self._elapsed_ms() >= self._brake_ms:
                    self._start_segment()
                    self._reset_pi()
                    self._set_state(S3_REVERSE)

            elif self._state == S3_REVERSE:
                self._reverse_robot()
                if self._avg_segment_counts() >= self._reverse_ticks:
                    self._stop_robot()
                    self._reset_pi()
                    self._start_segment()
                    self._set_state(S4_TURN_LEFT)

            elif self._state == S4_TURN_LEFT:
                self._turn_left_robot()
                if self._avg_segment_counts() >= self._turn_ticks:
                    self._stop_robot()
                    self._reset_pi()
                    self._set_state(S5_RESUME_DELAY)

            elif self._state == S5_RESUME_DELAY:
                self._stop_robot()
                if self._elapsed_ms() >= self._resume_ms:
                    self._override_off()
                    self._follow_en.put(1)
                    self._release_t0 = pyb.millis()
                    self._set_state(S6_WAIT_RELEASE)

            elif self._state == S6_WAIT_RELEASE:
                if self._any_hit_raw():
                    self._release_t0 = pyb.millis()
                elif pyb.elapsed_millis(self._release_t0) >= self._release_ms:
                    self._rearm_until = pyb.millis() + self._rearm_ms
                    self._set_state(S0_IDLE)

            yield self._state