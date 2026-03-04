'''
Quadrature encoder interface class (hybrid / compatibility version)

This version is designed to:
- Work with your current ME405 codebase (main.py, task_motor.py, shares/queues)
- Preserve controller/logging defaults in COUNTS and COUNTS/S
- Add optional scaled and rad-based outputs 
- Support timer number OR timer object in constructor
- Support sign inversion cleanly via invert=True/False

Default behavior (safe for your current code):
    get_position() -> counts (int-like)
    get_velocity() -> counts/s (float)
'''

import pyb
from time import ticks_us, ticks_diff


class encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self,
                 tim,
                 chA_pin,
                 chB_pin,
                 period=65535,
                 invert=False,
                 counts_per_scaled_unit=6.54809,   # Scaled Units (e.g., degrees) per count
                 counts_per_rad_unit=229.1831):    # Rad-like Units (e.g., radians) per count
        '''
        Initializes an Encoder object.

        Parameters
        ----------
        tim : int or pyb.Timer
            Timer number (e.g., 1, 2) or an already-created Timer object
        chA_pin, chB_pin : str
            Pin names for encoder A/B channels (e.g., 'PA0', 'PA1')
        period : int
            Timer period (default 65535 for 16-bit full range)
        invert : bool
            True flips reported position/velocity sign (useful for one wheel)
        counts_per_scaled_unit : float
            Conversion factor used by get_position_scaled()/get_velocity_scaled()
        counts_per_rad_unit : float
            Conversion factor used by get_position_rad()/get_velocity_rad()
        '''

        # Sign convention (clean + explicit)
        self.sign = -1 if invert else 1

        # Conversion constants for optional scaled/rad outputs
        self.counts_per_scaled_unit = counts_per_scaled_unit
        self.counts_per_rad_unit = counts_per_rad_unit

        # State
        self.position = 0       # accumulated raw counts (signed only in getter)
        self.prev_count = 0
        self.delta = 0
        self.dt_us = 0          # microseconds between updates
        self.prev_time = ticks_us()

        # Wraparound config
        self.period = period
        self.half_period = (period + 1) // 2

        # Accept timer number or existing timer object
        if isinstance(tim, int):
            self.tim = pyb.Timer(tim, prescaler=0, period=period)
        else:
            self.tim = tim

        # Encoder pins
        self.pinA = pyb.Pin(chA_pin, pyb.Pin.AF_PP)
        self.pinB = pyb.Pin(chB_pin, pyb.Pin.AF_PP)

        # Put timer in encoder mode
        self.ch1 = self.tim.channel(1, pyb.Timer.ENC_AB, pin=self.pinA)
        self.ch2 = self.tim.channel(2, pyb.Timer.ENC_AB, pin=self.pinB)

        # Initialize references
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()

    def update(self):
        '''
        Runs one update step:
        - Computes dt
        - Reads timer counter
        - Handles wraparound
        - Updates accumulated position
        '''
        now = ticks_us()
        self.dt_us = ticks_diff(now, self.prev_time)
        self.prev_time = now

        cur_count = self.tim.counter()
        raw_delta = cur_count - self.prev_count

        # Wraparound correction (generic for configured period)
        if raw_delta > self.half_period:
            raw_delta -= (self.period + 1)
        elif raw_delta < -self.half_period:
            raw_delta += (self.period + 1)

        self.delta = raw_delta
        self.position += raw_delta
        self.prev_count = cur_count

    # ------------------------------------------------------------------
    # PRIMARY METHODS
    # ------------------------------------------------------------------

    def get_position(self):
        '''
        Returns position in COUNTS (compatible with Share("l") in your code)
        '''
        return self.sign * self.position

    def get_velocity(self):
        '''
        Returns velocity in COUNTS/S (compatible with your PID setpoints/shares)
        '''
        if self.dt_us == 0:
            return 0.0
        return self.sign * (self.delta * 1e6 / self.dt_us)

    # ------------------------------------------------------------------
    # OPTIONAL EXTRA METHODS (added encoder functionality)
    # ------------------------------------------------------------------

    def get_position_scaled(self):
        '''
        Returns position in "scaled units" using conversion
        (position / 6.54809 by default)
        '''
        return self.get_position() / self.counts_per_scaled_unit

    def get_velocity_scaled(self):
        '''
        Returns velocity in "scaled units per second"
        (velocity_counts_per_s / 6.54809 by default)
        '''
        return self.get_velocity() / self.counts_per_scaled_unit

    def get_position_rad(self):
        '''
        Returns position in rad-like units using conversion
        (position / 229.1831 by default)
        '''
        return self.get_position() / self.counts_per_rad_unit

    def get_velocity_rad(self):
        '''
        Returns velocity in rad/s-like units using conversion
        (velocity_counts_per_s / 229.1831 by default)
        '''
        return self.get_velocity() / self.counts_per_rad_unit

    # Helpful aliases if you want explicit naming later
    def get_position_counts(self):
        return self.get_position()

    def get_velocity_counts(self):
        return self.get_velocity()

    def zero(self, reset_hw_counter=False):
        '''
        Zero encoder state.

        Parameters
        ----------
        reset_hw_counter : bool
            False (default): leaves hardware timer counter alone, resets software reference
                            (safe/default for your current task flow)
            True: resets timer counter to 0
        '''
        if reset_hw_counter:
            try:
                self.tim.counter(0)
                self.prev_count = 0
            except Exception:
                # Fallback safely if timer counter reset is unavailable
                self.prev_count = self.tim.counter()
        else:
            self.prev_count = self.tim.counter()

        self.position = 0
        self.delta = 0
        self.dt_us = 0
        self.prev_time = ticks_us()


# Alias for compatibility with code that imports Encoder (capital E)
Encoder = encoder