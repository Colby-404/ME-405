import pyb
from time import ticks_us, ticks_diff


class encoder:

    def __init__(self, tim, chA_pin, chB_pin, period=65535, invert=False,
                 counts_per_scaled_unit=6.54809, counts_per_rad_unit=229.1831):
        self.sign = -1 if invert else 1
        self.counts_per_scaled_unit = counts_per_scaled_unit
        self.counts_per_rad_unit = counts_per_rad_unit

        self.position = 0
        self.prev_count = 0
        self.delta = 0
        self.dt_us = 0
        self.prev_time = ticks_us()

        self.period = period
        self.half_period = (period + 1) // 2

        if isinstance(tim, int):
            self.tim = pyb.Timer(tim, prescaler=0, period=period)
        else:
            self.tim = tim

        self.pinA = pyb.Pin(chA_pin, pyb.Pin.AF_PP)
        self.pinB = pyb.Pin(chB_pin, pyb.Pin.AF_PP)
        self.ch1 = self.tim.channel(1, pyb.Timer.ENC_AB, pin=self.pinA)
        self.ch2 = self.tim.channel(2, pyb.Timer.ENC_AB, pin=self.pinB)

        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()

    def update(self):
        now = ticks_us()
        self.dt_us = ticks_diff(now, self.prev_time)
        self.prev_time = now

        cur_count = self.tim.counter()
        raw_delta = cur_count - self.prev_count

        if raw_delta > self.half_period:
            raw_delta -= (self.period + 1)
        elif raw_delta < -self.half_period:
            raw_delta += (self.period + 1)

        self.delta = raw_delta
        self.position += raw_delta
        self.prev_count = cur_count

    def get_position(self):
        return self.sign * self.position

    def get_velocity(self):
        if self.dt_us == 0:
            return 0.0
        return self.sign * (self.delta * 1e6 / self.dt_us)

    def get_position_scaled(self):
        return self.get_position() / self.counts_per_scaled_unit

    def get_velocity_scaled(self):
        return self.get_velocity() / self.counts_per_scaled_unit

    def get_position_rad(self):
        return self.get_position() / self.counts_per_rad_unit

    def get_velocity_rad(self):
        return self.get_velocity() / self.counts_per_rad_unit

    def get_position_counts(self):
        return self.get_position()

    def get_velocity_counts(self):
        return self.get_velocity()

    def zero(self, reset_hw_counter=False):
        if reset_hw_counter:
            try:
                self.tim.counter(0)
                self.prev_count = 0
            except Exception:
                self.prev_count = self.tim.counter()
        else:
            self.prev_count = self.tim.counter()

        self.position = 0
        self.delta = 0
        self.dt_us = 0
        self.prev_time = ticks_us()


Encoder = encoder
