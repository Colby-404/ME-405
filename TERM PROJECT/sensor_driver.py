# sensor_driver.py — QTR-MD-08A analog line sensor (STM32 pyb, 12-bit ADC)

import pyb
import utime


class QTRSensorsAnalog:

    ADC_MAX = 4095  # STM32 pyb.ADC.read() is 12-bit

    def __init__(self, adc_pins, emitter_pin=None, invert=False):
        self.count = len(adc_pins)
        if self.count < 1:
            raise ValueError("adc_pins must contain at least 1 pin")

        # ADC channels
        self._adcs = []
        for p in adc_pins:
            pin = p if isinstance(p, pyb.Pin) else pyb.Pin(p)
            self._adcs.append(pyb.ADC(pin))

        self._emitter = None
        if emitter_pin is not None:
            ep = emitter_pin if isinstance(emitter_pin, pyb.Pin) else pyb.Pin(emitter_pin)
            self._emitter = pyb.Pin(ep, pyb.Pin.OUT_PP)
            self._emitter.high()

        self.invert = bool(invert)

        self.raw = [0] * self.count
        self.norm = [0] * self.count       # 0..1000
        self.white = [0] * self.count
        self.black = [self.ADC_MAX] * self.count
        self._have_white = False
        self._have_black = False
        self.last_position = (self.count - 1) * 1000 // 2
        self.min_reading = 20              # threshold below which norm is ignored

    def emitters_on(self):
        if self._emitter is not None:
            self._emitter.high()
            utime.sleep_us(250)

    def emitters_off(self):
        if self._emitter is not None:
            self._emitter.low()
            utime.sleep_us(250)

    def read_raw(self, oversample=1, emitters=True):
        os = int(oversample) if oversample and oversample > 0 else 1

        if emitters:
            self.emitters_on()

        for i in range(self.count):
            acc = 0
            for _ in range(os):
                acc += self._adcs[i].read()
            self.raw[i] = acc // os

        if emitters:
            self.emitters_off()

        return self.raw

    def calibrate_white(self, samples=50, oversample=1, emitters=True, settle_ms=0):
        if settle_ms > 0:
            utime.sleep_ms(settle_ms)

        sums = [0] * self.count
        s = max(1, int(samples))
        for _ in range(s):
            self.read_raw(oversample=oversample, emitters=emitters)
            for i in range(self.count):
                sums[i] += self.raw[i]

        self.white = [val // s for val in sums]
        self._have_white = True

        if not self._have_black:  # ensure denom > 0 before black is calibrated
            self.black = [min(self.ADC_MAX, w + 1) for w in self.white]

        return self.white

    def calibrate_black(self, samples=50, oversample=1, emitters=True, settle_ms=0):
        if settle_ms > 0:
            utime.sleep_ms(settle_ms)

        sums = [0] * self.count
        s = max(1, int(samples))
        for _ in range(s):
            self.read_raw(oversample=oversample, emitters=emitters)
            for i in range(self.count):
                sums[i] += self.raw[i]

        self.black = [val // s for val in sums]
        self._have_black = True

        if not self._have_white:  # ensure denom > 0 before white is calibrated
            self.white = [max(0, b - 1) for b in self.black]

        return self.black

    def calibrated_ready(self):
        return self._have_white and self._have_black

    def fix_calibration_order(self):
        for i in range(self.count):
            if self.black[i] < self.white[i]:
                self.black[i], self.white[i] = self.white[i], self.black[i]

    def read_normalized(self, oversample=1, emitters=True):
        self.read_raw(oversample=oversample, emitters=emitters)

        if not self.calibrated_ready():  # crude scale if not yet calibrated
            for i in range(self.count):
                v = (self.raw[i] * 1000) // self.ADC_MAX
                v = 1000 - v if self.invert else v
                self.norm[i] = 0 if v < 0 else (1000 if v > 1000 else v)
            return self.norm

        for i in range(self.count):
            w = self.white[i]
            b = self.black[i]

            if b < w:  # swap if calibration order was reversed
                w, b = b, w

            denom = b - w
            if denom <= 0:
                v = 0
            else:
                v = (self.raw[i] - w) * 1000 // denom

            # clamp to 0..1000
            if v < 0:
                v = 0
            elif v > 1000:
                v = 1000

            if self.invert:
                v = 1000 - v

            self.norm[i] = v

        return self.norm

    def read_line_position(self, oversample=1, emitters=True):
        self.read_normalized(oversample=oversample, emitters=emitters)

        weighted_sum = 0
        total = 0

        for i in range(self.count):
            v = self.norm[i]
            if v < self.min_reading:
                v = 0
            weighted_sum += v * (i * 1000)
            total += v

        if total == 0:
            return self.last_position

        pos = weighted_sum // total
        self.last_position = pos
        return pos

    def line_error(self, oversample=1, emitters=True):
        pos = self.read_line_position(oversample=oversample, emitters=emitters)
        center = (self.count - 1) * 1000 // 2
        return pos - center

    def emitter_test(self, oversample=1):
        on_vals = self.read_raw(oversample=oversample, emitters=True)[:]
        utime.sleep_ms(10)
        off_vals = self.read_raw(oversample=oversample, emitters=False)[:]
        print("RAW emitters ON :", on_vals)
        print("RAW emitters OFF:", off_vals)

    def diagnostics(self):
        print("QTR Analog diagnostics")
        print("count:", self.count)
        print("invert:", self.invert)
        print("have_white:", self._have_white, "have_black:", self._have_black)
        print("white:", self.white)
        print("black:", self.black)
        print("raw:", self.raw)
        print("norm:", self.norm)
        print("last_position:", self.last_position)