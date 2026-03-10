# line_sensor.py
# MicroPython driver for Pololu QTR-MD-08A (Analog, 8-channel)
# STM32 pyb.ADC version (NUCLEO-L476RG): adc.read() returns 0..4095 (12-bit)

import pyb
import utime


class QTRSensorsAnalog:
    """
    QTR reflectance sensor array (analog version).

    For QTR "A" versions (typical behavior):
      - Strong reflectance (white) -> output near 0 V  -> LOW ADC
      - Weak reflectance (black)  -> output near VCC  -> HIGH ADC

    Normalization default:
      white -> 0, black -> 1000

    Robustness included:
      - Per-channel auto-fix if calibration datums get reversed (black < white)
      - Safe denom handling (never negative / never divide-by-zero)
    """

    ADC_MAX = 4095  # STM32 pyb.ADC.read() is 12-bit

    def __init__(self, adc_pins, emitter_pin=None, invert=False):
        """
        adc_pins    : list[str|pyb.Pin] of analog pins (length = sensor count)
        emitter_pin : optional CTRL pin (str|pyb.Pin). If provided, drives emitters on/off.
        invert      : if True, flips normalized so white->1000 and black->0
        """
        self.count = len(adc_pins)
        if self.count < 1:
            raise ValueError("adc_pins must contain at least 1 pin")

        # ADC channels
        self._adcs = []
        for p in adc_pins:
            pin = p if isinstance(p, pyb.Pin) else pyb.Pin(p)
            self._adcs.append(pyb.ADC(pin))

        # Optional emitter control
        self._emitter = None
        if emitter_pin is not None:
            ep = emitter_pin if isinstance(emitter_pin, pyb.Pin) else pyb.Pin(emitter_pin)
            self._emitter = pyb.Pin(ep, pyb.Pin.OUT_PP)
            # Default ON
            self._emitter.high()

        self.invert = bool(invert)

        # Latest reads
        self.raw = [0] * self.count
        self.norm = [0] * self.count  # 0..1000

        # Calibration datums
        self.white = [0] * self.count
        self.black = [self.ADC_MAX] * self.count
        self._have_white = False
        self._have_black = False

        # Last known line position in "QTR units" (0..(N-1)*1000)
        self.last_position = (self.count - 1) * 1000 // 2

        # Ignore tiny normalized values in centroid calc
        self.min_reading = 20  # 0..1000

    # -----------------------------
    # Emitter control
    # -----------------------------
    def emitters_on(self):
        if self._emitter is not None:
            self._emitter.high()
            utime.sleep_us(250)

    def emitters_off(self):
        if self._emitter is not None:
            self._emitter.low()
            utime.sleep_us(250)

    # -----------------------------
    # Raw reading
    # -----------------------------
    def read_raw(self, oversample=1, emitters=True):
        """
        Reads raw ADC values into self.raw and returns the list.

        oversample: averages N ADC reads per channel to reduce noise
        emitters: if True, turn emitters on during measurement (recommended)
        """
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

    # -----------------------------
    # Calibration datums
    # -----------------------------
    def calibrate_white(self, samples=50, oversample=1, emitters=True, settle_ms=0):
        """
        Capture WHITE datum (high reflectance).
        Put robot over the background (white) and call this.
        Stores per-channel average into self.white.
        """
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

        # If black isn't set yet, initialize black just above white so denom>0
        if not self._have_black:
            self.black = [min(self.ADC_MAX, w + 1) for w in self.white]

        return self.white

    def calibrate_black(self, samples=50, oversample=1, emitters=True, settle_ms=0):
        """
        Capture BLACK datum (low reflectance).
        Put robot over the line (black) and call this.
        Stores per-channel average into self.black.
        """
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

        # If white isn't set yet, initialize white just below black so denom>0
        if not self._have_white:
            self.white = [max(0, b - 1) for b in self.black]

        return self.black

    def calibrated_ready(self):
        return self._have_white and self._have_black

    def fix_calibration_order(self):
        """
        Optional one-time cleanup after calibration:
        If any channel has black < white, swap them so black >= white.

        You can call this once after running calibrate_white() and calibrate_black().
        """
        for i in range(self.count):
            if self.black[i] < self.white[i]:
                self.black[i], self.white[i] = self.white[i], self.black[i]

    # -----------------------------
    # Normalization (0..1000)
    # -----------------------------
    def read_normalized(self, oversample=1, emitters=True):
        """
        Updates self.raw, then converts to normalized 0..1000 using:
            norm = (raw - white) / (black - white) * 1000

        Default meaning:
            white -> 0
            black -> 1000

        If invert=True:
            white -> 1000
            black -> 0

        Robust behavior:
            - If calibration datums are reversed for a channel (black < white),
              they are treated as swapped for that read so denom stays positive.
        """
        self.read_raw(oversample=oversample, emitters=emitters)

        # If not calibrated, crude scale 0..1000 from ADC range
        if not self.calibrated_ready():
            for i in range(self.count):
                v = (self.raw[i] * 1000) // self.ADC_MAX
                v = 1000 - v if self.invert else v
                self.norm[i] = 0 if v < 0 else (1000 if v > 1000 else v)
            return self.norm

        for i in range(self.count):
            w = self.white[i]
            b = self.black[i]

            # --- KEY FIX: ensure denom is positive even if calibration was reversed ---
            if b < w:
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

    # -----------------------------
    # Centroid / Line position
    # -----------------------------
    def read_line_position(self, oversample=1, emitters=True):
        """
        Returns line position in Pololu-style units: 0..(N-1)*1000

        Uses normalized values as weights and computes:
            position = sum(weight_i * i*1000) / sum(weight_i)

        If all readings are "zero", returns last_position.
        """
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
        """
        Signed error from center in "QTR units".
          error < 0 means line is left of center
          error > 0 means line is right of center
        """
        pos = self.read_line_position(oversample=oversample, emitters=emitters)
        center = (self.count - 1) * 1000 // 2
        return pos - center

    # -----------------------------
    # Debug / Quick tests
    # -----------------------------
    def emitter_test(self, oversample=1):
        """
        Quick test: compare readings with emitters on vs off, same spot.
        If ON and OFF are almost identical, emitter control pin likely isn't working.
        """
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