# imu_driver.py
# MicroPython BNO055 IMU driver (pyb.I2C), minimal features required by lab:

import time
import ustruct

try:
    from machine import SoftI2C, Pin
except Exception:
    SoftI2C = None
    Pin = None


class _I2CAdapter:
    """Wrap machine.SoftI2C to look like pyb.I2C for this driver."""
    def __init__(self, i2c):
        self._i2c = i2c

    def mem_write(self, data, addr, reg, timeout=0):
        if isinstance(data, int):
            data = bytes([data & 0xFF])
        self._i2c.writeto_mem(addr, reg, data)

    def mem_read(self, n, addr, reg, timeout=0):
        return self._i2c.readfrom_mem(addr, reg, n)

class BNO055:
    ADDR_A = 0x28
    ADDR_B = 0x29

    # Registers
    REG_CHIP_ID     = 0x00
    REG_OPR_MODE    = 0x3D
    REG_PWR_MODE    = 0x3E
    REG_SYS_TRIGGER = 0x3F
    REG_UNIT_SEL    = 0x3B
    REG_CALIB_STAT  = 0x35

    # Data registers
    REG_EULER_H_LSB = 0x1A  # heading (yaw)
    REG_GYR_X_LSB   = 0x14  # gyro x,y,z

    # Calibration profile storage region
    REG_CALIB_START = 0x55
    CALIB_LEN       = 22

    # Power modes
    PWR_NORMAL = 0x00

    # Operation modes
    MODE_CONFIG  = 0x00
    MODE_IMUPLUS = 0x08  # accel + gyro fusion (no mag) - often best indoors
    MODE_NDOF    = 0x0C  # full fusion (accel+gyro+mag)

    def __init__(self, i2c, addr=ADDR_A, timeout_ms=50):
        """
        i2c: pyb.I2C object already configured in CONTROLLER mode.
        """
        self.i2c = i2c
        self.addr = addr
        self.timeout_ms = timeout_ms
        self._mode = self.MODE_CONFIG

        # Confirm chip ID (BNO055 can boot slowly)
        chip = self._read_u8(self.REG_CHIP_ID)
        if chip != 0xA0:
            time.sleep_ms(700)
            chip = self._read_u8(self.REG_CHIP_ID)
            if chip != 0xA0:
                raise OSError("BNO055 not found (CHIP_ID=0x%02X)" % chip)

        # Start in CONFIG
        self.set_mode(self.MODE_CONFIG)

        # Normal power
        self._write_u8(self.REG_PWR_MODE, self.PWR_NORMAL)
        time.sleep_ms(10)

        # Leave UNIT_SEL default (Euler deg, gyro dps)
        # self._write_u8(self.REG_UNIT_SEL, 0x00)

        # Clear SYS_TRIGGER
        self._write_u8(self.REG_SYS_TRIGGER, 0x00)
        time.sleep_ms(10)

    # ---------- low-level helpers ----------
    def _write_u8(self, reg, val):
        self.i2c.mem_write(val & 0xFF, self.addr, reg, timeout=self.timeout_ms)

    def _read_u8(self, reg):
        return self.i2c.mem_read(1, self.addr, reg, timeout=self.timeout_ms)[0]

    def _read_len(self, reg, n):
        return self.i2c.mem_read(n, self.addr, reg, timeout=self.timeout_ms)

    # ---------- required lab methods ----------
    def set_mode(self, mode):
        """Safely switch operating mode (switch through CONFIG)."""
        if mode == self._mode:
            return

        # go CONFIG first
        self._write_u8(self.REG_OPR_MODE, self.MODE_CONFIG)
        time.sleep_ms(25)

        # set target
        self._write_u8(self.REG_OPR_MODE, mode & 0xFF)
        time.sleep_ms(25)

        self._mode = mode

    def get_calib_status(self):
        """
        Returns dict with sys/gyr/acc/mag each 0..3 and raw byte.
        """
        cs = self._read_u8(self.REG_CALIB_STAT)
        return {
            "sys": (cs >> 6) & 0x03,
            "gyr": (cs >> 4) & 0x03,
            "acc": (cs >> 2) & 0x03,
            "mag": (cs >> 0) & 0x03,
            "raw": cs,
        }

    def get_calib_coeffs(self):
        """Read 22 bytes calibration profile (do in CONFIG mode)."""
        prev = self._mode
        self.set_mode(self.MODE_CONFIG)
        data = self._read_len(self.REG_CALIB_START, self.CALIB_LEN)
        self.set_mode(prev)
        return bytes(data)

    def set_calib_coeffs(self, data):
        """Write 22 bytes calibration profile (do in CONFIG mode)."""
        if data is None or len(data) != self.CALIB_LEN:
            raise ValueError("Calibration data must be %d bytes" % self.CALIB_LEN)

        prev = self._mode
        self.set_mode(self.MODE_CONFIG)
        self.i2c.mem_write(data, self.addr, self.REG_CALIB_START, timeout=self.timeout_ms)
        self.set_mode(prev)
        time.sleep_ms(10)

    def read_euler(self):
        """
        Returns (heading_deg, roll_deg, pitch_deg).
        Scale: 1 LSB = 1/16 degree (default).
        """
        buf = self._read_len(self.REG_EULER_H_LSB, 6)
        h, r, p = ustruct.unpack("<hhh", buf)
        return (h / 16.0, r / 16.0, p / 16.0)

    def heading(self):
        return self.read_euler()[0]

    def read_gyro(self):
        """
        Returns (gx_dps, gy_dps, gz_dps).
        Scale: 1 LSB = 1/16 dps (default).
        """
        buf = self._read_len(self.REG_GYR_X_LSB, 6)
        gx, gy, gz = ustruct.unpack("<hhh", buf)
        return (gx / 16.0, gy / 16.0, gz / 16.0)

    def yaw_rate(self):
        return self.read_gyro()[2]
    
    def get_calib_raw(self):
        """Return the raw CALIB_STAT byte (no dict allocation)."""
        return self._read_u8(self.REG_CALIB_STAT)
    
    @classmethod
    def from_softi2c(cls, scl_pin, sda_pin, freq=100000, addr=ADDR_A, timeout_ms=50):
        if SoftI2C is None or Pin is None:
            raise RuntimeError("SoftI2C not available in this MicroPython build")
        soft = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
        return cls(_I2CAdapter(soft), addr=addr, timeout_ms=timeout_ms)

    # ---------- convenient file helpers ----------
    def save_calib_to_file(self, filename="bno055_calib.bin"):
        data = self.get_calib_coeffs()
        with open(filename, "wb") as f:
            f.write(data)

    def load_calib_from_file(self, filename="bno055_calib.bin"):
        try:
            with open(filename, "rb") as f:
                data = f.read()
            if len(data) != self.CALIB_LEN:
                return False
            self.set_calib_coeffs(data)
            return True
        except OSError:
            return False