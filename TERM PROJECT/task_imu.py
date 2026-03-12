# task_imu.py

import micropython

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_imu:
    def __init__(self,
                 imu,                       # BNO055 object
                 enable_share=None,         # Share("B")
                 mode_share=None,           # Share("B") desired BNO055 mode byte (0x08, 0x0C, ...)
                 zero_cmd_share=None,       # Share("B") write 1 -> zero heading
                 save_cal_cmd_share=None,   # Share("B") write 1 -> save calib profile to file
                 heading_deg=None,          # Share("f") published (zeroed) heading
                 yaw_rate_dps=None,         # Share("f") published yaw rate (deg/s)
                 calib_raw=None,            # Share("B") raw calib byte
                 require_mag=False,
                 calib_file="bno055_calib.bin",
                 try_load_calib=True,
                 fusion_mode=None):
        self._imu = imu
        self._en = enable_share

        self._mode_share = mode_share
        self._zero_cmd = zero_cmd_share
        self._save_cmd = save_cal_cmd_share

        self._heading = heading_deg
        self._yaw = yaw_rate_dps
        self._calib = calib_raw

        self._require_mag = bool(require_mag)
        self._calib_file = calib_file
        self._try_load = bool(try_load_calib)

        # If mode_share isn't used, we'll set this once in INIT
        self._fusion_mode = fusion_mode  # if None, leave as-is

        self._state = S0_INIT

        # Runtime state
        self._last_mode = None
        self._heading_offset = 0.0

    def _get_u8_from_share(self, sh):
        try:
            return int(sh.get()) & 0xFF
        except Exception:
            return None

    def _apply_mode(self, desired):
        # Ignore invalid / missing mode values
        if desired is None:
            return

        if desired == 0x00:  # block CONFIG mode from UI
            return

        if (self._last_mode is None) or (desired != self._last_mode):
            try:
                self._imu.set_mode(desired)
                self._last_mode = desired
            except Exception:
                pass

    def _apply_mode_if_needed(self):
        if self._mode_share is None:
            return
        desired = self._get_u8_from_share(self._mode_share)
        self._apply_mode(desired)

    def run(self):
        while True:
            if self._state == S0_INIT:
                if self._mode_share is not None:
                    self._apply_mode_if_needed()
                elif self._fusion_mode is not None:
                    self._apply_mode(int(self._fusion_mode) & 0xFF)

                if self._try_load:
                    try:
                        self._imu.load_calib_from_file(self._calib_file)
                    except Exception:
                        pass

                if self._heading is not None:
                    self._heading.put(0.0)
                if self._yaw is not None:
                    self._yaw.put(0.0)
                if self._calib is not None:
                    self._calib.put(0)

                self._heading_offset = 0.0
                self._state = S1_RUN

            elif self._state == S1_RUN:
                enabled = True
                if self._en is not None:
                    try:
                        enabled = bool(self._en.get())
                    except Exception:
                        enabled = True

                if enabled:
                    self._apply_mode_if_needed()

                    if self._zero_cmd is not None:
                        try:
                            if int(self._zero_cmd.get()) == 1:
                                h_now = float(self._imu.heading())
                                self._heading_offset = h_now
                                if self._heading is not None:
                                    self._heading.put(0.0)
                                self._zero_cmd.put(0)
                        except Exception:
                            try:
                                self._zero_cmd.put(0)
                            except Exception:
                                pass

                    if self._save_cmd is not None:
                        try:
                            if int(self._save_cmd.get()) == 1:
                                self._imu.save_calib_to_file(self._calib_file)
                                self._save_cmd.put(0)
                        except Exception:
                            try:
                                self._save_cmd.put(0)
                            except Exception:
                                pass

                    try:
                        h_now = float(self._imu.heading())
                        h_pub = (h_now - self._heading_offset + 360.0) % 360.0
                        if self._heading is not None:
                            self._heading.put(h_pub)
                    except Exception:
                        pass

                    try:
                        wz = float(self._imu.yaw_rate())
                        if self._yaw is not None:
                            self._yaw.put(wz)
                    except Exception:
                        pass

                    cs = None
                    try:
                        cs = int(self._imu.get_calib_raw()) & 0xFF
                    except AttributeError:
                        try:
                            cs = int(self._imu.get_calib_status()["raw"]) & 0xFF
                        except Exception:
                            cs = None
                    except Exception:
                        cs = None

                    if (cs is not None) and (self._calib is not None):
                        try:
                            self._calib.put(cs)
                        except Exception:
                            pass

            yield self._state
