import pyb
from pyb import USB_VCP


class task_tuning_ui:
    def __init__(self,
                 start_user,
                 setpoint, Kp, Ki,
                 ser=None,
                 follow_en=None,
                 Kp_line=None,
                 Ki_line=None,
                 cal_cmd=None,
                 cal_done=None,
                 line_err=None,
                 dv_out=None,
                 leftMotorGo=None,
                 rightMotorGo=None,
                 posL=None,
                 posR=None,
                 omegaL=None,
                 omegaR=None,
                 setpointL=None,
                 setpointR=None,
                 script_state=None,
                 script_total_counts=None,
                 script_segment_counts=None,
                 line_ok=None,
                 imu_heading=None,
                 imu_yawrate=None,
                 imu_calraw=None,
                 imu_en=None,
                 imu_mode=None,
                 imu_zero_cmd=None,
                 imu_save_cmd=None):

        self._start_user = start_user
        self._setpoint = setpoint
        self._Kp = Kp
        self._Ki = Ki
        self._ser = ser if ser is not None else USB_VCP()
        self._follow_en = follow_en
        self._Kp_line = Kp_line
        self._Ki_line = Ki_line
        self._cal_cmd = cal_cmd
        self._cal_done = cal_done
        self._line_err = line_err
        self._dv_out = dv_out
        self._leftMotorGo = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._posL = posL
        self._posR = posR
        self._omegaL = omegaL
        self._omegaR = omegaR
        self._setpointL = setpointL
        self._setpointR = setpointR
        self._script_state = script_state
        self._script_total_counts = script_total_counts
        self._script_segment_counts = script_segment_counts
        self._line_ok = line_ok
        self._imu_heading = imu_heading
        self._imu_yawrate = imu_yawrate
        self._imu_calraw = imu_calraw
        self._imu_en = imu_en
        self._imu_mode = imu_mode
        self._imu_zero_cmd = imu_zero_cmd
        self._imu_save_cmd = imu_save_cmd

        self._digits = "0123456789"
        self._term1 = "\r"
        self._term2 = "\n"
        self._ui_mode = "CMD"
        self._menu_shown = False
        self._cal_armed = False
        self._cal_waiting = False
        self._stream_end_ms = 0
        self._stream_next_ms = 0
        self._stream_period_ms = 100

        try:
            self._user_btn = pyb.Pin('PC13', pyb.Pin.IN)
        except Exception:
            self._user_btn = None
        self._user_btn_prev = 1

    def print_help(self):
        import ui_help
        ui_help.print_help(self._ser)

    def _safe_get(self, share, default=None):
        if share is None:
            return default
        try:
            return share.get()
        except Exception:
            return default

    def _stage_name(self, code):
        names = {
            0: "IDLE",
            1: "RUN",
            2: "TUNE_ZONE",
            3: "TURN_SMALL",
            4: "FWD_1",
            5: "TURN_90",
            6: "FWD_2",
            7: "LOST_STOP",
            8: "SCRIPT_EXIT",
        }
        return names.get(int(code), "UNKNOWN")

    def _read_char_blocking(self):
        while not self._ser.any():
            pyb.delay(1)
        return self._ser.read(1).decode()

    def read_number(self, prompt):
        s = self._ser
        s.write(prompt)
        buf = ""
        while True:
            ch = self._read_char_blocking()
            if ch in self._digits:
                s.write(ch.encode())
                buf += ch
            elif ch == "." and "." not in buf:
                s.write(ch.encode())
                buf += ch
            elif ch == "-" and len(buf) == 0:
                s.write(ch.encode())
                buf += ch
            elif ch == "\x7f" and len(buf) > 0:
                s.write(ch.encode())
                buf = buf[:-1]
            elif ch == self._term1 or ch == self._term2:
                if len(buf) == 0:
                    s.write(b"\r\nValue not changed\r\n")
                    return None
                if buf == "-" or buf == ".":
                    s.write(b"\r\nInvalid entry, try again\r\n")
                    s.write(prompt)
                    buf = ""
                    continue
                s.write(b"\r\n")
                return float(buf)

    def _start_stream(self, ms=3000):
        now = pyb.millis()
        self._stream_end_ms = now + int(ms)
        self._stream_next_ms = now
        self._ui_mode = "SENSE_STREAM"

    def _run_line_follow_from_button(self):
        s = self._ser
        s.write(b"n\r\n")
        if self._leftMotorGo is None or self._rightMotorGo is None:
            s.write(b"Motor Go shares not wired in main.py\r\n>: ")
            return
        if self._follow_en is not None:
            self._follow_en.put(1)
        self._leftMotorGo.put(1)
        self._rightMotorGo.put(1)
        s.write(b"Motors RUNNING. Streaming encoder/tune/script status...\r\n")
        self._start_stream(3600000)

    def _stop_line_follow_from_button(self):
        s = self._ser
        s.write(b"x\r\n")
        if self._leftMotorGo is not None:
            self._leftMotorGo.put(0)
        if self._rightMotorGo is not None:
            self._rightMotorGo.put(0)
        if self._follow_en is not None:
            self._follow_en.put(0)
        self._ui_mode = "CMD"
        s.write(b"Motors stopped.\r\n>: ")

    def _cancel_cal(self):
        s = self._ser
        self._cal_armed = False
        self._cal_waiting = False
        if self._cal_cmd is not None:
            self._cal_cmd.put(0)
        if self._cal_done is not None:
            self._cal_done.put(0)
        s.write(b"\r\nCalibration canceled/disarmed.\r\n>: ")

    def _do_cal(self, which):
        s = self._ser
        if self._cal_cmd is None:
            s.write(b"\r\ncal_cmd share not wired in main.py\r\n>: ")
            return
        self._cal_waiting = True
        if self._cal_done is not None:
            self._cal_done.put(0)
        if which == 1:
            s.write(b"\r\nCalibrating WHITE... hold steady.\r\n")
            self._cal_cmd.put(1)
            s.write(b"Command sent.\r\n>: ")
        else:
            s.write(b"\r\nCalibrating BLACK... hold steady.\r\n")
            self._cal_cmd.put(2)
            s.write(b"Command sent.\r\n>: ")
            self._cal_armed = False

    def _print_status(self):
        s = self._ser
        stage_code = self._safe_get(self._script_state, 0)
        stage = self._stage_name(stage_code)
        total = self._safe_get(self._script_total_counts, 0.0)
        seg = self._safe_get(self._script_segment_counts, 0.0)
        posL = self._safe_get(self._posL, 0)
        posR = self._safe_get(self._posR, 0)
        spL = self._safe_get(self._setpointL, 0.0)
        spR = self._safe_get(self._setpointR, 0.0)
        line_ok = self._safe_get(self._line_ok, 1)

        s.write(b"\r\n--- STATUS ---\r\n")
        s.write(("stage:      {}\r\n".format(stage)).encode())
        s.write(("total:      {}\r\n".format(total)).encode())
        s.write(("segment:    {}\r\n".format(seg)).encode())
        s.write(("posL:       {}\r\n".format(posL)).encode())
        s.write(("posR:       {}\r\n".format(posR)).encode())
        s.write(("spL:        {}\r\n".format(spL)).encode())
        s.write(("spR:        {}\r\n".format(spR)).encode())
        s.write(("V_nom:      {}\r\n".format(self._safe_get(self._setpoint, 0.0))).encode())
        s.write(("wheel Kp:   {}\r\n".format(self._safe_get(self._Kp, 0.0))).encode())
        s.write(("wheel Ki:   {}\r\n".format(self._safe_get(self._Ki, 0.0))).encode())
        s.write(("line Kp:    {}\r\n".format(self._safe_get(self._Kp_line, 0.0))).encode())
        s.write(("line Ki:    {}\r\n".format(self._safe_get(self._Ki_line, 0.0))).encode())
        s.write(("line_ok:    {}\r\n".format(int(line_ok))).encode())
        s.write(("line_err:   {}\r\n".format(self._safe_get(self._line_err, 0.0))).encode())
        s.write(b"--------------\r\n>: ")

    def _print_stream_line(self):
        import gc
        gc.collect()
        try:
            s = self._ser
            stage = self._stage_name(self._safe_get(self._script_state, 0))
            total = self._safe_get(self._script_total_counts, 0.0)
            seg = self._safe_get(self._script_segment_counts, 0.0)
            posL = self._safe_get(self._posL, 0)
            posR = self._safe_get(self._posR, 0)
            spL = self._safe_get(self._setpointL, 0.0)
            spR = self._safe_get(self._setpointR, 0.0)
            line_ok = int(self._safe_get(self._line_ok, 1))

            s.write((
                "stage={} total={} seg={} posL={} posR={} spL={} spR={} V={} wheelKp={} wheelKi={} lineKp={} lineKi={} line_ok={}\r\n".format(
                    stage,
                    total,
                    seg,
                    posL,
                    posR,
                    spL,
                    spR,
                    self._safe_get(self._setpoint, 0.0),
                    self._safe_get(self._Kp, 0.0),
                    self._safe_get(self._Ki, 0.0),
                    self._safe_get(self._Kp_line, 0.0),
                    self._safe_get(self._Ki_line, 0.0),
                    line_ok,
                )
            ).encode())
        except Exception:
            pass

    def ui_poll(self):
        s = self._ser
        if self._start_user.get() != 0:
            return

        if self._user_btn is not None:
            try:
                btn_now = self._user_btn.value()
                if self._user_btn_prev == 1 and btn_now == 0:
                    left_running = bool(self._safe_get(self._leftMotorGo, 0))
                    right_running = bool(self._safe_get(self._rightMotorGo, 0))
                    if left_running or right_running or self._ui_mode == "SENSE_STREAM":
                        self._stop_line_follow_from_button()
                    else:
                        self._run_line_follow_from_button()
                    self._user_btn_prev = btn_now
                    return
                self._user_btn_prev = btn_now
            except Exception:
                pass

        if self._ui_mode == "CMD":
            if self._ser.any():
                cmd = self._ser.read(1).decode()
                c = cmd.lower()

                if c == 'h':
                    s.write(b"\r\n")
                    self.print_help()
                elif c == 'k':
                    s.write(b"\r\n")
                    self._ui_mode = "GET_KP"
                elif c == 's':
                    s.write(b"\r\n")
                    self._ui_mode = "GET_SP"
                elif c == 'o':
                    s.write(b"\r\n")
                    self._ui_mode = "GET_LKP"
                elif c == 'n':
                    self._run_line_follow_from_button()
                elif c == 'x':
                    self._stop_line_follow_from_button()
                elif c == 'p':
                    self._print_status()
                elif c == 'f':
                    s.write(b"\r\n")
                    if self._follow_en is None:
                        s.write(b"follow_en share not wired in main.py\r\n>: ")
                    else:
                        newv = 0 if int(self._follow_en.get()) else 1
                        self._follow_en.put(newv)
                        s.write(("follow_en set to {}\r\n>: ".format(newv)).encode())
                elif c == 'i':
                    s.write(b"\r\n")
                    if self._imu_en is None:
                        s.write(b"imu_en share not wired in main.py\r\n>: ")
                    else:
                        newv = 0 if int(self._imu_en.get()) else 1
                        self._imu_en.put(newv)
                        s.write(("imu_en set to {}\r\n>: ".format(newv)).encode())
                elif c == 'm':
                    s.write(b"\r\n")
                    if self._imu_mode is None:
                        s.write(b"imu_mode share not wired in main.py\r\n>: ")
                    else:
                        cur = int(self._imu_mode.get()) & 0xFF
                        newm = 0x0C if cur == 0x08 else 0x08
                        self._imu_mode.put(newm)
                        name = "NDOF" if newm == 0x0C else "IMUPLUS"
                        s.write(("imu_mode set to {} (0x{:02X})\r\n>: ".format(name, newm)).encode())
                elif c == 'z':
                    s.write(b"\r\n")
                    if self._imu_zero_cmd is None:
                        s.write(b"imu_zero_cmd share not wired in main.py\r\n>: ")
                    else:
                        self._imu_zero_cmd.put(1)
                        s.write(b"IMU heading zero requested.\r\n>: ")
                elif c == 'u':
                    s.write(b"\r\n")
                    if self._imu_save_cmd is None:
                        s.write(b"imu_save_cmd share not wired in main.py\r\n>: ")
                    else:
                        self._imu_save_cmd.put(1)
                        s.write(b"Requested IMU calibration save.\r\n>: ")
                elif c == 'c':
                    s.write(b"\r\n")
                    self._cal_armed = True
                    self._cal_waiting = False
                    if self._cal_done is not None:
                        self._cal_done.put(0)
                    s.write(b"Calibration ARMED:\r\n")
                    s.write(b"  w = calibrate WHITE\r\n")
                    s.write(b"  b = calibrate BLACK\r\n")
                    s.write(b"  a = cancel calibration/disarm\r\n")
                    s.write(b">: ")
                elif c == 'w':
                    if not self._cal_armed:
                        s.write(b"\r\nPress 'c' to arm calibration first.\r\n>: ")
                    else:
                        self._do_cal(1)
                elif c == 'b':
                    if not self._cal_armed:
                        s.write(b"\r\nPress 'c' to arm calibration first.\r\n>: ")
                    else:
                        self._do_cal(2)
                elif c == 'a':
                    self._cancel_cal()
                elif cmd == '\r' or cmd == '\n':
                    s.write(b"\r\n>: ")
                else:
                    s.write(b"\r\nUnknown command. Press 'h' for help.\r\n>: ")

        elif self._ui_mode == "GET_KP":
            val = self.read_number(b"\r\nEnter wheel proportional gain, Kp:\r\n>: ")
            if val is not None:
                self._Kp.put(val)
                s.write(("Wheel Kp set to {}\r\n".format(val)).encode())
            self._ui_mode = "GET_KI"

        elif self._ui_mode == "GET_KI":
            val = self.read_number(b"\r\nEnter wheel integral gain, Ki:\r\n>: ")
            if val is not None:
                self._Ki.put(val)
                s.write(("Wheel Ki set to {}\r\n".format(val)).encode())
            self._ui_mode = "CMD"
            s.write(b"\r\n>: ")

        elif self._ui_mode == "GET_SP":
            val = self.read_number(b"\r\nEnter V_nom (forward speed, counts/s):\r\n>: ")
            if val is not None:
                self._setpoint.put(val)
                s.write(("V_nom set to {}\r\n".format(val)).encode())
            self._ui_mode = "CMD"
            s.write(b"\r\n>: ")

        elif self._ui_mode == "GET_LKP":
            val = self.read_number(b"\r\nEnter line Kp_line:\r\n>: ")
            if val is not None and self._Kp_line is not None:
                self._Kp_line.put(val)
                s.write(("Kp_line set to {}\r\n".format(val)).encode())
            self._ui_mode = "GET_LKI"

        elif self._ui_mode == "GET_LKI":
            val = self.read_number(b"\r\nEnter line Ki_line:\r\n>: ")
            if val is not None and self._Ki_line is not None:
                self._Ki_line.put(val)
                s.write(("Ki_line set to {}\r\n".format(val)).encode())
            self._ui_mode = "CMD"
            s.write(b"\r\n>: ")

        elif self._ui_mode == "SENSE_STREAM":
            now = pyb.millis()
            if s.any():
                ch = s.read(1).decode().lower()
                if ch == 'x':
                    self._stop_line_follow_from_button()
                else:
                    s.write(b"\r\nStream stopped.\r\n>: ")
                    self._ui_mode = "CMD"
                return
            if now >= self._stream_end_ms:
                s.write(b"\r\nStream done.\r\n>: ")
                self._ui_mode = "CMD"
                return
            if now >= self._stream_next_ms:
                self._print_stream_line()
                self._stream_next_ms = now + self._stream_period_ms

    def run(self):
        while True:
            if self._start_user.get() == 0 and not self._menu_shown:
                self.print_help()
                self._menu_shown = True
            if self._start_user.get() != 0:
                self._menu_shown = False
                self._ui_mode = "CMD"
            if (self._cal_done is not None and int(self._cal_done.get()) == 1 and self._cal_waiting):
                self._ser.write(b"\r\nCalibration step completed (sensor task reports done).\r\n>: ")
                self._cal_done.put(0)
                self._cal_waiting = False
            self.ui_poll()
            yield
