import micropython
import pyb
from pyb import USB_VCP
from task_share import Share, Queue

# ---------------- FSM States for data UI ----------------
S0_INIT = micropython.const(0)
S1_CMD  = micropython.const(1)
S2_COL  = micropython.const(2)
S3_DIS  = micropython.const(3)

# ---------------- Modes for data UI ----------------
MODE_NONE = micropython.const(0)
MODE_L    = micropython.const(1)
MODE_R    = micropython.const(2)
MODE_BOTH = micropython.const(3)

UI_prompt = ">: "


class task_user:
    # Data-collection UI task (runs only when start_user == 1)

    def __init__(self,
                 start_user,
                 leftMotorGo, rightMotorGo,
                 dataValuesL, timeValuesL,
                 dataValuesR, timeValuesR,
                 setpoint, Kp, Ki,
                 ser=None,
                 follow_en=None,
                 Kp_line=None,
                 Ki_line=None,
                 line_err=None,
                 dv_out=None,
                 print_live_en=None,
                 # --- IMU shares (optional) ---
                 imu_heading=None,   # Share("f") heading deg
                 imu_yawrate=None,   # Share("f") yaw rate dps
                 imu_calraw=None):   # Share("B") raw calib byte

        self._state = S0_INIT

        self._start_user   = start_user
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo

        self._ser = ser if ser is not None else USB_VCP()

        # Buffers
        self._dataL = dataValuesL
        self._timeL = timeValuesL
        self._dataR = dataValuesR
        self._timeR = timeValuesR

        # Inner-loop shares (printed for header)
        self._setpoint = setpoint   # v_nom
        self._Kp = Kp
        self._Ki = Ki

        # Outer-loop shares (optional, printed for header)
        self._follow_en = follow_en
        self._Kp_line = Kp_line
        self._Ki_line = Ki_line
        self._line_err = line_err
        self._dv_out = dv_out

        # IMU shares (optional)
        self._imu_heading = imu_heading
        self._imu_yawrate = imu_yawrate
        self._imu_calraw  = imu_calraw

        # Live printing gate (motor/control tasks must read this)
        self._print_live_en = print_live_en

        self._mode = MODE_NONE
        self._both_phase = 0  # 0=print left, 1=print right

        self._ser.write(b"task_user instantiated\r\n")

    # ---------------- Printing helpers ----------------

    def _print_gains_header(self):
        s = self._ser
        s.write(b"--------------------------------------------------------------------------------\r\n")
        s.write(("V_nom (setpoint): {}\r\n".format(self._setpoint.get())).encode())
        s.write(("Wheel Kp:         {}\r\n".format(self._Kp.get())).encode())
        s.write(("Wheel Ki:         {}\r\n".format(self._Ki.get())).encode())

        if (self._follow_en is not None) and (self._Kp_line is not None) and (self._Ki_line is not None):
            s.write(("Follow_en:        {}\r\n".format(int(self._follow_en.get()))).encode())
            s.write(("Line Kp:          {}\r\n".format(self._Kp_line.get())).encode())
            s.write(("Line Ki:          {}\r\n".format(self._Ki_line.get())).encode())

        if (self._line_err is not None) and (self._dv_out is not None):
            s.write(("Line err:         {}\r\n".format(self._line_err.get())).encode())
            s.write(("dv_out:           {}\r\n".format(self._dv_out.get())).encode())

        # IMU values (optional)
        if (self._imu_heading is not None) and (self._imu_yawrate is not None):
            s.write(("IMU heading:     {}\r\n".format(self._imu_heading.get())).encode())
            s.write(("IMU yaw_rate:    {}\r\n".format(self._imu_yawrate.get())).encode())
        if self._imu_calraw is not None:
            s.write(("IMU cal raw:     0x{:02X}\r\n".format(int(self._imu_calraw.get()) & 0xFF)).encode())

        if self._print_live_en is not None:
            s.write(("Live print:       {}\r\n".format(int(self._print_live_en.get()))).encode())

        s.write(b"--------------------------------------------------------------------------------\r\n")

    def _print_footer_and_prompt(self):
        s = self._ser
        s.write(b"--------------------------------------------------------------------------------\r\n")
        s.write(b"Commands: l=left, r=right, b=both, x=stop, v=toggle live print, h=menu\r\n")
        s.write(UI_prompt.encode())

    def _print_section_header(self, label):
        s = self._ser
        s.write(b"Data collection complete...\r\nPrinting data...\r\n")
        s.write(b"--------------------------------------------------------------------------------\r\n")
        s.write((label + "\r\n").encode())
        s.write(b"Time [us], Data\r\n")

    # ---------------- Start modes ----------------

    def _start_left(self):
        self._dataL.clear(); self._timeL.clear()
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(False)
        self._mode = MODE_L
        self._print_gains_header()
        self._ser.write(b"Starting LEFT data collection...\r\n(press x to stop)\r\nPlease wait...\r\n")

    def _start_right(self):
        self._dataR.clear(); self._timeR.clear()
        self._rightMotorGo.put(True)
        self._leftMotorGo.put(False)
        self._mode = MODE_R
        self._print_gains_header()
        self._ser.write(b"Starting RIGHT data collection...\r\n(press x to stop)\r\nPlease wait...\r\n")

    def _start_both(self):
        self._dataL.clear(); self._timeL.clear()
        self._dataR.clear(); self._timeR.clear()
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._mode = MODE_BOTH
        self._both_phase = 0
        self._print_gains_header()
        self._ser.write(b"Starting BOTH data collection (simultaneous)...\r\n(press x to stop)\r\nPlease wait...\r\n")

    def _stop_all(self):
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        self._mode = MODE_NONE

    # ---------------- Main FSM ----------------

    def run(self):
        while True:

            # Only active when start_user == 1
            if self._start_user.get() == 0:
                self._state = S0_INIT
                yield
                continue

            if self._state == S0_INIT:
                s = self._ser
                s.write(b"\r\nUser task active.\r\n")
                s.write(b"Commands: l=left, r=right, b=both, x=stop, v=toggle live print, h=menu\r\n")
                if self._print_live_en is not None:
                    s.write(("Live print: {}\r\n".format(int(self._print_live_en.get()))).encode())
                s.write(UI_prompt.encode())

                self._mode = MODE_NONE
                self._both_phase = 0
                self._state = S1_CMD

            elif self._state == S1_CMD:
                if self._ser.any():
                    ch = self._ser.read(1).decode()

                    if ch in ("l", "L"):
                        self._ser.write((ch + "\r\n").encode())
                        self._start_left()
                        self._state = S2_COL

                    elif ch in ("r", "R"):
                        self._ser.write((ch + "\r\n").encode())
                        self._start_right()
                        self._state = S2_COL

                    elif ch in ("b", "B"):
                        self._ser.write((ch + "\r\n").encode())
                        self._start_both()
                        self._state = S2_COL

                    elif ch in ("x", "X"):
                        self._stop_all()
                        self._ser.write(b"Stopping...\r\n")
                        self._ser.write(UI_prompt.encode())

                    elif ch in ("v", "V"):
                        self._ser.write((ch + "\r\n").encode())
                        if self._print_live_en is None:
                            self._ser.write(b"print_live_en share not wired in main.py\r\n")
                            self._ser.write(UI_prompt.encode())
                        else:
                            try:
                                left_running = bool(self._leftMotorGo.get())
                            except Exception:
                                left_running = False
                            try:
                                right_running = bool(self._rightMotorGo.get())
                            except Exception:
                                right_running = False

                            if left_running or right_running:
                                self._ser.write(b"Cannot toggle live printing while motors are running/collecting.\r\n")
                                self._ser.write(b"Stop motors (press 'x') first, then toggle.\r\n")
                                self._ser.write(UI_prompt.encode())
                            else:
                                newv = 0 if int(self._print_live_en.get()) else 1
                                self._print_live_en.put(newv)
                                self._ser.write(("Live printing set to {}\r\n".format(newv)).encode())
                                self._ser.write(UI_prompt.encode())

                    elif ch in ("h", "H"):
                        self._stop_all()
                        self._ser.write(b"Returning to tuning menu...\r\n")
                        self._start_user.put(0)
                        self._state = S0_INIT

            elif self._state == S2_COL:
                if self._ser.any():
                    ch = self._ser.read(1).decode()
                    if ch in ("x", "X"):
                        self._stop_all()
                        self._ser.write(b"\r\nStopped.\r\n")
                        self._ser.write(UI_prompt.encode())
                        self._state = S1_CMD
                        yield
                        continue

                if self._mode == MODE_L:
                    if self._dataL.full() or self._timeL.full():
                        self._leftMotorGo.put(False)
                        self._print_section_header("LEFT DATA")
                        self._state = S3_DIS

                elif self._mode == MODE_R:
                    if self._dataR.full() or self._timeR.full():
                        self._rightMotorGo.put(False)
                        self._print_section_header("RIGHT DATA")
                        self._state = S3_DIS

                elif self._mode == MODE_BOTH:
                    if (self._dataL.full() or self._timeL.full() or
                        self._dataR.full() or self._timeR.full()):
                        self._leftMotorGo.put(False)
                        self._rightMotorGo.put(False)
                        self._both_phase = 0
                        self._print_section_header("LEFT DATA")
                        self._state = S3_DIS

            elif self._state == S3_DIS:
                if self._mode == MODE_L:
                    if self._dataL.any():
                        self._ser.write("{},{}\r\n".format(self._timeL.get(), self._dataL.get()).encode())
                    else:
                        self._print_footer_and_prompt()
                        self._mode = MODE_NONE
                        self._state = S1_CMD

                elif self._mode == MODE_R:
                    if self._dataR.any():
                        self._ser.write("{},{}\r\n".format(self._timeR.get(), self._dataR.get()).encode())
                    else:
                        self._print_footer_and_prompt()
                        self._mode = MODE_NONE
                        self._state = S1_CMD

                elif self._mode == MODE_BOTH:
                    if self._both_phase == 0:
                        if self._dataL.any():
                            self._ser.write("{},{}\r\n".format(self._timeL.get(), self._dataL.get()).encode())
                        else:
                            self._both_phase = 1
                            self._ser.write(b"--------------------------------------------------------------------------------\r\n")
                            self._print_section_header("RIGHT DATA")
                    else:
                        if self._dataR.any():
                            self._ser.write("{},{}\r\n".format(self._timeR.get(), self._dataR.get()).encode())
                        else:
                            self._print_footer_and_prompt()
                            self._mode = MODE_NONE
                            self._state = S1_CMD

            yield


class task_tuning_ui:
    # Tuning menu UI task (runs only when start_user == 0)

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
                 # --- IMU shares (optional) ---
                 imu_heading=None,   # Share("f") heading deg
                 imu_yawrate=None,   # Share("f") yaw rate dps
                 imu_calraw=None,    # Share("B") raw calib byte
                 imu_en=None,        # Share("B") enable (optional)
                 imu_mode=None,      # Share("B") BNO055 mode byte (0x08 IMUPLUS, 0x0C NDOF, ...)
                 imu_zero_cmd=None,  # Share("B") write 1 -> zero heading
                 imu_save_cmd=None,  # Share("B") write 1 -> save calib to file
                 # --- State estimator shares (optional) ---
                 est_en=None,        # Share("B") enable
                 xhat_omegaL=None,   # Share("f") rad/s
                 xhat_omegaR=None,   # Share("f") rad/s
                 xhat_s=None,        # Share("f") mm
                 xhat_psi=None,      # Share("f") rad
                 yhat_sL=None,       # Share("f") mm
                 yhat_sR=None,       # Share("f") mm
                 yhat_psi=None,      # Share("f") rad
                 yhat_psidot=None,   # Share("f") rad/s
                 x_pos=None,         # Share("f") mm
                 y_pos=None,         # Share("f") mm
                 dist_traveled=None  # Share("f") mm
                 ):
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

        # IMU shares (optional)
        self._imu_heading = imu_heading
        self._imu_yawrate = imu_yawrate
        self._imu_calraw  = imu_calraw
        self._imu_en      = imu_en

        self._imu_mode     = imu_mode
        self._imu_zero_cmd = imu_zero_cmd
        self._imu_save_cmd = imu_save_cmd
        self._leftMotorGo = leftMotorGo
        self._rightMotorGo = rightMotorGo

        # State estimator shares (optional)
        self._est_en = est_en
        self._xhat_omegaL = xhat_omegaL
        self._xhat_omegaR = xhat_omegaR
        self._xhat_s = xhat_s
        self._xhat_psi = xhat_psi
        self._yhat_sL = yhat_sL
        self._yhat_sR = yhat_sR
        self._yhat_psi = yhat_psi
        self._yhat_psidot = yhat_psidot
        self._x_pos = x_pos
        self._y_pos = y_pos
        self._dist_traveled = dist_traveled

        self._digits = "0123456789"
        self._term1 = "\r"
        self._term2 = "\n"

        self._ui_mode = "CMD"  # CMD, GET_KP, GET_KI, GET_SP, GET_LKp, GET_LKi, SENSE_STREAM
        self._menu_shown = False

        self._cal_armed = False
        self._cal_waiting = False

        self._stream_end_ms = 0
        self._stream_next_ms = 0
        self._stream_period_ms = 50
        self._stream_kind = "LINE"   # "LINE" or "EST"

    def print_help(self):
        # Lazy import to save RAM at startup
        import ui_help
        ui_help.print_help(self._ser)

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
                s.write(ch.encode()); buf += ch
            elif ch == "." and "." not in buf:
                s.write(ch.encode()); buf += ch
            elif ch == "-" and len(buf) == 0:
                s.write(ch.encode()); buf += ch
            elif ch == "\x7f" and len(buf) > 0:
                s.write(ch.encode()); buf = buf[:-1]
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

    def _print_line_status(self):
        s = self._ser
        s.write(b"\r\n--- LINE STATUS ---\r\n")
        if self._follow_en is not None:
            s.write(("follow_en: {}\r\n".format(int(self._follow_en.get()))).encode())
        if self._line_err is not None:
            s.write(("line_err:  {}\r\n".format(self._line_err.get())).encode())
        if self._dv_out is not None:
            s.write(("dv_out:    {}\r\n".format(self._dv_out.get())).encode())
        if self._Kp_line is not None and self._Ki_line is not None:
            s.write(("Kp_line:   {}\r\n".format(self._Kp_line.get())).encode())
            s.write(("Ki_line:   {}\r\n".format(self._Ki_line.get())).encode())

        # IMU status (optional)
        if self._imu_en is not None:
            s.write(("imu_en:    {}\r\n".format(int(self._imu_en.get()))).encode())
        if self._imu_mode is not None:
            s.write(("imu_mode:  0x{:02X}\r\n".format(int(self._imu_mode.get()) & 0xFF)).encode())
        if self._imu_heading is not None:
            s.write(("imu_hdg:   {}\r\n".format(self._imu_heading.get())).encode())
        if self._imu_yawrate is not None:
            s.write(("imu_wz:    {}\r\n".format(self._imu_yawrate.get())).encode())
        if self._imu_calraw is not None:
            s.write(("imu_cal:   0x{:02X}\r\n".format(int(self._imu_calraw.get()) & 0xFF)).encode())

        s.write(("cal_armed: {}\r\n".format(int(self._cal_armed))).encode())
        s.write(b"-------------------\r\n>: ")

    def _print_est_status(self):
        s = self._ser

        # If nothing is wired, give a clear message
        if (self._xhat_omegaL is None and self._xhat_omegaR is None and self._xhat_s is None and
                self._xhat_psi is None and self._yhat_sL is None and self._yhat_sR is None and
                self._yhat_psi is None and self._yhat_psidot is None and self._x_pos is None and
                self._y_pos is None and self._dist_traveled is None):
            s.write(b"\r\nState estimator shares not wired in main.py\r\n>: ")
            return

        s.write(b"\r\n--- STATE ESTIMATOR ---\r\n")
        if self._est_en is not None:
            s.write(("est_en:        {}\r\n".format(int(self._est_en.get()))).encode())

        # xhat
        if self._xhat_omegaL is not None:
            s.write(("xhat_omegaL:   {}  [rad/s]\r\n".format(self._xhat_omegaL.get())).encode())
        if self._xhat_omegaR is not None:
            s.write(("xhat_omegaR:   {}  [rad/s]\r\n".format(self._xhat_omegaR.get())).encode())
        if self._xhat_s is not None:
            s.write(("xhat_s:        {}  [mm]\r\n".format(self._xhat_s.get())).encode())
        if self._xhat_psi is not None:
            s.write(("xhat_psi:      {}  [rad]\r\n".format(self._xhat_psi.get())).encode())

        # yhat
        if self._yhat_sL is not None:
            s.write(("yhat_sL:       {}  [mm]\r\n".format(self._yhat_sL.get())).encode())
        if self._yhat_sR is not None:
            s.write(("yhat_sR:       {}  [mm]\r\n".format(self._yhat_sR.get())).encode())
        if self._yhat_psi is not None:
            s.write(("yhat_psi:      {}  [rad]\r\n".format(self._yhat_psi.get())).encode())
        if self._yhat_psidot is not None:
            s.write(("yhat_psidot:   {}  [rad/s]\r\n".format(self._yhat_psidot.get())).encode())

        # pose
        if self._x_pos is not None:
            s.write(("x_pos:         {}  [mm]\r\n".format(self._x_pos.get())).encode())
        if self._y_pos is not None:
            s.write(("y_pos:         {}  [mm]\r\n".format(self._y_pos.get())).encode())
        if self._dist_traveled is not None:
            s.write(("dist_traveled: {}  [mm]\r\n".format(self._dist_traveled.get())).encode())

        s.write(b"-----------------------\r\n>: ")

    def _start_stream(self, ms=3000, kind="LINE"):
        now = pyb.millis()
        self._stream_end_ms = now + int(ms)
        self._stream_next_ms = now
        self._stream_kind = kind
        self._ui_mode = "SENSE_STREAM"

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
            s.write(b"Command sent. Now move to BLACK and press 'b'.\r\n>: ")
        else:
            s.write(b"\r\nCalibrating BLACK... hold steady.\r\n")
            self._cal_cmd.put(2)
            s.write(b"Command sent. Calibration complete (disarmed).\r\n>: ")
            self._cal_armed = False

    def ui_poll(self):
        s = self._ser

        if self._start_user.get() != 0:
            return

        if self._ui_mode == "CMD":
            if s.any():
                cmd = s.read(1).decode()
                s.write(cmd.encode())
                c = cmd.lower()

                if c == "a":
                    self._cancel_cal()
                    return

                if c == "h":
                    s.write(b"\r\n")
                    self.print_help()

                elif c == "k":
                    s.write(b"\r\n")
                    self._ui_mode = "GET_KP"

                elif c == "s":
                    s.write(b"\r\n")
                    self._ui_mode = "GET_SP"

                elif c == "g":
                    s.write(b"\r\nStarting data-collection UI...\r\n")
                    self._start_user.put(1)

                elif c == "n":
                    s.write(b"\r\n")
                    if self._leftMotorGo is None or self._rightMotorGo is None:
                        s.write(b"Motor Go shares not wired in main.py\r\n>: ")
                    else:
                        if self._follow_en is not None:
                            self._follow_en.put(1)

                        self._leftMotorGo.put(1)
                        self._rightMotorGo.put(1)

                        if self._Kp_line is not None and self._Ki_line is not None:
                            s.write(("Kp_line:   {}\r\n".format(self._Kp_line.get())).encode())
                            s.write(("Ki_line:   {}\r\n".format(self._Ki_line.get())).encode())

                        s.write(b"Motors RUNNING (LINE-FOLLOW MODE). Streaming line status...\r\n")
                        self._start_stream(3600000, kind="LINE")

                elif c == "x":
                    s.write(b"\r\n")
                    if self._leftMotorGo is None or self._rightMotorGo is None:
                        s.write(b"Motor Go shares not wired in main.py\r\n>: ")
                    else:
                        self._leftMotorGo.put(0)
                        self._rightMotorGo.put(0)
                        if self._follow_en is not None:
                            self._follow_en.put(0)
                        s.write(b"Motors stopped.\r\n>: ")

                elif c == "f":
                    s.write(b"\r\n")
                    if self._follow_en is None:
                        s.write(b"follow_en share not wired in main.py\r\n>: ")
                    else:
                        newv = 0 if int(self._follow_en.get()) else 1
                        self._follow_en.put(newv)
                        s.write(("follow_en set to {}\r\n>: ".format(newv)).encode())

                elif c == "i":
                    s.write(b"\r\n")
                    if self._imu_en is None:
                        s.write(b"imu_en share not wired in main.py\r\n>: ")
                    else:
                        newv = 0 if int(self._imu_en.get()) else 1
                        self._imu_en.put(newv)
                        s.write(("imu_en set to {}\r\n>: ".format(newv)).encode())

                elif c == "m":
                    s.write(b"\r\n")
                    if self._imu_mode is None:
                        s.write(b"imu_mode share not wired in main.py\r\n>: ")
                    else:
                        cur = int(self._imu_mode.get()) & 0xFF
                        # Toggle IMUPLUS (0x08) <-> NDOF (0x0C)
                        newm = 0x0C if cur == 0x08 else 0x08
                        self._imu_mode.put(newm)
                        name = "NDOF" if newm == 0x0C else "IMUPLUS"
                        s.write(("imu_mode set to {} (0x{:02X})\r\n>: ".format(name, newm)).encode())

                elif c == "z":
                    s.write(b"\r\n")
                    if self._imu_zero_cmd is None:
                        s.write(b"imu_zero_cmd share not wired in main.py\r\n>: ")
                    else:
                        self._imu_zero_cmd.put(1)
                        s.write(b"IMU heading zero requested.\r\n>: ")

                elif c == "u":
                    s.write(b"\r\n")
                    if self._imu_save_cmd is None:
                        s.write(b"imu_save_cmd share not wired in main.py\r\n>: ")
                    else:
                        self._imu_save_cmd.put(1)
                        s.write(b"Requested IMU calibration save to bno055_calib.bin\r\n>: ")

                elif c == "o":
                    s.write(b"\r\n")
                    if self._Kp_line is None or self._Ki_line is None:
                        s.write(b"Line gain shares not wired in main.py\r\n>: ")
                    else:
                        self._ui_mode = "GET_LKp"

                elif c == "c":
                    s.write(b"\r\n")
                    self._cal_armed = True
                    self._cal_waiting = False
                    if self._cal_done is not None:
                        self._cal_done.put(0)
                    s.write(b"Calibration ARMED:\r\n")
                    s.write(b"  w = calibrate WHITE\r\n")
                    s.write(b"  b = calibrate BLACK\r\n")
                    s.write(b"  a = cancel/disarm\r\n")
                    s.write(b">: ")

                elif c == "w":
                    if not self._cal_armed:
                        s.write(b"\r\nPress 'c' to arm calibration first.\r\n>: ")
                    else:
                        self._do_cal(1)

                elif c == "b":
                    if not self._cal_armed:
                        s.write(b"\r\nPress 'c' to arm calibration first.\r\n>: ")
                    else:
                        self._do_cal(2)

                elif c == "p":
                    self._print_line_status()

                elif c == "q":
                    s.write(b"\r\nStreaming line status for 3 seconds (press any key to stop)...\r\n")
                    self._start_stream(3000, kind="LINE")

                elif c == "e":
                    self._print_est_status()

                elif c == "r":
                    s.write(b"\r\nStreaming estimator status for 3 seconds (press any key to stop)...\r\n")
                    self._start_stream(3000, kind="EST")

                elif cmd == "\r" or cmd == "\n":
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

        elif self._ui_mode == "GET_LKp":
            val = self.read_number(b"\r\nEnter line Kp_line:\r\n>: ")
            if val is not None:
                self._Kp_line.put(val)
                s.write(("Kp_line set to {}\r\n".format(val)).encode())
            self._ui_mode = "GET_LKi"

        elif self._ui_mode == "GET_LKi":
            val = self.read_number(b"\r\nEnter line Ki_line:\r\n>: ")
            if val is not None:
                self._Ki_line.put(val)
                s.write(("Ki_line set to {}\r\n".format(val)).encode())
            self._ui_mode = "CMD"
            s.write(b"\r\n>: ")

        elif self._ui_mode == "SENSE_STREAM":
            now = pyb.millis()

            if s.any():
                ch = s.read(1).decode()
                if ch.lower() == "x":
                    if self._leftMotorGo is not None and self._rightMotorGo is not None:
                        self._leftMotorGo.put(0)
                        self._rightMotorGo.put(0)
                    if self._follow_en is not None:
                        self._follow_en.put(0)
                    s.write(b"\r\nMotors stopped. Stream stopped.\r\n>: ")
                else:
                    s.write(b"\r\nStream stopped.\r\n>: ")
                self._ui_mode = "CMD"
                return

            if now >= self._stream_end_ms:
                s.write(b"\r\nStream done.\r\n>: ")
                self._ui_mode = "CMD"
                return

            if now >= self._stream_next_ms:
                if self._stream_kind == "EST":
                    # Estimator stream
                    s.write(b"xhat_wL=")
                    s.write(("{}".format(self._xhat_omegaL.get()) if self._xhat_omegaL is not None else "NA").encode())
                    s.write(b"  xhat_wR=")
                    s.write(("{}".format(self._xhat_omegaR.get()) if self._xhat_omegaR is not None else "NA").encode())
                    s.write(b"  xhat_s=")
                    s.write(("{}".format(self._xhat_s.get()) if self._xhat_s is not None else "NA").encode())
                    s.write(b"  xhat_psi=")
                    s.write(("{}".format(self._xhat_psi.get()) if self._xhat_psi is not None else "NA").encode())

                    s.write(b"  x_pos=")
                    s.write(("{}".format(self._x_pos.get()) if self._x_pos is not None else "NA").encode())
                    s.write(b"  y_pos=")
                    s.write(("{}".format(self._y_pos.get()) if self._y_pos is not None else "NA").encode())
                    s.write(b"  dist=")
                    s.write(("{}".format(self._dist_traveled.get()) if self._dist_traveled is not None else "NA").encode())
                    s.write(b"\r\n")

                else:
                    # Existing line + IMU stream
                    le = self._line_err.get() if self._line_err is not None else None
                    dv = self._dv_out.get() if self._dv_out is not None else None
                    fe = int(self._follow_en.get()) if self._follow_en is not None else None

                    s.write(b"line_err=")
                    s.write(("{}".format(le) if le is not None else "NA").encode())
                    s.write(b"  dv_out=")
                    s.write(("{}".format(dv) if dv is not None else "NA").encode())
                    s.write(b"  follow_en=")
                    s.write(("{}".format(fe) if fe is not None else "NA").encode())

                    # IMU stream (optional)
                    if self._imu_heading is not None:
                        s.write(b"  imu_heading=")
                        s.write(("{}".format(self._imu_heading.get())).encode())
                    if self._imu_yawrate is not None:
                        s.write(b"  imu_yawrate=")
                        s.write(("{}".format(self._imu_yawrate.get())).encode())
                    if self._imu_calraw is not None:
                        s.write(b"  imu_cal=")
                        s.write(("0x{:02X}".format(int(self._imu_calraw.get()) & 0xFF)).encode())

                    if self._Kp_line is not None:
                        s.write(b"  Kp_line: ")
                        s.write(("{}".format(self._Kp_line.get())).encode())
                    if self._Ki_line is not None:
                        s.write(b"  Ki_line: ")
                        s.write(("{}".format(self._Ki_line.get())).encode())

                    # State estimator stream (optional, appended)
                    if self._xhat_s is not None:
                        s.write(b"  xhat_s=")
                        s.write(("{}".format(self._xhat_s.get())).encode())
                    if self._xhat_psi is not None:
                        s.write(b"  xhat_psi=")
                        s.write(("{}".format(self._xhat_psi.get())).encode())
                    if self._x_pos is not None:
                        s.write(b"  x_pos=")
                        s.write(("{}".format(self._x_pos.get())).encode())
                    if self._y_pos is not None:
                        s.write(b"  y_pos=")
                        s.write(("{}".format(self._y_pos.get())).encode())
                    if self._dist_traveled is not None:
                        s.write(b"  dist=")
                        s.write(("{}".format(self._dist_traveled.get())).encode())

                    s.write(b"\r\n")

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