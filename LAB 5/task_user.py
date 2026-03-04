"""
task_user + task_tuning_ui (Lab 0x05 upgraded, rewritten + FIXED)

Fixes included (final, consistent mapping):
DATA UI (start_user == 1):
  l/r/b = start left/right/both collection
  x     = stop (works during collection too)
  v     = toggle live printing gate (print_live_en)
  h     = return to tuning menu

TUNING UI (start_user == 0):
  h = help
  k = set wheel gains (Kp, Ki)
  s = set v_nom
  g = go to data collection UI
  n = run motors (live test, no logging/printing)
  x = stop motors   
  f = toggle follow_en
  o = set line gains (Kp_line, Ki_line)
  c = arm calibration + show instructions
      w = calibrate WHITE (only when armed)
      B = calibrate BLACK (only when armed)   (capital B avoids conflict with 'b' cancel and 'k' wheel gains)
      b = cancel/disarm calibration
  p = print one-shot line status
  q = stream line status for 3 seconds

NOTE:
- UI only sets cal_cmd flags. Sensor task must watch cal_cmd and call:
    qtr.calibrate_white() when cal_cmd==1
    qtr.calibrate_black() when cal_cmd==2
  then set cal_done=1 when finished.
"""

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
    """
    Data-collection UI task (runs only when start_user == 1)

    Commands:
      l = collect/print LEFT
      r = collect/print RIGHT
      b = collect/print BOTH (simultaneous)
      x = stop (clears go flags)  [WORKS DURING COLLECTION TOO]
      v = toggle LIVE printing while motors run (print_live_en)
      h = return to tuning menu (start_user = 0)
    """

    def __init__(self,
                 start_user: Share,
                 leftMotorGo: Share, rightMotorGo: Share,
                 dataValuesL: Queue, timeValuesL: Queue,
                 dataValuesR: Queue, timeValuesR: Queue,
                 setpoint: Share, Kp: Share, Ki: Share,
                 ser=None,
                 follow_en: Share = None,
                 Kp_line: Share = None,
                 Ki_line: Share = None,
                 line_err: Share = None,
                 dv_out: Share = None,
                 print_live_en: Share = None):

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

        if self._print_live_en is not None:
            s.write(("Live print:       {}\r\n".format(int(self._print_live_en.get()))).encode())

        s.write(b"--------------------------------------------------------------------------------\r\n")

    def _print_footer_and_prompt(self):
        s = self._ser
        s.write(b"--------------------------------------------------------------------------------\r\n")
        s.write(b"Commands: l=left, r=right, b=both, x=stop, v=toggle live print, h=menu\r\n")
        s.write(UI_prompt.encode())

    def _print_section_header(self, label: str):
        s = self._ser
        s.write(b"Data collection complete...\r\nPrinting data...\r\n")
        s.write(b"--------------------------------------------------------------------------------\r\n")
        s.write((label + "\r\n").encode())
        # Generic CSV header (works for speed OR position)
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
                # Allow STOP during collection
                if self._ser.any():
                    ch = self._ser.read(1).decode()
                    if ch in ("x", "X"):
                        self._stop_all()
                        self._ser.write(b"\r\nStopped.\r\n")
                        self._ser.write(UI_prompt.encode())
                        self._state = S1_CMD
                        yield
                        continue
                    # ignore other keys while collecting

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
                # Print CSV rows until queues empty
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
    """
    Tuning menu UI task (runs only when start_user == 0)

    Commands:
      h = help
      k = set wheel Kp, Ki
      s = set v_nom
      g = go to data collection UI
      f = toggle follow_en
      o = set line gains
      c = arm calibration (w=white, b=black, a=cancel)
      p = print one-shot line status
      q = stream line status for 3 seconds
    """

    def __init__(self,
                 start_user: Share,
                 setpoint: Share, Kp: Share, Ki: Share,
                 ser=None,
                 follow_en: Share = None,
                 Kp_line: Share = None,
                 Ki_line: Share = None,
                 cal_cmd: Share = None,
                 cal_done: Share = None,
                 line_err: Share = None,
                 dv_out: Share = None,  
                 leftMotorGo: Share = None,
                rightMotorGo: Share = None,
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
        self._leftMotorGo = leftMotorGo
        self._rightMotorGo = rightMotorGo

        self._digits = set(map(str, range(10)))
        self._term = {"\r", "\n"}

        self._ui_mode = "CMD"  # CMD, GET_KP, GET_KI, GET_SP, GET_LKp, GET_LKi, SENSE_STREAM
        self._menu_shown = False

        # Calibration state
        self._cal_armed = False
        self._cal_waiting = False  # waiting for sensor task to set cal_done=1

        # Streaming
        self._stream_end_ms = 0
        self._stream_next_ms = 0
        self._stream_period_ms = 50

    def print_help(self):
        s = self._ser
        s.write(b"+------------------------------------------------------------------------------+\r\n")
        s.write(b"| ME 405 Romi Tuning Interface (Lab 0x05)                                       |\r\n")
        s.write(b"+---+--------------------------------------------------------------------------+\r\n")
        s.write(b"| h | Help menu                                                                |\r\n")
        s.write(b"| k | Enter wheel gains (Kp, Ki)                                               |\r\n")
        s.write(b"| s | Set V_nom (forward speed setpoint)                                       |\r\n")
        s.write(b"| g | Data Collection UI                                                       |\r\n")
        s.write(b"| n | Run motors (live test, no logging/printing)                              |\r\n")
        s.write(b"| x | Stop motors                                                              |\r\n")
        s.write(b"| f | Toggle line follow enable                                                |\r\n")
        s.write(b"| o | Enter line-follow gains (Kp_line, Ki_line)                               |\r\n")
        s.write(b"| c | Arm calibration (w=white, b=black, a=cancel)                             |\r\n")
        s.write(b"| p | Print line status (err, dv, follow_en)                                   |\r\n")
        s.write(b"| q | Stream line status for 3 seconds                                         |\r\n")
        s.write(b"+---+--------------------------------------------------------------------------+\r\n")
        s.write(b"\r\n>: ")
        pyb.delay(10)

    def _read_char_blocking(self):
        while not self._ser.any():
            pyb.delay(1)
        return self._ser.read(1).decode()

    def read_number(self, prompt: bytes):
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
            elif ch in self._term:
                if len(buf) == 0:
                    s.write(b"\r\nValue not changed\r\n")
                    return None
                if buf in {"-", "."}:
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
        s.write(("cal_armed: {}\r\n".format(int(self._cal_armed))).encode())
        s.write(b"-------------------\r\n>: ")

    def _start_stream(self, ms=3000):
        now = pyb.millis()
        self._stream_end_ms = now + int(ms)
        self._stream_next_ms = now
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

    def _do_cal(self, which: int):
        # which: 1=white, 2=black
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

        # Only active when start_user == 0 (tuning mode)
        if self._start_user.get() != 0:
            return

        if self._ui_mode == "CMD":
            if s.any():
                cmd = s.read(1).decode()
                s.write(cmd.encode())
                c = cmd.lower()  # makes w/W, b/B, a/A all work

                # --- Always-available cancel ---
                if c == "a":
                    self._cancel_cal()
                    return

                if c == "h":
                    s.write(b"\r\n")
                    self.print_help()

                elif c == "k":
                    # Wheel gains entry (always)
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
                        self._leftMotorGo.put(1)
                        self._rightMotorGo.put(1)
                        s.write(b"Motors RUNNING (live mode, no logging).\r\n>: ")

                elif c == "x":
                    s.write(b"\r\n")
                    if self._leftMotorGo is None or self._rightMotorGo is None:
                        s.write(b"Motor Go shares not wired in main.py\r\n>: ")
                    else:
                        self._leftMotorGo.put(0)
                        self._rightMotorGo.put(0)
                        s.write(b"Motors stopped.\r\n>: ")

                elif c == "f":
                    s.write(b"\r\n")
                    if self._follow_en is None:
                        s.write(b"follow_en share not wired in main.py\r\n>: ")
                    else:
                        newv = 0 if int(self._follow_en.get()) else 1
                        self._follow_en.put(newv)
                        s.write(("follow_en set to {}\r\n>: ".format(newv)).encode())

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
                    self._start_stream(3000)

                elif cmd in ("\r", "\n"):
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
                s.read(1)
                s.write(b"\r\nStream stopped.\r\n>: ")
                self._ui_mode = "CMD"
                return

            if now >= self._stream_end_ms:
                s.write(b"\r\nStream done.\r\n>: ")
                self._ui_mode = "CMD"
                return

            if now >= self._stream_next_ms:
                le = self._line_err.get() if self._line_err is not None else None
                dv = self._dv_out.get() if self._dv_out is not None else None
                fe = int(self._follow_en.get()) if self._follow_en is not None else None

                s.write(b"line_err=")
                s.write(("{}".format(le) if le is not None else "NA").encode())
                s.write(b"  dv_out=")
                s.write(("{}".format(dv) if dv is not None else "NA").encode())
                s.write(b"  follow_en=")
                s.write(("{}".format(fe) if fe is not None else "NA").encode())
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

            # Only announce cal_done if we were waiting on a calibration step
            if (self._cal_done is not None and int(self._cal_done.get()) == 1 and self._cal_waiting):
                self._ser.write(b"\r\nCalibration step completed (sensor task reports done).\r\n>: ")
                self._cal_done.put(0)
                self._cal_waiting = False

            self.ui_poll()
            yield