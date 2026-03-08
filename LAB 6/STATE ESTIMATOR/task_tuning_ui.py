# task_tuning_ui.py
# Lean tuning/stream UI for Romi (MicroPython, low-RAM build)

import pyb
from pyb import USB_VCP


class task_tuning_ui:

    def __init__(self, v_nom, Kp, Ki, ser=None,
                 motors=None, line=None, cal=None, imu=None, enc=None, est=None):

        self._ser = ser if ser is not None else USB_VCP()

        # Wheel/forward setpoint + inner gains
        self._v_nom = v_nom
        self._Kp = Kp
        self._Ki = Ki

        # Optional groups
        self._leftGo = None
        self._rightGo = None
        if motors is not None:
            self._leftGo, self._rightGo = motors

        self._follow_en = None
        self._Kp_line = None
        self._Ki_line = None
        self._line_err = None
        self._dv_out = None
        if line is not None:
            self._follow_en, self._Kp_line, self._Ki_line, self._line_err, self._dv_out = line

        self._cal_cmd = None
        self._cal_done = None
        if cal is not None:
            self._cal_cmd, self._cal_done = cal

        self._imu_en = None
        self._imu_mode = None
        self._imu_zero_cmd = None
        self._imu_save_cmd = None
        self._imu_heading = None
        self._imu_yawrate = None
        self._imu_calraw = None
        if imu is not None:
            (self._imu_en, self._imu_mode, self._imu_zero_cmd, self._imu_save_cmd,
             self._imu_heading, self._imu_yawrate, self._imu_calraw) = imu

        # Encoder position shares (optional; counts)
        self._posL = None
        self._posR = None
        if enc is not None:
            self._posL, self._posR = enc

        # Estimator shares
        self._est_en = None
        self._xhat_s = None
        self._xhat_psi = None
        self._x_pos = None
        self._y_pos = None
        self._dist = None
        # Optional yhat (estimated outputs) for proof/logging
        self._yhat_sL = None
        self._yhat_sR = None
        self._yhat_psi = None
        self._yhat_psidot = None
        if est is not None:
            # est tuple supports either:
            #   (est_en, xhat_s, xhat_psi, x_pos, y_pos, dist)
            # or extended:
            #   (est_en, xhat_s, xhat_psi, x_pos, y_pos, dist, yhat_sL, yhat_sR, yhat_psi, yhat_psidot)
            try:
                self._est_en = est[0]
                self._xhat_s = est[1]
                self._xhat_psi = est[2]
                self._x_pos = est[3]
                self._y_pos = est[4]
                self._dist  = est[5]
                if len(est) >= 10:
                    self._yhat_sL = est[6]
                    self._yhat_sR = est[7]
                    self._yhat_psi = est[8]
                    self._yhat_psidot = est[9]
            except Exception:
                pass

        # Calibration state
        self._cal_armed = False
        self._cal_waiting = False

        # Streaming
        self._streaming = False
        self._stream_kind = 0   # 0=line, 1=est
        self._stream_end_ms = 0
        self._stream_next_ms = 0
        self._stream_period_ms = 100


        # Extra logging can steal CPU time and make the line follower look wobbly.
        # Keep verbose logging OFF by default; you can toggle it with the 'v' command.
        self._stream_verbose = False
        self._stream_verbose_period_ms = 120
        # Startup
        self._menu_shown = False

    # --------- helpers ---------

    def _w(self, b):
        self._ser.write(b)

    def _wnum(self, x):
        self._ser.write(str(x).encode())

    def print_help(self):
        # Lazy import to save RAM at startup
        import ui_help
        ui_help.print_help(self._ser)

    def _read_char_blocking(self):
        while not self._ser.any():
            pyb.delay(1)
        return self._ser.read(1).decode()

    def _read_number(self, prompt_bytes):
        s = self._ser
        s.write(prompt_bytes)
        buf = ""
        while True:
            ch = self._read_char_blocking()
            if ch.isdigit() or (ch == "." and "." not in buf) or (ch == "-" and len(buf) == 0):
                s.write(ch.encode()); buf += ch
            elif ch == "\x7f" and len(buf) > 0:
                s.write(ch.encode()); buf = buf[:-1]
            elif ch == "\r" or ch == "\n":
                if not buf or buf == "-" or buf == ".":
                    s.write(b"\r\n(no change)\r\n>: ")
                    return None
                s.write(b"\r\n")
                try:
                    return float(buf)
                except Exception:
                    s.write(b"bad\r\n>: ")
                    return None

    def _print_line_status(self):
        s = self._ser
        s.write(b"\r\nLINE ")
        if self._follow_en is not None:
            s.write(b"follow_en="); s.write(str(int(self._follow_en.get())).encode()); s.write(b" ")
        if self._line_err is not None:
            s.write(b"err="); s.write(str(self._line_err.get()).encode()); s.write(b" ")
        if self._dv_out is not None:
            s.write(b"dv_out="); s.write(str(self._dv_out.get()).encode()); s.write(b" ")
        if self._imu_heading is not None:
            s.write(b"hdg="); s.write(str(self._imu_heading.get()).encode()); s.write(b" ")
        if self._imu_yawrate is not None:
            s.write(b"wz="); s.write(str(self._imu_yawrate.get()).encode()); s.write(b" ")
        # measured wheel positions (counts)
        if self._posL is not None:
            s.write(b"posL="); s.write(str(self._posL.get()).encode()); s.write(b" ")
        if self._posR is not None:
            s.write(b"posR="); s.write(str(self._posR.get()).encode()); s.write(b" ")
        # estimated outputs (yhat)
        if self._yhat_sL is not None:
            s.write(b"yhat_sL="); s.write(str(self._yhat_sL.get()).encode()); s.write(b" ")
        if self._yhat_sR is not None:
            s.write(b"yhat_sR="); s.write(str(self._yhat_sR.get()).encode()); s.write(b" ")
        if self._yhat_psi is not None:
            s.write(b"yhat_psi="); s.write(str(self._yhat_psi.get()).encode()); s.write(b" ")
        if self._yhat_psidot is not None:
            s.write(b"yhat_psidot="); s.write(str(self._yhat_psidot.get()).encode()); s.write(b" ")
        if self._imu_calraw is not None:
            s.write(b"cal="); s.write(("0x%02X" % (int(self._imu_calraw.get()) & 0xFF)).encode())
        s.write(b"\r\n>: ")

    def _print_est_status(self):
        s = self._ser
        if self._xhat_s is None and self._xhat_psi is None and self._x_pos is None and self._y_pos is None and self._dist is None:
            s.write(b"\r\nEST not wired\r\n>: ")
            return
        s.write(b"\r\nEST ")
        if self._est_en is not None:
            s.write(b"est_en="); s.write(str(int(self._est_en.get())).encode()); s.write(b" ")
        if self._xhat_s is not None:
            s.write(b"s="); s.write(str(self._xhat_s.get()).encode()); s.write(b" ")
        else:
            s.write(b"s=NA ")
        if self._xhat_psi is not None:
            s.write(b"psi="); s.write(str(self._xhat_psi.get()).encode()); s.write(b" ")
        else:
            s.write(b"psi=NA ")
        if self._x_pos is not None:
            s.write(b"x="); s.write(str(self._x_pos.get()).encode()); s.write(b" ")
        else:
            s.write(b"x=NA ")
        if self._y_pos is not None:
            s.write(b"y="); s.write(str(self._y_pos.get()).encode()); s.write(b" ")
        else:
            s.write(b"y=NA ")
        if self._dist is not None:
            s.write(b"d="); s.write(str(self._dist.get()).encode())
        else:
            s.write(b"d=NA")
        s.write(b"\r\n>: ")

    def _start_stream(self, ms, kind):
        now = pyb.millis()
        self._streaming = True
        self._stream_kind = kind
        self._stream_end_ms = now + int(ms)
        self._stream_next_ms = now

    def _stop_stream(self):
        self._streaming = False

    # --------- calibration ---------

    def _cancel_cal(self):
        self._cal_armed = False
        self._cal_waiting = False
        if self._cal_cmd is not None:
            self._cal_cmd.put(0)
        if self._cal_done is not None:
            self._cal_done.put(0)
        self._ser.write(b"\r\ncal cancel\r\n>: ")

    def _do_cal(self, which):
        if self._cal_cmd is None:
            self._ser.write(b"\r\ncal not wired\r\n>: ")
            return
        self._cal_waiting = True
        if self._cal_done is not None:
            self._cal_done.put(0)
        if which == 1:
            self._ser.write(b"\r\ncal WHITE\r\n>: ")
            self._cal_cmd.put(1)
        else:
            self._ser.write(b"\r\ncal BLACK\r\n>: ")
            self._cal_cmd.put(2)
            self._cal_armed = False

    # --------- main loop ---------

    def run(self):
        s = self._ser

        while True:
            if not self._menu_shown:
                self.print_help()
                self._menu_shown = True

            # report calibration done (if used)
            if self._cal_done is not None and self._cal_waiting:
                try:
                    if int(self._cal_done.get()) == 1:
                        s.write(b"\r\ncal done\r\n>: ")
                        self._cal_done.put(0)
                        self._cal_waiting = False
                except Exception:
                    pass

            # stop stream on any key
            if self._streaming:
                if s.any():
                    ch = s.read(1).decode()
                    if ch.lower() == "x":
                        if self._leftGo is not None and self._rightGo is not None:
                            self._leftGo.put(0); self._rightGo.put(0)
                        if self._follow_en is not None:
                            self._follow_en.put(0)
                        s.write(b"\r\nstop\r\n>: ")
                    else:
                        s.write(b"\r\nstream stop\r\n>: ")
                    self._stop_stream()
                    yield
                    continue

                now = pyb.millis()
                if now >= self._stream_end_ms:
                    s.write(b"\r\nstream done\r\n>: ")
                    self._stop_stream()
                    yield
                    continue

                if now >= self._stream_next_ms:
                    if self._stream_kind == 1:
                        # EST stream (one line)
                        s.write(b"est ")
                        if self._xhat_s is not None:
                            s.write(b"s="); s.write(str(self._xhat_s.get()).encode()); s.write(b" ")
                        else:
                            s.write(b"s=NA ")
                        if self._xhat_psi is not None:
                            s.write(b"psi="); s.write(str(self._xhat_psi.get()).encode()); s.write(b" ")
                        else:
                            s.write(b"psi=NA ")
                        # yhat (estimated outputs)
                        if self._yhat_sL is not None:
                            s.write(b"yhat_sL="); s.write(str(self._yhat_sL.get()).encode()); s.write(b" ")
                        if self._yhat_sR is not None:
                            s.write(b"yhat_sR="); s.write(str(self._yhat_sR.get()).encode()); s.write(b" ")
                        if self._yhat_psi is not None:
                            s.write(b"yhat_psi="); s.write(str(self._yhat_psi.get()).encode()); s.write(b" ")
                        if self._yhat_psidot is not None:
                            s.write(b"yhat_psidot="); s.write(str(self._yhat_psidot.get()).encode()); s.write(b" ")
                        if self._x_pos is not None:
                            s.write(b"x="); s.write(str(self._x_pos.get()).encode()); s.write(b" ")
                        else:
                            s.write(b"x=NA ")
                        if self._y_pos is not None:
                            s.write(b"y="); s.write(str(self._y_pos.get()).encode()); s.write(b" ")
                        else:
                            s.write(b"y=NA ")
                        if self._dist is not None:
                            s.write(b"d="); s.write(str(self._dist.get()).encode())
                        else:
                            s.write(b"d=NA")
                        s.write(b"\r\n")
                    else:
                        # LINE stream (one line, plus appended estimator fields for step-collector)
                        le = self._line_err.get() if self._line_err is not None else None
                        dv = self._dv_out.get() if self._dv_out is not None else None
                        fe = int(self._follow_en.get()) if self._follow_en is not None else None

                        s.write(b"line_err="); s.write((str(le) if le is not None else "NA").encode())
                        s.write(b" dv_out="); s.write((str(dv) if dv is not None else "NA").encode())
                        s.write(b" follow_en="); s.write((str(fe) if fe is not None else "NA").encode())

                        if self._imu_heading is not None:
                            s.write(b" hdg="); s.write(str(self._imu_heading.get()).encode())
                        if self._imu_yawrate is not None:
                            s.write(b" wz="); s.write(str(self._imu_yawrate.get()).encode())

                        # estimator append (used by step_collector_loop.py)
                        if self._xhat_s is not None:
                            s.write(b" xhat_s="); s.write(str(self._xhat_s.get()).encode())
                        if self._xhat_psi is not None:
                            s.write(b" xhat_psi="); s.write(str(self._xhat_psi.get()).encode())
                        if self._x_pos is not None:
                            s.write(b" x_pos="); s.write(str(self._x_pos.get()).encode())
                        if self._y_pos is not None:
                            s.write(b" y_pos="); s.write(str(self._y_pos.get()).encode())
                        if self._dist is not None:
                            s.write(b" dist="); s.write(str(self._dist.get()).encode())

                        # Optional verbose append: measured wheel positions + estimated outputs.
                        # NOTE: Extra USB printing can steal CPU time.
                        if self._stream_verbose:
                            # measured wheel positions (counts)
                            if self._posL is not None:
                                s.write(b" posL="); s.write(str(self._posL.get()).encode())
                            if self._posR is not None:
                                s.write(b" posR="); s.write(str(self._posR.get()).encode())
                            # estimated outputs (yhat)
                            if self._yhat_sL is not None:
                                s.write(b" yhat_sL="); s.write(str(self._yhat_sL.get()).encode())
                            if self._yhat_sR is not None:
                                s.write(b" yhat_sR="); s.write(str(self._yhat_sR.get()).encode())
                            if self._yhat_psi is not None:
                                s.write(b" yhat_psi="); s.write(str(self._yhat_psi.get()).encode())
                            if self._yhat_psidot is not None:
                                s.write(b" yhat_psidot="); s.write(str(self._yhat_psidot.get()).encode())

                        s.write(b"\r\n")

                    self._stream_next_ms = now + (self._stream_verbose_period_ms if self._stream_verbose else self._stream_period_ms)

                yield
                continue

            # --------- command mode ---------
            if s.any():
                cmd = s.read(1).decode()
                c = cmd.lower()
                s.write(cmd.encode())

                if c == "h":
                    s.write(b"\r\n")
                    self.print_help()

                elif c == "v":
                    # Toggle verbose streaming (adds posL/posR and yhat_* fields).
                    self._stream_verbose = not self._stream_verbose
                    s.write(b"\r\nverbose_stream=")
                    s.write(str(int(self._stream_verbose)).encode())
                    if self._stream_verbose:
                        s.write(b" (slower stream)")
                    s.write(b"\r\n>: ")

                elif c == "k":
                    v = self._read_number(b"\r\nKp?:\r\n>: ")
                    if v is not None:
                        self._Kp.put(v); s.write(b"Kp="); s.write(str(v).encode()); s.write(b"\r\n")
                    v = self._read_number(b"Ki?:\r\n>: ")
                    if v is not None:
                        self._Ki.put(v); s.write(b"Ki="); s.write(str(v).encode()); s.write(b"\r\n>: ")
                    else:
                        s.write(b">: ")

                elif c == "s":
                    v = self._read_number(b"\r\nV_nom?:\r\n>: ")
                    if v is not None:
                        self._v_nom.put(v); s.write(b"V="); s.write(str(v).encode()); s.write(b"\r\n>: ")
                    else:
                        s.write(b">: ")

                elif c == "o":
                    if self._Kp_line is None or self._Ki_line is None:
                        s.write(b"\r\nline gains NA\r\n>: ")
                    else:
                        v = self._read_number(b"\r\nKp_line?:\r\n>: ")
                        if v is not None:
                            self._Kp_line.put(v)
                        v = self._read_number(b"Ki_line?:\r\n>: ")
                        if v is not None:
                            self._Ki_line.put(v)
                        s.write(b">: ")

                elif c == "n":
                    s.write(b"\r\n")
                    if self._leftGo is None or self._rightGo is None:
                        s.write(b"motors NA\r\n>: ")
                    else:
                        if self._follow_en is not None:
                            self._follow_en.put(1)
                        self._leftGo.put(1); self._rightGo.put(1)
                        s.write(b"run+stream\r\n")
                        self._start_stream(3600000, 0)

                elif c == "x":
                    s.write(b"\r\n")
                    if self._leftGo is not None and self._rightGo is not None:
                        self._leftGo.put(0); self._rightGo.put(0)
                    if self._follow_en is not None:
                        self._follow_en.put(0)
                    s.write(b"stop\r\n>: ")

                elif c == "f":
                    s.write(b"\r\n")
                    if self._follow_en is None:
                        s.write(b"follow NA\r\n>: ")
                    else:
                        nv = 0 if int(self._follow_en.get()) else 1
                        self._follow_en.put(nv)
                        s.write(b"follow="); s.write(str(nv).encode()); s.write(b"\r\n>: ")

                elif c == "p":
                    self._print_line_status()

                elif c == "q":
                    s.write(b"\r\nstream line (3s)\r\n")
                    self._start_stream(3000, 0)

                elif c == "e":
                    self._print_est_status()

                elif c == "r":
                    s.write(b"\r\nstream est (3s)\r\n")
                    self._start_stream(3000, 1)

                elif c == "i":
                    s.write(b"\r\n")
                    if self._imu_en is None:
                        s.write(b"imu NA\r\n>: ")
                    else:
                        nv = 0 if int(self._imu_en.get()) else 1
                        self._imu_en.put(nv)
                        s.write(b"imu="); s.write(str(nv).encode()); s.write(b"\r\n>: ")

                elif c == "m":
                    s.write(b"\r\n")
                    if self._imu_mode is None:
                        s.write(b"mode NA\r\n>: ")
                    else:
                        cur = int(self._imu_mode.get()) & 0xFF
                        nv = 0x0C if cur == 0x08 else 0x08
                        self._imu_mode.put(nv)
                        s.write(b"mode=0x"); s.write(("%02X" % nv).encode()); s.write(b"\r\n>: ")

                elif c == "z":
                    s.write(b"\r\n")
                    if self._imu_zero_cmd is None:
                        s.write(b"zero NA\r\n>: ")
                    else:
                        self._imu_zero_cmd.put(1)
                        s.write(b"zero\r\n>: ")

                elif c == "u":
                    s.write(b"\r\n")
                    if self._imu_save_cmd is None:
                        s.write(b"save NA\r\n>: ")
                    else:
                        self._imu_save_cmd.put(1)
                        s.write(b"save\r\n>: ")

                elif c == "c":
                    s.write(b"\r\n")
                    self._cal_armed = True
                    self._cal_waiting = False
                    if self._cal_done is not None:
                        self._cal_done.put(0)
                    s.write(b"cal armed (w/b/a)\r\n>: ")

                elif c == "a":
                    self._cancel_cal()

                elif c == "w":
                    if not self._cal_armed:
                        s.write(b"\r\narm cal first (c)\r\n>: ")
                    else:
                        self._do_cal(1)

                elif c == "b":
                    if not self._cal_armed:
                        s.write(b"\r\narm cal first (c)\r\n>: ")
                    else:
                        self._do_cal(2)

                elif cmd == "\r" or cmd == "\n":
                    s.write(b"\r\n>: ")

                else:
                    s.write(b"\r\n?\r\n>: ")

            yield
