''' 
task_user - A user interface task that reads character commands from a serial port and
manipulates shared variables to communicate with other tasks based on the user
commands. 

The user can enter "l" or "r" to start data collection on the left or right
motor and encoder pair, respectively. While data collection is occurring, the
UI task blocks out the user interface and waits for the data collection to end.
When the data collection ends, the UI task prints the collected data over the
serial port in a comma-separated format with time stamps.

MODIFIED:
- Supports BOTH left and right data buffers in ONE user task (only one task reads USB_VCP)
- Prints Setpoint, Kp, Ki at the top before testing
- User enters:
    'l' -> collect/print LEFT
    'r' -> collect/print RIGHT
    'b' -> collect/print BOTH
    'x' -> stop (clears go flags)
    'h' -> return to tuning menu
'''

''' 
task_user - A user interface task that reads character commands from a serial port and
manipulates shared variables to communicate with other tasks based on the user commands. 

Commands:
    'l' -> collect/print LEFT
    'r' -> collect/print RIGHT
    'b' -> collect/print BOTH
    'x' -> stop (clears go flags)
    'h' -> return to tuning menu
'''

import micropython
import pyb
from pyb import USB_VCP
from task_share import Share, Queue

# ---------------- FSM States ----------------
S0_INIT = micropython.const(0)
S1_CMD  = micropython.const(1)
S2_COL  = micropython.const(2)
S3_DIS  = micropython.const(3)

# ---------------- Modes ----------------
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
      x = stop (clears go flags)
      h = return to tuning menu (start_user = 0)
    """

    def __init__(self,
                 start_user: Share,
                 leftMotorGo: Share, rightMotorGo: Share,
                 dataValuesL: Queue, timeValuesL: Queue,
                 dataValuesR: Queue, timeValuesR: Queue,
                 setpoint: Share, Kp: Share, Ki: Share,
                 ser=None):

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

        # Gains/setpoint shares
        self._setpoint = setpoint
        self._Kp = Kp
        self._Ki = Ki

        self._mode = MODE_NONE

        # For MODE_BOTH printing
        self._both_phase = 0  # 0=print left, 1=print right

        self._ser.write(b"task_user instantiated\r\n")

    def _print_gains_header(self):
        self._ser.write(b"--------------------------------------------------------------------------------\r\n")
        self._ser.write(("Setpoint: {}\r\n".format(self._setpoint.get())).encode())
        self._ser.write(("Kp:       {}\r\n".format(self._Kp.get())).encode())
        self._ser.write(("Ki:       {}\r\n".format(self._Ki.get())).encode())
        self._ser.write(b"--------------------------------------------------------------------------------\r\n")

    def _start_left(self):
        self._dataL.clear()
        self._timeL.clear()

        self._leftMotorGo.put(True)
        self._rightMotorGo.put(False)

        self._mode = MODE_L
        self._print_gains_header()
        self._ser.write(b"Starting LEFT data collection...\r\nPlease wait...\r\n")

    def _start_right(self):
        self._dataR.clear()
        self._timeR.clear()

        self._rightMotorGo.put(True)
        self._leftMotorGo.put(False)

        self._mode = MODE_R
        self._print_gains_header()
        self._ser.write(b"Starting RIGHT data collection...\r\nPlease wait...\r\n")

    def _start_both(self):
        self._dataL.clear()
        self._timeL.clear()
        self._dataR.clear()
        self._timeR.clear()

        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)

        self._mode = MODE_BOTH
        self._both_phase = 0
        self._print_gains_header()
        self._ser.write(b"Starting BOTH data collection (simultaneous)...\r\nPlease wait...\r\n")

    def _print_section_header(self, label: str):
        # Important: label lines contain no commas, so your PC parser won't treat them as data
        self._ser.write(b"Data collection complete...\r\nPrinting data...\r\n")
        self._ser.write(b"--------------------------------------------------------------------------------\r\n")
        self._ser.write((label + "\r\n").encode())
        # This is what the PC parser keys off of:
        self._ser.write(b"Time [us], Position [counts]\r\n")

    def _print_footer_and_prompt(self):
        self._ser.write(b"--------------------------------------------------------------------------------\r\n")
        self._ser.write(b"Commands: l=left, r=right, b=both, x=stop, h=menu\r\n")
        self._ser.write(UI_prompt.encode())

    def run(self):
        while True:

            # If tuning menu is active, keep this task idle
            if self._start_user.get() == 0:
                self._state = S0_INIT
                yield
                continue

            # ---------------- State 0: Init ----------------
            if self._state == S0_INIT:
                self._ser.write(b"\r\nUser task active.\r\n")
                self._ser.write(b"Commands: l=left, r=right, b=both, x=stop, h=menu\r\n")
                self._ser.write(UI_prompt.encode())
                self._mode = MODE_NONE
                self._both_phase = 0
                self._state = S1_CMD

            # ---------------- State 1: Wait for command ----------------
            elif self._state == S1_CMD:
                if self._ser.any():
                    inChar = self._ser.read(1).decode()

                    if inChar in {"l", "L"}:
                        self._ser.write((inChar + "\r\n").encode())
                        self._start_left()
                        self._state = S2_COL

                    elif inChar in {"r", "R"}:
                        self._ser.write((inChar + "\r\n").encode())
                        self._start_right()
                        self._state = S2_COL

                    elif inChar in {"b", "B"}:
                        self._ser.write((inChar + "\r\n").encode())
                        self._start_both()
                        self._state = S2_COL

                    elif inChar in {"x", "X"}:
                        self._leftMotorGo.put(False)
                        self._rightMotorGo.put(False)
                        self._mode = MODE_NONE
                        self._ser.write(b"Stopping...\r\n")
                        self._ser.write(UI_prompt.encode())

                    elif inChar in {"h", "H"}:
                        self._leftMotorGo.put(False)
                        self._rightMotorGo.put(False)
                        self._mode = MODE_NONE
                        self._ser.write(b"Returning to tuning menu...\r\n")
                        self._start_user.put(0)
                        self._state = S0_INIT

            # ---------------- State 2: Collect data (lock out UI) ----------------
            elif self._state == S2_COL:
                # Drain serial so keys don't queue up while collecting
                if self._ser.any():
                    self._ser.read(1)

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
                        # Stop BOTH and transition to printing
                        self._leftMotorGo.put(False)
                        self._rightMotorGo.put(False)
                        self._both_phase = 0
                        self._print_section_header("LEFT DATA")
                        self._state = S3_DIS

            # ---------------- State 3: Print CSV ----------------
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
                    # Phase 0: print all LEFT data
                    if self._both_phase == 0:
                        if self._dataL.any():
                            self._ser.write("{},{}\r\n".format(self._timeL.get(), self._dataL.get()).encode())
                        else:
                            # Switch to RIGHT section header
                            self._both_phase = 1
                            self._ser.write(b"--------------------------------------------------------------------------------\r\n")
                            self._print_section_header("RIGHT DATA")

                    # Phase 1: print all RIGHT data
                    else:
                        if self._dataR.any():
                            self._ser.write("{},{}\r\n".format(self._timeR.get(), self._dataR.get()).encode())
                        else:
                            self._print_footer_and_prompt()
                            self._mode = MODE_NONE
                            self._state = S1_CMD

            yield


# NOTE: Your task_tuning_ui can remain exactly as you had it.
# If you want me to paste it back in unchanged, tell me and I’ll include it.


class task_tuning_ui:
    """
    Tuning menu UI task (runs only when start_user == 0)

    Commands:
      h = help
      k = enter Kp, Ki
      s = enter setpoint
      g = start task_user (start_user=1)
    """

    def __init__(self, start_user: Share, setpoint: Share, Kp: Share, Ki: Share, ser=None):
        self._start_user = start_user
        self._setpoint = setpoint
        self._Kp = Kp
        self._Ki = Ki

        self._ser = ser if ser is not None else USB_VCP()

        self._digits = set(map(str, range(10)))
        self._term = {"\r", "\n"}

        self._ui_mode = "CMD"  # CMD, GET_KP, GET_KI, GET_SP
        self._menu_shown = False

    def print_help(self):
        s = self._ser
        s.write(b"+------------------------------------------------------------------------------+\r\n")
        s.write(b"| ME 405 Romi Tuning Interface Help Menu                                       |\r\n")
        s.write(b"+---+--------------------------------------------------------------------------+\r\n")
        s.write(b"| h | Print help menu                                                          |\r\n")
        s.write(b"| k | Enter new gain values                                                    |\r\n")
        s.write(b"| s | Choose a new setpoint                                                    |\r\n")
        s.write(b"| g | Go to data collection UI                                                 |\r\n")
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

    def ui_poll(self):
        s = self._ser

        # If data UI is active, do nothing
        if self._start_user.get() != 0:
            return

        if self._ui_mode == "CMD":
            if s.any():
                cmd = s.read(1).decode()
                s.write(cmd.encode())

                if cmd == "h":
                    s.write(b"\r\n")
                    self.print_help()

                elif cmd == "k":
                    s.write(b"\r\n")
                    self._ui_mode = "GET_KP"

                elif cmd == "s":
                    s.write(b"\r\n")
                    self._ui_mode = "GET_SP"

                elif cmd == "g":
                    s.write(b"\r\nStarting data-collection UI...\r\n")
                    self._start_user.put(1)
                    s.write(b">: ")

                elif cmd in ("\r", "\n"):
                    s.write(b"\r\n>: ")

                else:
                    s.write(b"\r\nUnknown command. Press 'h' for help.\r\n>: ")

        elif self._ui_mode == "GET_KP":
            val = self.read_number(b"\r\nEnter proportional gain, Kp:\r\n>: ")
            if val is not None:
                self._Kp.put(val)
                s.write(("Kp set to {}\r\n".format(val)).encode())
            self._ui_mode = "GET_KI"

        elif self._ui_mode == "GET_KI":
            val = self.read_number(b"\r\nEnter integral gain, Ki:\r\n>: ")
            if val is not None:
                self._Ki.put(val)
                s.write(("Ki set to {}\r\n".format(val)).encode())
            self._ui_mode = "CMD"
            s.write(b"\r\n>: ")

        elif self._ui_mode == "GET_SP":
            val = self.read_number(b"\r\nEnter setpoint value:\r\n>: ")
            if val is not None:
                self._setpoint.put(val)
                s.write(("Setpoint set to {}\r\n".format(val)).encode())
            self._ui_mode = "CMD"
            s.write(b"\r\n>: ")

    def run(self):
        while True:
            if self._start_user.get() == 0 and not self._menu_shown:
                self.print_help()
                self._menu_shown = True

            if self._start_user.get() != 0:
                self._menu_shown = False
                self._ui_mode = "CMD"

            self.ui_poll()
            yield