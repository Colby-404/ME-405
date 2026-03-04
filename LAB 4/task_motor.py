from motor_driver import motor_driver
from encoder      import encoder
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0)  # State 0 - initialization
S1_WAIT = micropython.const(1)  # State 1 - wait for go command
S2_RUN  = micropython.const(2)  # State 2 - run closed loop control


class task_motor:
    '''
    Motor task: reads encoder, applies effort, and (optionally) logs data.
    The motor is NEVER disabled due to logging.
    '''

    def __init__(self,
                 mot: motor_driver,
                 enc: encoder,
                 enable: Share,
                 effort: Share,
                 pos_meas: Share,
                 omega_meas: Share,
                 dataValues: Queue = None,
                 timeValues: Queue = None):

        self._state = S0_INIT

        self._mot = mot
        self._enc = enc

        self._enable = enable
        self._effort = effort
        self._pos_meas = pos_meas
        self._omega_meas = omega_meas

        self._dataValues = dataValues
        self._timeValues = timeValues

        self._startTime = 0

        try:
            self._mot.disable()
        except:
            pass

        print("Motor Task object instantiated")

    def run(self):
        # Add these the first time run() executes
        if not hasattr(self, "_logging_enabled"):
            self._logging_enabled = True
        if not hasattr(self, "_motor_enabled"):
            self._motor_enabled = False

        while True:

            # ---------------- INIT ----------------
            if self._state == S0_INIT:
                self._state = S1_WAIT

            # ---------------- WAIT ----------------
            elif self._state == S1_WAIT:
                self._motor_enabled = False
                self._logging_enabled = True  # re-arm logging for next run

                if self._enable.get():
                    self._enc.zero()
                    self._startTime = ticks_us()
                    self._state = S2_RUN

            # ---------------- RUN ----------------
            elif self._state == S2_RUN:

                # 1) Update encoder and publish measurements
                self._enc.update()
                omega = self._enc.get_velocity()
                pos = self._enc.get_position()

                self._omega_meas.put(omega)
                self._pos_meas.put(pos)

                # 2) Apply effort
                if self._enable.get():
                    u = int(self._effort.get())

                    # Enable motor once (less thrash)
                    if not self._motor_enabled:
                        self._mot.enable()
                        self._motor_enabled = True

                    self._mot.set_effort(u)
                else:
                    self._mot.set_effort(0)
                    self._state = S1_WAIT

                # 3) Log data (ONLY logging, never motor control)
                if self._enable.get() and self._dataValues and self._timeValues and self._logging_enabled:
                    if not self._dataValues.full() and not self._timeValues.full():
                        t = ticks_us()
                        self._timeValues.put(ticks_diff(t, self._startTime))
                        self._dataValues.put(float(omega))
                    else:
                        # Buffers full -> stop logging ONLY (do not disable motor)
                        self._logging_enabled = False

            yield self._state