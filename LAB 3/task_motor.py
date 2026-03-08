''' 
'''
from motor_driver import motor_driver
from encoder      import encoder
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
                mot: motor_driver,
                enc: encoder,
                enable: Share,
                effort: Share,
                omega_meas: Share,
                dataValues: Queue = None,
                timeValues: Queue = None):
        
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            enable (Share):  A share object representing a boolean flag to
                                enable motor control
            effort (Share):  A share object representing the desired motor effort
            omega_meas (Share): A share object representing the measured motor omega
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''

        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: motor_driver = mot        # A motor object
        
        self._enc: encoder      = enc        # An encoder object

        self._enable: Share   = enable  # Enable flag (0/1) for motor control

        self._effort: Share   = effort  # The desired motor effort (output of the controller)

        self._omega_meas: Share = omega_meas # The measured motor omega (feedback to the controller)
        
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data
        
        try: 
            self._mot.disable()
        except:
            pass

        print("Motor Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
    
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                #print("Initializing motor task")
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._enable.get():
                   #print("Starting motor loop")
                    
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    self._startTime = ticks_us()
                    self._state = S2_RUN
                
            elif self._state == S2_RUN:  # Closed-loop control state

                # 1) Update encoder + publish omega to control loop
                self._enc.update()
                omega = self._enc.get_velocity()
                self._omega_meas.put(omega)

                # 2) Apply effort if enabled
                if self._enable.get():
                    u = int(self._effort.get())
                    self._mot.enable()
                    self._mot.set_effort(u)
                else:
                    self._mot.set_effort(0)
                    try:
                        self._mot.disable()
                    except:
                        pass
                    self._state = S1_WAIT

                # 3) Data collection (only if queues exist)
                if self._dataValues is not None and self._timeValues is not None:
                    if (not self._dataValues.full()) and (not self._timeValues.full()):
                        t = ticks_us()
                        self._timeValues.put(ticks_diff(t, self._startTime))
                        self._dataValues.put(float(omega))
                    else:
                        # buffers full -> stop this motor + stop collection
                        self._mot.set_effort(0)
                        try:
                            self._mot.disable()
                        except:
                            pass
                        self._enable.put(0)     # clears the go flag so UI can print
                        self._state = S1_WAIT



 
            yield self._state

