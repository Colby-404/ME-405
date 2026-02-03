''' This file demonstrates an example motor task using a custom class with a
    run method implemented as a generator
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
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
                 mot: motor_driver, enc: encoder,
                 goFlag: Share, dataValues: Queue, timeValues: Queue):
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''

        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: motor_driver = mot        # A motor object
        
        self._enc: encoder      = enc        # An encoder object
        
        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection
        
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data
        
        print("Motor Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                # print("Initializing motor task")
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._goFlag.get():
                    # print("Starting motor loop")
                    
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    self._startTime = ticks_us()
                    self._state = S2_RUN
                
            elif self._state == S2_RUN: # Closed-loop control state
                # print(f"Running motor loop, cycle {self._dataValues.num_in()}")
                
                # Run the encoder update algorithm and then capture the present
                # position of the encoder. You will eventually need to capture
                # the motor speed instead of position here.
                self._enc.update()
                pos = self._enc.get_position()
                
                # Collect a timestamp to use for this sample
                t   = ticks_us()
                
                # Actuate the motor using a control law. The one used here in
                # the example is a "bang bang" controller, and will work very
                # poorly in practice. Note that the set position is zero. You
                # will replace this with the output of your PID controller that
                # uses feedback from the velocity measurement.
                self._mot.set_effort(100 if pos < 0 else -100)
                
                # Store the sampled values in the queues
                self._dataValues.put(pos)
                self._timeValues.put(ticks_diff(t, self._startTime))
                
                # When the queues are full, data collection is over
                if self._dataValues.full():
                    # print("Exiting motor loop")
                    self._state = S1_WAIT
                    self._goFlag.put(False)
            
            yield self._state