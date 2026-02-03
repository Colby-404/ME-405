
import pyb
from time import ticks_us, ticks_diff   # Use to get dt value in update()

class encoder:

    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chA_pin, chB_pin, period = 65535):   # Uses full range before wrapping

        '''Initializes an Encoder object'''

        self.position   = 0             # Total accumulated position of the encoder
        self.prev_count = 0             # Counter value from the most recent update
        self.delta      = 0             # Change in count between last two updates
        self.dt         = 0             # Amount of time between last two updates
        self.prev_time = ticks_us()     # Units in microseconds

        self.period = period                        # Sets period as the period
        self.half_period = (period + 1) // 2        # Threshold for wrap-around

        # Accept timer number or timer object

        if isinstance(tim, int):
            self.tim = pyb.Timer(tim, prescaler=0, period=period)       # If a timer number is provided, create and configure the timer here   
        else:
            self.tim = tim                                              # If a timer object is provided, use the existing timer without re-initializing it

        # Configure pins for encoder (alternate function)
        self.pinA = pyb.Pin(chA_pin, pyb.Pin.AF_PP)                     # Configure encoder pins for timer alternate function
        self.pinB = pyb.Pin(chB_pin, pyb.Pin.AF_PP)                 

        # Put timer into encoder mode (common MicroPython pattern)
        self.ch1 = self.tim.channel(1, pyb.Timer.ENC_AB, pin=self.pinA)    # Use channel 1 for encoder
        self.ch2 = self.tim.channel(2, pyb.Timer.ENC_AB, pin=self.pinB)    # Use channel 2 for encoder

        # Start prev_count at the actual counter value
        self.prev_count = self.tim.counter()
    
    def update(self):

        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        
        now = ticks_us()                           # Current time 

        self.dt = ticks_diff(now, self.prev_time)  # How much time has passed 
        self.prev_time = now                       # Saves last know time

        cur = self.tim.counter()                   # Current count
        raw_delta = cur - self.prev_count          # Differace betwen current count - last known count 

        if raw_delta > self.half_period:
            raw_delta -= (self.period + 1)

        elif raw_delta < -self.half_period:
            raw_delta += (self.period + 1)

        self.delta = raw_delta
        self.position += raw_delta
        self.prev_count = cur

    def get_position(self):

        '''Returns the most recently updated value of position as determined
           within the update() method'''
        
        return self.position
            
    def get_velocity(self):

        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        
        if self.dt == 0:
            return 0
        
        return self.delta/self.dt
    
    def zero(self):

        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        
        self.position   = 0             # Total accumulated position of the encoder
        self.delta      = 0             # Change in count between last two updates
        self.dt         = 0             # Amount of time between last two updates

        self.prev_time = ticks_us() 
        self.prev_count = self.tim.counter() # Creating the reference


