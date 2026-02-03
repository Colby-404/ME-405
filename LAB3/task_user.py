''' This file demonstrates an example UI task using a custom class with a
    run method implemented as a generator
'''
from pyb import USB_VCP
from task_share import Share
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_CMD  = micropython.const(1) # State 1 - wait for character input
S2_COL  = micropython.const(2) # State 2 - wait for data collection to end
S3_DIS  = micropython.const(3) # State 3 - display the collected data

UI_prompt = ">: "

class task_user:
    '''
    A class that represents a UI task. The task is responsible for reading user
    input over a serial port, parsing the input for single-character commands,
    and then manipulating shared variables to communicate with other tasks based
    on the user commands.
    '''

    def __init__(self, leftMotorGo, rightMotorGo, dataValues, timeValues):
        '''
        Initializes a UI task object
        
        Args:
            leftMotorGo (Share):  A share object representing a boolean flag to
                                  start data collection on the left motor
            rightMotorGo (Share): A share object representing a boolean flag to
                                  start data collection on the right motor
            dataValues (Queue):   A queue object used to store collected encoder
                                  position values
            timeValues (Queue):   A queue object used to store the time stamps
                                  associated with the collected encoder data
        '''
        
        self._state: int          = S0_INIT      # The present state
        
        self._leftMotorGo: Share  = leftMotorGo  # The "go" flag to start data
                                                 # collection from the left
                                                 # motor and encoder pair
        
        self._rightMotorGo: Share = rightMotorGo # The "go" flag to start data
                                                 # collection from the right
                                                 # motor and encoder pair
        
        self._ser: stream         = USB_VCP()    # A serial port object used to
                                                 # read character entry and to
                                                 # print output
        
        self._dataValues: Queue   = dataValues   # A reusable buffer for data
                                                 # collection
        
        self._timeValues: Queue   = timeValues   # A reusable buffer for time
                                                 # stamping collected data
        
        self._ser.write("User Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                self._ser.write("Initializing user task\r\n")
                self._ser.write("Waiting for go command: 'l' for left, 'r' for right\r\n")
                self._ser.write(UI_prompt)
                self._state = S1_CMD
                
            elif self._state == S1_CMD: # Wait for UI commands
                # Wait for at least one character in serial buffer
                if self._ser.any():
                    # Read the character and decode it into a string
                    inChar = self._ser.read(1).decode()
                    # If the character is an upper or lower case "l", start data
                    # collection on the left motor and if it is an "r", start
                    # data collection on the right motor
                    if inChar in {"l", "L"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._leftMotorGo.put(True)
                        self._ser.write("Starting left motor loop...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n")
                        self._state = S2_COL
                    elif inChar in {"r", "R"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._rightMotorGo.put(True)
                        self._ser.write("Starting right motor loop...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n")
                        self._state = S2_COL
                
            elif self._state == S2_COL:
                # While the data is collecting (in the motor task) block out the
                # UI and discard any character entry so that commands don't
                # queue up in the serial buffer
                if self._ser.any(): self._ser.read(1)
                
                # When both go flags are clear, the data collection must have
                # ended and it is time to print the collected data.
                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write("Printing data...\r\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Time, Position\r\n")
                    self._state = S3_DIS
            
            elif self._state == S3_DIS:
                # While data remains in the buffer, print that data in a command
                # separated format. Otherwise, the data collection is finished.
                if self._dataValues.any():
                    self._ser.write(f"{self._timeValues.get()},{self._dataValues.get()},\r\n")
                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Waiting for go command: 'l' for left, 'r' for right\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
            
            yield self._state