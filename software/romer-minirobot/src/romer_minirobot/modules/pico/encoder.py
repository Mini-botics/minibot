from machine import Pin
import time

class Encoder:
    '''
    Represents a quadrature encoder module connected to two pins on the Pico.

    Args:
        A (int): Pin number for A channel
        B (int): Pin number for B channel
        pole_pairs (int): Number of pole pairs of the encoder
        debounce_time (int): Debounce time in milliseconds for the encoder

    Attributes:
        A (Pin): Pin object for A channel
        B (Pin): Pin object for B channel
        A_state (int): State of A channel
        B_state (int): State of B channel
        ppr (int): Number of pulses per revolution of the encoder
        debounce_time (int): Debounce time in milliseconds for the encoder
        position (int): Current position of the encoder in pulses
        direction (int): Current direction of the encoder (1: Forward, 0: Backward)
        speed (float): Current speed of the encoder in revolutions per second (RPS)
        state (int): Current state of the encoder
        last_state (int): Last state of the encoder
        dir_forward (tuple): Lookup table for forward direction
        dir_backward (tuple): Lookup table for backward direction
        time_diff (int): Time difference between two consecutive interrupts
        last_interrupt_time_a (int): Time of last interrupt on A channel
        last_interrupt_time_b (int): Time of last interrupt on B channel

    Methods:
        A_callback: Callback function for handling interrupt on pin A
        B_callback: Callback function for handling interrupt on pin B
        update: Update method to determine the direction and position of the encoder
        get_direction: Get the current direction of the encoder
        get_position: Get the current position of the encoder in pulses
        get_speed: Get the current speed of the encoder in revolutions per second (RPS)
        get_time_diff: Get the time difference between two consecutive interrupts in milliseconds
        set_position: Set the current position of the encoder
        reset_position: Reset the position of the encoder to zero

    Example:
        enc = Encoder(A=4, B=5)
        while True:
            pass
    '''

    def __init__(self, A, B, pole_pairs=13, debounce_time=5):
        self.A = Pin(A, Pin.IN, Pin.PULL_UP)
        self.B = Pin(B, Pin.IN, Pin.PULL_UP)
        self.A_state = self.A.value()
        self.B_state = self.B.value()

        self.ppr = pole_pairs * 4 # pulses per revolution
        self.debounce_time = debounce_time
        self.position = 0 # Pulses
        self.direction = 1 # 1: Forward, 0: Backward
        self.speed = 0 # RPS

        self.state = 0
        self.last_state = 0
        self.dir_forward = (2, 0, 3, 1) # 0 -> 2 -> 3 -> 1 -> 0 lookup table for forward direction
        self.dir_backward = (1, 3, 0, 2) # 1 -> 3 -> 2 -> 0 -> 1 lookup table for backward direction

        self.time_diff = 0

        self.last_interrupt_time_a = 0
        self.last_interrupt_time_b = 0

        # Set up interrupts for both pins on any change (rising or falling edge)
        self.A.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.A_callback)
        self.B.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.B_callback)

    # Callback function to handle A interrupt when state changes with debounce
    def A_callback(self, pin):
        '''
        Callback function for handling interrupt on pin A.
        
        This function is called when an interrupt is triggered on pin A. It calculates the time difference 
        between the current interrupt and the last interrupt, and updates the state of pin A. The `update` 
        method is then called to perform any necessary actions based on the updated state.
        '''
        current_time = time.ticks_ms()
        self.time_diff = time.ticks_diff(current_time, self.last_interrupt_time_a)
        if self.time_diff > self.debounce_time:
            self.last_interrupt_time_a = current_time
            self.A_state = self.A.value()
            self.update()

    # Callback function to handle B interrupt when state changes with debounce
    def B_callback(self, pin):
        '''
        Callback function for handling interrupt on pin B.

        This function is called when an interrupt is triggered on pin B. It calculates the time difference
        between the current interrupt and the last interrupt, and updates the state of pin B. The `update`
        method is then called to perform any necessary actions based on the updated state.
        '''
        current_time = time.ticks_ms()
        self.time_diff = time.ticks_diff(current_time, self.last_interrupt_time_b)
        if self.time_diff > self.debounce_time:
            self.last_interrupt_time_b = current_time
            self.B_state = self.B.value()
            self.update()

    def update(self):
        '''
        Update method to determine the direction and position of the encoder based on the current state of the pins.
        '''

        self.last_state = self.state
        self.state = (self.A_state << 1) | self.B_state
        
        if self.dir_forward[self.last_state] == self.state:
            self.direction = 1
        elif self.dir_backward[self.last_state] == self.state:
            self.direction = 0
        else:
            print("Invalid state")
            return None
        
        if self.direction:
            self.position += 1
            self.speed = 1000 / self.ppr / self.time_diff
        else:
            self.position -= 1
            self.speed = -1000 / self.ppr / self.time_diff
    
    def get_direction(self):
        '''
        Get the current direction of the encoder.
        '''
        return self.direction
    
    def get_position(self):
        '''
        Get the current position of the encoder in pulses.
        '''      
        return self.position

    def get_speed(self):
        '''
        Get the current speed of the encoder in revolutions per second (RPS).
        '''
        speed = self.speed
        self.speed = 0
        return speed # RPS
    
    def get_time_diff(self):
        '''
        Get the time difference between two consecutive interrupts in milliseconds.
        '''
        return self.time_diff / 1000

    def set_position(self, position):
        '''
        Set the current position of the encoder.

        Args:
            position (int): New position of the encoder in pulses
        '''
        self.position = position

    def reset_position(self):
        '''
        Reset the position of the encoder to zero.
        '''
        self.position = 0