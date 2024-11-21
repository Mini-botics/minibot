from machine import Pin
from utime import ticks_ms

from ...urtps import EventPubNode

class Button(EventPubNode):
    """
    Represents a button component that can be used to detect button presses.

    Args:
        pin_number (int): The pin number to which the button is connected.
        mode (str): The mode of the button. Can be 'pull_up' or 'pull_down'.
        invert (bool): Whether to invert the button's logic level.
        poll_sec (float): The time interval (in seconds) between button state checks.
        name (str, optional): The name of the button. Defaults to 'button'.

    Example:
        button = Button(5, 'pull_up', False, 0.1, 'my_button')
        # Creates a button object connected to pin 5, with pull-up mode,
        # non-inverted logic level, and a polling interval of 0.1 seconds.
        # The button is named 'my_button'.
    """

    def __init__(self, pin_number, mode, invert, poll_ms = 500, repeat = 5, name='button') -> None:
        super().__init__(name, 'publishing')
        self.pin = Pin(pin_number, Pin.IN)
        if mode:
            if mode == 'pull_up':
                self.pin = Pin(pin_number, Pin.IN, Pin.PULL_UP)
            elif mode == 'pull_down':
                self.pin = Pin(pin_number, Pin.IN, Pin.PULL_DOWN)
        self.invert = not invert
        self.poll_ms = poll_ms
        self.repeat = repeat
        self.last_value = self.pin.value()
        self.changed_state = False
        self.last_time = ticks_ms()
        self.repeats = 0
        
    async def tick(self):
        """
        Checks the state of the button and publishes the button state if it has changed.
        """
        pin_val = self.pin.value()
        if pin_val != self.last_value:
            self.changed_state = True
            self.last_time = ticks_ms()
            self.last_value = not self.last_value
    

        if not self.changed_state:
            return
        
        if ticks_ms() - self.last_time < self.poll_ms:
            if self.repeats < self.repeat:
                #self.set_message(None)
                self.set_message(str(self.last_value == self.invert))
                self.last_time = ticks_ms()
                self.repeats += 1
            else:
                self.repeats = 0
                self.changed_state = False