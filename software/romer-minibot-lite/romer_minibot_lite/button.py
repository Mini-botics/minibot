from machine import Pin
from utime import ticks_ms


class Button():
    """
    Represents a button component that can be used to detect button presses.

    Args:
        pin_number (int): The pin number to which the button is connected.
        mode (str): The mode of the button. Can be 'pull_up' or 'pull_down'.
        invert (bool): Whether to invert the button's logic level.

    Example:
        button = Button(5, 'pull_up', False, 0.1, 'my_button')
        # Creates a button object connected to pin 5, with pull-up mode,
        # non-inverted logic level, and a polling interval of 0.1 seconds.
        # The button is named 'my_button'.
    """

    def __init__(self, pin_number, mode, invert, debounce_ms=60) -> None:
        if mode:
            if mode == 'pull_up':
                self.pin = Pin(pin_number, Pin.IN, Pin.PULL_UP)
            elif mode == 'pull_down':
                self.pin = Pin(pin_number, Pin.IN, Pin.PULL_DOWN)
        self.invert = not invert
        self.last_time = ticks_ms()

        self.state = self.pin.value()
        self.last_debounce_time = 0
        self.debounce_ms = debounce_ms
        
        self.pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self._callback)

    def _callback(self, pin):
        """
        Callback function for the button interrupt.
        """
        current_time = ticks_ms()
        if current_time - self.last_debounce_time > self.debounce_ms:
            self.last_debounce_time = current_time
            self.last_state = self.state
            self.state = pin.value()

            return self.state == self.invert

