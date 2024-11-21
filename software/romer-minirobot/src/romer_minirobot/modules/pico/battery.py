from machine import Pin, ADC
from utime import ticks_ms

from ...urtps import Node

class Battery(Node):
    """
    Represents a battery module that measures the battery percentage.

    Args:
        battery_pin (int): The pin number connected to the battery voltage.
        delta_time (int): The time interval (in milliseconds) between battery percentage measurements.
        name (str, optional): The name of the battery module. Defaults to 'battery'.
        R1 (float, optional): The value of resistor R1 in the voltage divider circuit. Defaults to 100.0.
        R2 (float, optional): The value of resistor R2 in the voltage divider circuit. Defaults to 47.0.
    """

    def __init__(self, battery_pin, delta_time, name='battery', R1=100.0, R2=47.0) -> None:
        super().__init__(name, 'publishing')
        self.battery_adc = ADC(Pin(battery_pin))
        self.battery_percentage = 0
        self.ratio = (R1 + R2) / R2
        self.delta_time = delta_time
        self.last_time = ticks_ms()

    async def tick(self):
        """
        Measures the battery percentage at regular intervals.

        Returns:
            float: The current battery percentage.
        """
        self.set_message(None)
        time = ticks_ms()
        if time - self.last_time < self.delta_time:
            return
        self.last_time = time
        self.battery_percentage = self.battery_adc.read_u16() / 65535 * 3.3 * self.ratio
        self.set_message(self.battery_percentage)