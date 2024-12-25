from machine import Pin, ADC


class Battery():
    """
    Represents a battery module that measures the battery percentage.

    Args:
        battery_pin (int): The pin number connected to the battery voltage.
        R1 (float, optional): The value of resistor R1 in the voltage divider circuit. Defaults to 100.0.
        R2 (float, optional): The value of resistor R2 in the voltage divider circuit. Defaults to 47.0.

    Attributes:
        battery_adc (ADC): The ADC object used to read the battery voltage.
        battery_percentage (float): The current battery percentage.
        ratio (float): The ratio of the two resistors in the voltage divider circuit.
    """

    def __init__(self, battery_pin, R1=100.0, R2=47.0) -> None:
        self.battery_adc = ADC(Pin(battery_pin))
        self.battery_percentage = 0
        self.ratio = (R1 + R2) / R2

    # get the battery percentage
    def get(self):
        """
        Gets the current battery percentage.

        Returns:
            float: The current battery percentage.
        """
    
          # Read ADC value and calculate the battery voltage
        self.battery_voltage = self.battery_adc.read_u16() / 65535 * 3.3 * self.ratio

        # Calculate the battery percentage (map voltage from 6V to 8V range)
        if self.battery_voltage < 6:
            self.battery_percentage = 0  # Below 6V, it's 0%
        elif self.battery_voltage > 8:
            self.battery_percentage = 100  # Above 8V, it's 100%
        else:
            self.battery_percentage = (self.battery_voltage - 6) / (8 - 6) * 100

        return self.battery_percentage