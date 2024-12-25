from machine import Pin, time_pulse_us
from utime import ticks_ms
from time import sleep_us

class Ultrasonic():
    """
    Represents an ultrasonic sensor module that measures distance.

    Args:
        trig_pin (int): The pin number connected to the trigger of the ultrasonic sensor.
        echo_pin (int): The pin number connected to the echo of the ultrasonic sensor.
        delta_time (int): The time interval (in milliseconds) between distance measurements.
        name (str, optional): The name of the ultrasonic module. Defaults to 'ultrasonic'.
    """

    def __init__(self, trig_pin, echo_pin, delta_time, name='ultrasonic') -> None:
        # Define pins
        self.trig_pin = Pin(trig_pin, Pin.OUT)
        self.echo_pin = Pin(echo_pin, Pin.IN)
        self.distance = 0
        self.delta_time = delta_time
        self.last_time = ticks_ms()

    def measure_distance(self):
        '''
        Measures the distance using the ultrasonic sensor.

        Returns:
            float: The distance in millimeters.
        '''

        # Ensure the trigger pin is low
        self.trig_pin.value(0)
        sleep_us(2)
        
        # Send a 10us pulse to trigger pin
        self.trig_pin.value(1)
        sleep_us(10)
        self.trig_pin.value(0)
        
        # Measure the duration of the pulse on the echo pin
        duration = time_pulse_us(self.echo_pin, 1, 100000)  # .1 second timeout

        if duration < 0:
            print("Out of range")
            return None
        
        # Calculate distance (duration in microseconds)
        self.distance = duration / 5.82

        return self.distance


