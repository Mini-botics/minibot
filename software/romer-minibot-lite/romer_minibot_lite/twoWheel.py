from machine import Pin, PWM

        
class TwoWheel():
    """
    A class representing a two-wheel motor controller.

    Args:
        motor1_pins (tuple): A tuple containing the GPIO pins for motor 1 in the format (pin1, pin2).
        motor2_pins (tuple): A tuple containing the GPIO pins for motor 2 in the format (pin1, pin2).
        freq (int): The frequency of the PWM signal in Hz.
        scale (float): The scale factor for the motor speed. The range of valid values is (-scale, scale).

    Example:
        To create a TwoWheel object with motor 1 connected to GPIO pins 6 and 7, motor 2 connected to GPIO pins 19 and 20,
        a frequency of 1000 Hz, and a scale factor of 1.0, you can do the following:

        >>> two_wheel = TwoWheel(motor1_pins=(6, 7), motor2_pins=(19, 20), freq=1000, scale=1.0)
    """

    def __init__(self, motor1_pins=(6, 7), motor2_pins=(20, 19), freq=1000, scale=1.0):
        # Define motor control pins
        self.motor1_pin2 = PWM(Pin(motor1_pins[0], Pin.OUT))
        self.motor1_pin1 = PWM(Pin(motor1_pins[1], Pin.OUT))
        self.motor2_pin2 = PWM(Pin(motor2_pins[0], Pin.OUT))
        self.motor2_pin1 = PWM(Pin(motor2_pins[1], Pin.OUT))
        # Set the frequency of the PWM signal
        self.motor1_pin1.freq(freq)
        self.motor1_pin2.freq(freq)
        self.motor2_pin1.freq(freq)
        self.motor2_pin2.freq(freq)
        self.scale = scale

    def motor1_write(self, duty_cycle, direction):
        """
        Set the duty cycle and direction of motor 1.

        Args:
            duty_cycle (int): The duty cycle of the PWM signal for motor 1. Valid range is 0 to 65535.
            direction (bool): The direction of rotation for motor 1. True for forward, False for backward.

        Example:
            To set the duty cycle of motor 1 to 5000 and rotate it forward, you can do the following:

            >>> two_wheel.motor1_write(5000, True)
        """
        if direction:
            self.motor1_pin1.duty_u16(duty_cycle)
            self.motor1_pin2.duty_u16(0)
        else:
            self.motor1_pin1.duty_u16(0)
            self.motor1_pin2.duty_u16(duty_cycle)

    def motor2_write(self, duty_cycle, direction):
        """
        Set the duty cycle and direction of motor 2.

        Args:
            duty_cycle (int): The duty cycle of the PWM signal for motor 2. Valid range is 0 to 65535.
            direction (bool): The direction of rotation for motor 2. True for forward, False for backward.

        Example:
            To set the duty cycle of motor 2 to 8000 and rotate it backward, you can do the following:

            >>> two_wheel.motor2_write(8000, False)
        """
        if direction:
            self.motor2_pin1.duty_u16(duty_cycle)
            self.motor2_pin2.duty_u16(0)
        else:
            self.motor2_pin1.duty_u16(0)
            self.motor2_pin2.duty_u16(duty_cycle)
