from machine import Pin, PWM

from ...urtps.node import Node
        
class Holonomic(Node):
    """
    Holonomic class represents a holonomic robot controller.

    Args:
        motor1_pins (tuple): A tuple containing the GPIO pins for motor 1 in the format (pin1, pin2).
        motor2_pins (tuple): A tuple containing the GPIO pins for motor 2 in the format (pin1, pin2).
        motor3_pins (tuple): A tuple containing the GPIO pins for motor 3 in the format (pin1, pin2).
        motor4_pins (tuple): A tuple containing the GPIO pins for motor 4 in the format (pin1, pin2).
        freq (int): The frequency of the PWM signal.
        scale (float): The scale factor for the motor speed.

    Example:
        # Create a Holonomic object with default pin configuration and scale factor
        holonomic = Holonomic()

        # Create a Holonomic object with custom pin configuration and scale factor
        motor1_pins = (6, 7)
        motor2_pins = (19, 20)
        motor3_pins = (9, 8)
        motor4_pins = (18, 17)
        freq = 1000
        scale = 0.5
        holonomic = Holonomic(motor1_pins, motor2_pins, motor3_pins, motor4_pins, freq, scale)
    """

    def __init__(self, name = 'holonomic', motor1_pins=(7, 6), motor2_pins=(20, 19),
                 motor3_pins=(8, 9), motor4_pins=(17, 18), freq=1000, scale=1.0):
        super().__init__(name, 'subscribing')
        # Define motor control pins
        self.motor1_pin2 = PWM(Pin(motor1_pins[0], Pin.OUT))
        self.motor1_pin1 = PWM(Pin(motor1_pins[1], Pin.OUT))
        self.motor2_pin2 = PWM(Pin(motor2_pins[0], Pin.OUT))
        self.motor2_pin1 = PWM(Pin(motor2_pins[1], Pin.OUT))

        self.motor3_pin2 = PWM(Pin(motor3_pins[0], Pin.OUT))
        self.motor3_pin1 = PWM(Pin(motor3_pins[1], Pin.OUT))
        self.motor4_pin2 = PWM(Pin(motor4_pins[0], Pin.OUT))
        self.motor4_pin1 = PWM(Pin(motor4_pins[1], Pin.OUT))
        # Set the frequency of the PWM signal
        self.motor1_pin1.freq(freq)
        self.motor1_pin2.freq(freq)
        self.motor2_pin1.freq(freq)
        self.motor2_pin2.freq(freq)
        self.motor3_pin1.freq(freq)
        self.motor3_pin2.freq(freq)
        self.motor4_pin1.freq(freq)
        self.motor4_pin2.freq(freq)
        self.scale = scale

    def motor_write(self, duty_cycle, direction, motor1, motor2):
        """
        Write the duty cycle and direction to the specified motor.

        Args:
            duty_cycle (int): The duty cycle value.
            direction (bool): The direction of rotation (True for forward, False for backward).
            motor1 (PWM): The PWM object representing the first motor pin.
            motor2 (PWM): The PWM object representing the second motor pin.
        """
        if direction:
            motor1.duty_u16(duty_cycle)
            motor2.duty_u16(0)
        else:
            motor1.duty_u16(0)
            motor2.duty_u16(duty_cycle)

    async def tick(self):
        if not self.get_message():
            return
        x_linear, y_linear, z_angular = self.get_message().split(",")
        x_linear = float(x_linear)
        y_linear = float(y_linear)
        z_angular = float(z_angular)

        x_linear = max(-self.scale, min(self.scale, x_linear))
        y_linear = max(-self.scale, min(self.scale, y_linear))
        z_angular = max(-1, min(1, z_angular))

        duty_cycle_x = int((x_linear / self.scale * 65535))
        duty_cycle_y = int((y_linear / self.scale * 65535))
        duty_cycle_r = int((z_angular / self.scale * 65535))

        duty_cycle_1 = duty_cycle_x - duty_cycle_y + duty_cycle_r
        duty_cycle_2 = duty_cycle_x + duty_cycle_y - duty_cycle_r
        duty_cycle_3 = duty_cycle_x + duty_cycle_y + duty_cycle_r
        duty_cycle_4 = duty_cycle_x - duty_cycle_y - duty_cycle_r

        self.motor_write(abs(duty_cycle_1), duty_cycle_1 > 0, self.motor1_pin1, self.motor1_pin2)
        self.motor_write(abs(duty_cycle_2), duty_cycle_2 > 0, self.motor2_pin1, self.motor2_pin2)
        self.motor_write(abs(duty_cycle_3), duty_cycle_3 > 0, self.motor3_pin1, self.motor3_pin2)
        self.motor_write(abs(duty_cycle_4), duty_cycle_4 > 0, self.motor4_pin1, self.motor4_pin2)

        self.set_message(None)
