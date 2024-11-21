from machine import Pin, PWM
from utime import ticks_ms
import rp2
from ...urtps.node import Node
from .encoder import Encoder
        
class TwoWheelPID(Node):
    """
    A class representing a two-wheel PID controller for motor control.

    Args:
        dt (float): The time interval between control updates in seconds. Default is 0.3 seconds.

    Attributes:
        motor1_pin2 (PWM): The PWM object representing the control pin for motor 1.
        motor1_pin1 (PWM): The PWM object representing the control pin for motor 1.
        motor2_pin2 (PWM): The PWM object representing the control pin for motor 2.
        motor2_pin1 (PWM): The PWM object representing the control pin for motor 2.
        motor1_hall_1 (Pin): The Pin object representing the hall sensor input for motor 1.
        motor1_hall_2 (Pin): The Pin object representing the hall sensor input for motor 1.
        motor2_hall_1 (Pin): The Pin object representing the hall sensor input for motor 2.
        motor2_hall_2 (Pin): The Pin object representing the hall sensor input for motor 2.
        pi1 (PI): The PI controller object for motor 1.
        pi2 (PI): The PI controller object for motor 2.
        last_time (int): The timestamp of the last control update.
        dt (float): The time interval between control updates in seconds.

    """

    def __init__(self, name = 'twoWheel', dt=0.3):
        super().__init__(name, 'subscribing')
        # Define motor control pins
        self.motor1_pin2 = PWM(Pin(6, Pin.OUT))
        self.motor1_pin1 = PWM(Pin(7, Pin.OUT))
        self.motor2_pin2 = PWM(Pin(20, Pin.OUT))
        self.motor2_pin1 = PWM(Pin(19, Pin.OUT))
        # Set the frequency of the PWM signal
        self.motor1_pin1.freq(100000)
        self.motor1_pin2.freq(100000)
        self.motor2_pin1.freq(100000)
        self.motor2_pin2.freq(100000)

        self.motor1_hall_1 = Pin(4, Pin.IN, Pin.PULL_UP)
        self.motor1_hall_2 = Pin(5, Pin.IN, Pin.PULL_UP)
        self.motor2_hall_1 = Pin(21, Pin.IN, Pin.PULL_UP)
        self.motor2_hall_2 = Pin(22, Pin.IN, Pin.PULL_UP)

        self.pid1 = PID(A=4, B=5, prop=85000, integ=500, deriv=0)
        self.pid2 = PID(A=22, B=21, prop=80000, integ=600, deriv=200)

        self.last_time = 0
        self.dt = dt
        
        self.RPM = 200
        self.RPS = self.RPM / 60
        
    def motor1_write(self, duty_cycle, direction):
        """
        Set the duty cycle and direction of motor 1.

        Args:
            duty_cycle (int): The duty cycle value between 0 and 65535.
            direction (bool): The direction of rotation. True for forward, False for backward.

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
            duty_cycle (int): The duty cycle value between 0 and 65535.
            direction (bool): The direction of rotation. True for forward, False for backward.

        """
        if direction:
            self.motor2_pin1.duty_u16(duty_cycle)
            self.motor2_pin2.duty_u16(0)
        else:
            self.motor2_pin1.duty_u16(0)
            self.motor2_pin2.duty_u16(duty_cycle)

    async def tick(self):
        """
        Perform a control update.

        This method is called periodically to update the motor control based on the received message.

        """
        cur_time = ticks_ms()
        if self.get_message():

            x_linear, z_angular = self.get_message().split(",")  # what will be the unit for this???
            
            x_linear = float(x_linear) * self.RPS
            z_angular = float(z_angular) * self.RPS

            self.pid1.set_ref_speed(x_linear + z_angular)
            self.pid2.set_ref_speed(x_linear - z_angular)

            #print(self.pid1.ref_speed, self.pid2.ref_speed)
        # print(f'motor1:{self.pid1.speed},motor2:{self.pid2.speed}')
        
        if cur_time - self.last_time > self.dt:
            self.pid1.update()
            self.pid2.update()
            motor1_speed = self.pid1.pid()
            motor2_speed = self.pid2.pid()
            #print(f'motor1ref:{self.pid1.ref_speed},motor1speed:{self.pid1.speed}')
            #print(f'motor2^ref:{self.pid2.ref_speed},motor2speed:{self.pid2.speed}')
            self.motor1_write(int(abs(motor1_speed)), motor1_speed > 0)
            self.motor2_write(int(abs(motor2_speed)), motor2_speed > 0)
        self.set_message(None)
        
    
class PID:
    """
    Proportional-Integral (PI) controller class for controlling the speed of a two-wheel robot.
    
    Args:
        hall1_pin (Pin): The pin connected to the first hall sensor.
        hall2_pin (Pin): The pin connected to the second hall sensor.
        prop (int, optional): The proportional gain of the controller. Defaults to 0.
        integ (int, optional): The integral gain of the controller. Defaults to 0.
        deriv (int, optional): The derivative gain of the controller. Defaults to 0.

    
    Attributes:
        position_old (int): The previous position of the robot.
        position (int): The current position of the robot.
        time_old (int): The previous time at which the position was updated.
        speed (float): The current speed of the robot.
        ref_speed (float): The reference speed set for the robot.
        prop (int): The proportional gain of the controller.
        integ (int): The integral gain of the controller.
        integ_sum (float): The sum of the integral errors.
        error (float): The current error between the reference speed and the actual speed.
        prev_error (float): The previous error between the reference speed and the actual speed.
    """
    
    def __init__(self,A, B, prop=10000, integ=0, deriv=0) -> None:
        self.dt = 0

        self.ref_speed = 0
        self.speed = 0
        
        self.prop = prop
        self.integ = integ
        self.deriv = deriv

        
        self.integ_sum = 0
        self.deriv_calc = 0
        
        self.error = 0
        self.prev_error = 0
        
        # Initialize the Encoder
        self.encoder = Encoder(A, B)
    
    def set_ref_speed(self, ref_speed):
        """
        Set the reference speed for the robot.
        
        Args:
            ref_speed (float): The reference speed to be set.
        """

        self.ref_speed = ref_speed
        self.speed = 0
        self.integ_sum = 0
        self.deriv_calc = 0
        self.error = 0            
        self.prev_error = 0
        
    def update(self):
        """
        Update the speed of the robot and time delta.
        """
        self.speed = self.encoder.get_speed() # Speed in RPS
        self.dt = self.encoder.get_time_diff() # Time delta in seconds
    
    def pid(self):
        """
        Calculate the control signal for the PID controller.
        
        Returns:
            float: The control signal calculated by the PID controller.
        """        
        self.prev_error = self.error
        self.error = self.ref_speed - self.speed
        #print(self.speed)
        
        if self.error * self.prev_error < 0:
            self.integ_sum = 0
        else:
            # Update the integral sum with the current error scaled by the time delta
            self.integ_sum += self.error * self.dt
        
        if self.dt:
            self.deriv_calc = (self.error - self.prev_error) / self.dt

        return self.prop * self.error + self.integ * self.integ_sum + self.deriv * self.deriv_calc