from machine import Pin, I2C
import ustruct
from utime import ticks_ms


class Accelerometer():
    '''
    Repreesents an accelerometer module for ADXL345 accelerometer sensor

    Args:
        sda (int): SDA pin number
        scl (int): SCL pin number
        delta_time (int): The time interval (in milliseconds) between acceleration measurements
        name (string): Name of the accelerometer module

    Attributes:
        ADXL345_ADDRESS (int): ADXL345 I2C address
        ADXL345_POWER_CTL (int): ADXL345 power control register
        ADXL345_DATA_FORMAT (int): ADXL345 data format register
        ADXL345_DATAX0 (int): ADXL345 X-axis data register

    Usage:
        accelerometer = Accelerometer()
        x, y, z = accelerometer.read_accel_data()
    '''

    def __init__(self, sda, scl, delta_time, name='accelerometer') -> None:

        # Constants
        self.ADXL345_ADDRESS = 0x53
        self.ADXL345_POWER_CTL = 0x2D
        self.ADXL345_DATA_FORMAT = 0x31
        self.ADXL345_DATAX0 = 0x32

        self.x = 0
        self.y = 0
        self.z = 0
        self.delta_time = delta_time
        self.last_time = ticks_ms()

        # Initialize I2C
        self.i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=40000)
        self.init_adxl345()

    # Initialize ADXL345
    def init_adxl345(self):
        self.i2c.writeto_mem(self.ADXL345_ADDRESS, self.ADXL345_POWER_CTL, bytearray([0x08]))  # Set bit 3 to 1 to enable measurement mode
        self.i2c.writeto_mem(self.ADXL345_ADDRESS, self.ADXL345_DATA_FORMAT, bytearray([0x0B]))  # Set data format to full resolution, +/- 16g
    
    # Read acceleration data
    def read_accel_data(self):
        data = self.i2c.readfrom_mem(self.ADXL345_ADDRESS, self.ADXL345_DATAX0, 6)
        x, y, z = ustruct.unpack('<3h', data)
        return x, y, z