from machine import Pin, I2C
import time
from minibot_220 import Accelerometer  # Assuming Accelerometer is saved as accelerometer.py
from minibot_220 import NeoPixel            # Assuming NeoPixel is saved as neopixel.py
from minibot_220 import Button         # Assuming Button is saved as button.py

# Initialize the Accelerometer
sda_pin = 12  # Replace with your SDA pin
scl_pin = 13  # Replace with your SCL pin
delta_time = 100  # Interval in milliseconds between readings

accelerometer = Accelerometer(sda=sda_pin, scl=scl_pin, delta_time=delta_time)

# Initialize the NeoPixel strip
neopixel_pin = 0  # Replace with your NeoPixel data pin
num_pixels = 20    # Number of LEDs in your NeoPixel strip
neopixel_strip = NeoPixel(pin_number=neopixel_pin, num_pixels=num_pixels)

# Initialize the Button
button_pin = 3  # Replace with your button pin
button = Button(pin_number=button_pin, mode='pull_up', invert=False, debounce_ms=60)

# Function to map accelerometer values to RGB
def map_to_rgb(x, y, z):
    # Assuming the range of x, y, z is -256 to 256
    r = int((x + 256) * (255 / 512))  # Map -256 to 256 into 0-255 for red
    g = int((y + 256) * (255 / 512))  # Map -256 to 256 into 0-255 for green
    b = int((z + 256) * (255 / 512))  # Map -256 to 256 into 0-255 for blue
    return r, g, b

# Main loop to control NeoPixels based on accelerometer data
while True:
    if not button.state:  # Button is pressed when state is True (invert if needed)
        # Set all NeoPixels to red when the button is pressed
        pixel_colors = [(i, (255, 0, 0)) for i in range(num_pixels)]  # Red color

    else:
        # Read accelerometer values
        x, y, z = accelerometer.read_accel_data()
        
        # Map accelerometer values to RGB
        r, g, b = map_to_rgb(x, y, z)

        # Create a list of RGB values to fill all NeoPixels
        pixel_colors = [(i, (r, g, b)) for i in range(num_pixels)]  # Fill all pixels with the same color

    # Update NeoPixels with the calculated RGB values
    neopixel_strip.fillwith(pixel_colors, brightness=0.5)

    # Print the RGB values and accelerometer data for debugging
    print(f"Accelerometer data -> X: {x}, Y: {y}, Z: {z}")
    print(f"RGB values -> R: {r}, G: {g}, B: {b}")

    # Wait for the next reading
    time.sleep(delta_time / 1000)
