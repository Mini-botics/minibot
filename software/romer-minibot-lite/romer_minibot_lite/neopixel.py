import neopixel
from machine import Pin

class NeoPixel():
    """
    Represents a NeoPixel module that controls a strip of individually addressable RGB LEDs.
    
    Args:
        pin_number (int): The pin number to which the NeoPixel strip is connected.
        num_pixels (int): The number of pixels in the NeoPixel strip.
        
    """
    
    def __init__(self, pin_number, num_pixels, name='neopixel'):
        self.pixels = neopixel.NeoPixel(Pin(pin_number), num_pixels)
        
    def fillwith(self, pixel_colors, brightness=1.0):
        """
        Sets specific NeoPixel LEDs to the specified RGB colors with brightness control.
        
        Args:
            pixel_colors (list): A list of tuples, each in the format (pixel_index, (R, G, B)),
                where `pixel_index` is the index of the pixel to update, and `(R, G, B)` is
                a tuple representing the RGB color value.
                Each RGB value should be a float or int between 0 and 255.
            brightness (float): A value between 0 and 1 that controls the brightness of the LEDs.
                0 is off, and 1 is full brightness.
        """
        for pixel_index, rgb in pixel_colors:
            color = (
                int(float(rgb[0]) * brightness),
                int(float(rgb[1]) * brightness),
                int(float(rgb[2]) * brightness)
            )
            # Ensure RGB values stay within the valid range (0-255)
            color = tuple(min(max(0, value), 255) for value in color)
            self.pixels[pixel_index] = color
        self.pixels.write()
