import neopixel
from machine import Pin

from ...urtps import Node


class NeoPixel(Node):
    """
    Represents a NeoPixel module that controls a strip of individually addressable RGB LEDs.
    
    Args:
        pin_number (int): The pin number to which the NeoPixel strip is connected.
        num_pixels (int): The number of pixels in the NeoPixel strip.
    """
    
    def __init__(self, pin_number, num_pixels, name = 'neopixel'):
        super().__init__(name, 'subscribing')
        self.pixels = neopixel.NeoPixel(Pin(pin_number), num_pixels)
        
    def fillwith(self, colors):
        """
        Fills the NeoPixel strip with the specified colors.
        
        Args:
            colors (list): A list of RGB color values in the format [R, G, B, R, G, B, ...].
                Each color value should be a float between 0 and 255.
        """
        for i in range(0,len(colors),3):
            color = (
                int(float(colors[i+0])),
                int(float(colors[i+1])),
                int(float(colors[i+2]))
            )
            self.pixels[i // 3] = color
        self.pixels.write()

    async def tick(self):
        if not self.get_message():
            return
        
        self.fillwith(self.get_message().split(','))
        self.set_message(None)
