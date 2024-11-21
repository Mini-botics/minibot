from ...urtps import EventPubNode

class NeoPixel(EventPubNode):
    
    """
    Represents a NeoPixel object that controls a strip of individually addressable RGB LEDs.

    Args:
        num_pixels (int): The number of pixels in the NeoPixel strip.
        brightness (float, optional): The brightness of the NeoPixel strip. Defaults to 1.0.

    Attributes:
        num_pixels (int): The number of pixels in the NeoPixel strip.
        pixels (list): A list of RGB tuples representing the color of each pixel.
        brightness (float): The brightness of the NeoPixel strip.

    """

    def __init__(self, num_pixels, brightness=1.0, name = 'neopixel'):
        super().__init__(name, 'publishing')
        self.num_pixels = num_pixels
        self.pixels = [(0, 0, 0)] * num_pixels
        self.brightness = brightness

    def _flatten(self):
        """
        Flattens the pixels list into a string representation.

        Returns:
            str: A string representation of the flattened pixels list.

        """
        return str([item for sublist in self.pixels for item in sublist])[1:-1]

    def set_brightness(self, brightness):
        """
        Sets the brightness of the NeoPixel strip.

        Args:
            brightness (float): The brightness value to set.

        Returns:
            str: The flattened pixels list as a string.

        """
        self.brightness = brightness
        return super().set_message(self._flatten())

    def __getitem__(self, index):
        """
        Gets the RGB color of a specific pixel.

        Args:
            index (int): The index of the pixel.

        Returns:
            tuple: The RGB color of the pixel.

        """
        return self.pixels[index]

    def __setitem__(self, index, rgb):
        """
        Sets the RGB color of a specific pixel.

        Args:
            index (int): The index of the pixel.
            rgb (tuple): The RGB color to set.

        """
        self.pixels[index] = (
            int(rgb[0] * self.brightness),
            int(rgb[1] * self.brightness),
            int(rgb[2] * self.brightness)
        )

    def fill_with(self, color):
        """
        Fills the entire NeoPixel strip with a specific color.

        Args:
            color (tuple): The RGB color to fill the strip with.

        """
        for i in range(self.num_pixels):
            self.__setitem__(i, color)

    def write(self):
        """
        Writes the current state of the NeoPixel strip.

        Returns:
            str: The flattened pixels list as a string.

        """
        return super().set_message(self._flatten())
        
