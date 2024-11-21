import random
import asyncio
from ..urtps.node import Node

class RandomNumberGenerator(Node):
    """
    A class that generates random numbers at regular intervals.

    Args:
        delta_time (int, optional): The time interval between number generations in seconds. Defaults to 1.

    Attributes:
        random_number (int): The generated random number.
        delta_time (int): The time interval between number generations in seconds.

    Example:
        generator = RandomNumberGenerator(delta_time=2)
        await generator.tick()  # Wait for the specified time interval
        print(generator.random_number)  # Print the generated random number
    """

    def __init__(self, delta_time=1):
        super().__init__('RandomNumberGenerator')
        self.random_number = 0
        self.delta_time = delta_time

    async def tick(self):
        """
        Generate a random number and set it as the message.

        This method generates a random number between 0 and 100 using the `random.randint()` function.
        The generated number is then set as the message using the `set_message()` method.

        Example:
            generator = RandomNumberGenerator()
            await generator.tick()  # Generate a random number
            print(generator.random_number)  # Print the generated random number
        """
        self.set_message(None)
        await asyncio.sleep(self.delta_time)
        self.random_number = random.randint(0, 100)
        self.set_message(self.random_number)