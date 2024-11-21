import asyncio
from ..urtps.node import BlockingNode
        
class Teller(BlockingNode):
    """
    A class representing a Teller node.

    This class inherits from the BlockingNode class and provides functionality for a Teller node in a robotic system.

    Attributes:
        static_id (int): A static variable to keep track of the number of Teller instances created.

    Methods:
        __init__(self, msg, delta_time, name=None): Initializes a Teller instance.
        tick(self): Performs a tick operation for the Teller node.

    Example:
        teller = Teller("Hello, world!", 0.5)
        await teller.tick()
    """

    static_id = 0

    def __init__(self, msg, delta_time, name=None) -> None:
        """
        Initializes a Teller instance.

        Args:
            msg (str): The message to be stored in the Teller instance.
            delta_time (float): The time interval between ticks.
            name (str, optional): The name of the Teller instance. If not provided, a default name will be assigned.

        Returns:
            None
        """
        if not name:
            name = f"Teller-{Teller.static_id}"
        super().__init__(name, delta_time)
        Teller.static_id += 1
        self.set_message(msg)

    async def tick(self):
        """
        Performs a tick operation for the Teller node.

        Returns:
            The stored message in the Teller instance.
        """
        return self.get_message()