from ..urtps.node import Node
        
class Echo(Node):
    """
    A class representing an Echo node.

    This node receives messages and prints them to the console.

    Attributes:
        name (str): The name of the Echo node.

    Methods:
        __init__(self, name): Initializes the Echo node with a name.
        tick(self): Performs the main functionality of the Echo node.

    Example:
        echo_node = Echo("echo_node")
        await echo_node.tick()
    """

    def __init__(self, name):
        super().__init__(name)
    
    async def tick(self):
        """
        Receives messages and prints them to the console.

        If a message is available, it prints the message to the console.
        After printing, it sets the message to None.

        Example:
            echo_node = Echo("echo_node")
            await echo_node.tick()
        """
        if self.get_message():
            print(f'Message received. {self.get_message()}')
        self.set_message(None)