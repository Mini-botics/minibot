from ..urtps.node import Node
        
class Bool(Node):
    """
    Represents a boolean node that subscribes to a message and stores its value as a boolean.
    
    Args:
        name (str): The name of the boolean node.
    
    Attributes:
        value (bool): The current value of the boolean node.
    """
    
    def __init__(self, name):
        super().__init__(name, 'subscribing')
        self.value = False
    
    def get(self):
        """
        Returns the current value of the boolean node.
        
        Returns:
            bool: The current value of the boolean node.
        """
        return self.value
    
    async def tick(self):
        """
        Updates the value of the boolean node based on the received message.
        """
        self.value = self.get_message() == 'True'
    