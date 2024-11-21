from ...urtps import Node

class Button(Node):
    """
    Represents a button node in the MiniRobot software.
    Args:
        name (str): The name of the button node. Defaults to 'button'.

    Attributes:
        button (bool): The current state of the button.

    Methods:
        set_message: Sets the button state based on the received message.
        tick: Asynchronously updates the button state.
        get: Returns the current state of the button.

    Example:
        button = Button('my_button')
        button.set_message('True')
        print(button.get())  # Output: True
    """

    def __init__(self, name='button'):
        """
        Initializes a Button object.

        Args:
            name (str, optional): The name of the button. Defaults to 'button'.
        """
        super().__init__(name, 'subscribing')
        self.button = None

    def set_message(self, message):
        """
        Sets the button state based on the given message.

        Args:
            message (str): The message to set the button state. Should be either 'True' or 'False'.

        Returns:
            None
        """
        self.button = message == 'True'
    
    async def tick(self):
        return self.button
    
    def get(self):
        """
        Returns the current state of the button.

        Returns:
            bool: The current state of the button. True if the button is pressed, False otherwise.
        """
        return self.button
    
    