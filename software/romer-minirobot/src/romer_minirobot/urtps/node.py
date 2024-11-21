from ..utils import is_running_on_pico

class BaseNode:
    """
    Base class for nodes in the MiniRobot system.

    Args:
        name (str): The name of the node.
        type (str): The type of the node.

    Attributes:
        type (str): The type of the node.
        name (str): The name of the node.
        message: The message associated with the node.

    Methods:
        set_message(message): Sets the message for the node.
        get_message(): Returns the message of the node.
        encode(): Encodes the node's name and message.
        decode(data): Decodes the data and returns the name and message as a list.

    Example:
        node = BaseNode("Node1", "Type1")
        node.set_message("Hello, world!")
        encoded_data = node.encode()
        print(encoded_data)  # Output: "Node1|Hello, world!"
        decoded_data = BaseNode.decode(encoded_data.encode())
        print(decoded_data)  # Output: ['Node1', 'Hello, world!']
    """

    def __init__(self, name, type) -> None:
        """
        Initializes a Node object.

        Args:
            name (str): The name of the node.
            type (str): The type of the node.

        Attributes:
            type (str): The type of the node.
            name (str): The name of the node.
            message (None): The message associated with the node (initially set to None).
        """
        self.type = type
        self.name = name
        self.message = None
    
    def set_message(self, message):
        """
        Sets the message attribute of the Node object.

        Args:
            message (str): The message to be set.

        Returns:
            None
        """
        self.message = message
    
    def get_message(self):
        """
        Returns the message stored in the node.
        
        Returns:
            str: The message stored in the node.
        """
        return self.message
    
    def encode(self):
        """
        Encodes the name and message of the node into a byte string.

        Returns:
            bytes: The encoded byte string.
        """
        return f'{self.name}|{self.message}'.encode()
    
    @staticmethod
    def decode(data):
        """
        Decode the given data by converting it from bytes to string and splitting it by '|'.

        Args:
            data (bytes): The data to be decoded.

        Returns:
            list: A list of strings obtained by splitting the decoded data.

        """
        return data.decode().split('|')
    
class Node(BaseNode):
    """
    Represents a node in the MiniRobot system.

    Args:
        name (str): The name of the node.
        type (str): The type of the node.

    Attributes:
        name (str): The name of the node.
        type (str): The type of the node.
        
    Example:
        To create a custom node that prints "Hello, World!" during each tick:

        ```python
        class CustomNode(Node):
            async def tick(self):
                print("Hello, World!")
        ```

    """
    def __init__(self, name, type) -> None:
        super().__init__(name, type)
    
    async def tick(self):
        """
        Perform the tick operation for the node.

        This method should be implemented by subclasses to define the behavior of the node during each tick.

        Raises:
            NotImplementedError: If the method is not implemented by a subclass.

        """
        raise NotImplementedError

class BlockingNode(BaseNode):
    """
    A class representing a blocking node in a MiniRobot application.

    This class extends the `BaseNode` class and provides additional functionality
    for blocking message retrieval and setting based on a specified time interval.

    Args:
        name (str): The name of the node.
        type (str): The type of the node.
        delta_time (float): The time interval (in seconds) between message retrievals or settings.

    Attributes:
        gettime (function): A function for retrieving the current time.
        delta_time (float): The time interval (in milliseconds) between message retrievals or settings.
        last_time (float): The timestamp of the last message retrieval or setting.

    Example:
        # Create a blocking node with a time interval of 0.5 seconds
        node = BlockingNode("my_node", "blocking", 0.5)

        # Retrieve a message from the node
        message = node.get_message()

        # Set a message for the node
        node.set_message("Hello, world!")
    """

    def __init__(self, name, type, delta_time) -> None:
        """
        Initializes a Node object.

        Args:
            name (str): The name of the node.
            type (str): The type of the node.
            delta_time (float): The time interval for updating the node, in seconds.

        Returns:
            None

        Raises:
            None

        """
        self.gettime = None
        if is_running_on_pico():
            from utime import ticks_ms
            self.gettime = ticks_ms
            self.delta_time = delta_time * 1000
        else:
            from time import time
            self.gettime = time
            self.delta_time = delta_time
            
        super().__init__(name, type)
        self.last_time = 0
        
    def get_message(self):
        """
        Retrieves a message from the node.

        This method checks if enough time has passed since the last message retrieval.
        If the elapsed time is greater than the specified delta time, it retrieves a new message
        by calling the parent class's `get_message` method. Otherwise, it returns None.

        Returns:
            The retrieved message if enough time has passed, otherwise None.
        """
        cur_time = self.gettime()
        if cur_time - self.last_time > self.delta_time:
            self.last_time = cur_time
            return super().get_message()
        return None
    
    def set_message(self, message):
        """
        Sets the message for the node.

        This method sets the message for the node and ensures that the message is only set
        if the specified time interval has passed since the last message was set.

        Parameters:
        - message (str): The message to be set.

        Returns:
        - None: If the time interval has not passed since the last message was set.
        - str: The message that was set.

        """
        cur_time = self.gettime()
        if cur_time - self.last_time > self.delta_time:
            self.last_time = cur_time
            return super().set_message(message)
        return None
    
class EventPubNode(BaseNode):
    """
    A class representing an event publishing node.

    This class inherits from the BaseNode class and provides functionality for setting and getting messages,
    as well as tracking events.

    Attributes:
        name (str): The name of the node.
        type (str): The type of the node.
        event (bool): A flag indicating whether an event has occurred.
        message (Any): The message associated with the event.

    Methods:
        set_message(message: Any) -> None: Sets the message and updates the event flag.
        get_message() -> Any: Returns the message if an event has occurred, otherwise returns None.
        tick() -> bool: Checks if an event has occurred and returns the event flag.

    Example:
        node = EventPubNode("event_node", "publisher")
        node.set_message("Event occurred!")
        message = node.get_message()
        if message:
            print(message)  # Output: Event occurred!
    """

    def __init__(self, name, type) -> None:
        """
        Initializes a Node object.

        Args:
            name (str): The name of the node.
            type (str): The type of the node.

        Returns:
            None
        """
        super().__init__(name, type)
        self.event = False
    
    def set_message(self, message):
        """
        Sets the message for the node and triggers an event.

        Args:
            message: The message to be set.

        Returns:
            The result of the super class's set_message method.
        """
        self.event = True
        return super().set_message(message)
    
    def get_message(self):
        """
        Retrieves the message stored in the node.

        Returns:
            str or None: The message stored in the node if available, None otherwise.
        """
        if self.event:
            self.event = False
            return self.message
        return None
    
    async def tick(self):
        return self.event

class EventSubNode(BaseNode):
    """
    A class representing an event subscriber node.

    This class inherits from the BaseNode class and provides functionality for subscribing to events and retrieving messages.

    Attributes:
        name (str): The name of the node.
        type (str): The type of the node.
        event (bool): A flag indicating whether an event has occurred.
        message (str): The message associated with the event.

    Methods:
        tick: Asynchronously checks if an event has occurred.
        set_message: Sets the message associated with the event.
        get_message: Retrieves the message associated with the event.

    Example:
        node = EventSubNode("event_node", "subscriber")
        node.set_message("Event occurred!")
        message = node.get_message()
        print(message)  # Output: Event occurred!
    """

    def __init__(self, name, type) -> None:
        """
        Initializes a Node object.

        Args:
            name (str): The name of the node.
            type (str): The type of the node.

        Returns:
            None
        """
        super().__init__(name, type)
        self.event = False
    
    async def tick(self):
        return self.event
    
    def set_message(self, message):
        """
        Sets the message for the node.

        This method updates the message attribute of the node with the provided message.
        If the new message is different from the current message, it sets the event flag to True.

        Args:
            message (str): The new message to set.

        Returns:
            None
        """
        if message != self.message:
            self.event = True
            self.message = message
    
    def get_message(self):
        """
        Retrieves the message stored in the node.

        Returns:
            The message stored in the node if there is an event, otherwise None.
        """
        if self.event:
            self.event = False
            return self.message
        return None
