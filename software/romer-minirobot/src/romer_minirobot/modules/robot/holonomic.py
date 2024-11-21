from ...urtps import EventPubNode

class Holonomic(EventPubNode):
    """
    A class representing a holonomic robot.

    This class provides methods to set the movement message and move the robot in a holonomic manner.

    Args:
        EventPubNode: The base class for publishing events.

    Attributes:
        None

    Example:
        holonomic_robot = Holonomic()
        holonomic_robot.move(1, 0, 0)  # Moves the robot forward with a linear velocity of 1 in the x-axis.
    """

    def __init__(self, name = 'holonomic'):
        super().__init__(name, 'publishing')

    def set_message(self, x_linear, y_linear, z_angular):
        """
        Set the movement message for the robot.

        Args:
            x_linear (float): The linear velocity in the x-axis.
            y_linear (float): The linear velocity in the y-axis.
            z_angular (float): The angular velocity around the z-axis.

        Returns:
            str: The formatted movement message.

        Example:
            set_message(1, 0, 0)  # Returns '1,0,0' as the movement message.
        """
        return super().set_message(f'{x_linear},{y_linear},{z_angular}')
    
    def move(self, x_linear, y_linear, z_angular):
        """
        Move the robot in a holonomic manner.

        Args:
            x_linear (float): The linear velocity in the x-axis.
            y_linear (float): The linear velocity in the y-axis.
            z_angular (float): The angular velocity around the z-axis.

        Returns:
            str: The formatted movement message.

        Example:
            move(1, 0, 0)  # Returns '1,0,0' as the movement message and moves the robot forward with a linear velocity of 1 in the x-axis.
        """
        return self.set_message(x_linear, y_linear, z_angular)
