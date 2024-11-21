from ...urtps import EventPubNode

class TwoWheel(EventPubNode):
    """
    Represents a two-wheel robot.

    This class provides methods to control the movement of the robot.

    Args:
        EventPubNode: The base class for event publishing nodes.

    Attributes:
        None

    Example:
        tw = TwoWheel()
        tw.move(0.5, 0.2)
    """

    def __init__(self,name = 'twoWheel'):
        super().__init__(name, 'publishing')

    def set_message(self, x_linear, z_angular):
        """
        Set the message for the robot's movement.

        Args:
            x_linear (float): The linear velocity in the x-axis.
            z_angular (float): The angular velocity in the z-axis.

        Returns:
            str: The formatted message containing the velocities.

        Example:
            message = set_message(0.5, 0.2)
            print(message)  # Output: "0.5,0.2"
        """
        return super().set_message(f'{x_linear},{z_angular}')
    
    def move(self, x_linear, z_angular):
        """
        Move the robot.

        Args:
            x_linear (float): The linear velocity in the x-axis.
            z_angular (float): The angular velocity in the z-axis.

        Returns:
            str: The formatted message containing the velocities.

        Example:
            tw = TwoWheel()
            tw.move(0.5, 0.2)
        """
        return self.set_message(x_linear, z_angular)