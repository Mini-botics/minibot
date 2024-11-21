from ...urtps import EventPubNode

class TwoWheelPID(EventPubNode):
    """
    A class representing a two-wheel PID controller for a robot.

    This class provides methods to set the desired linear and angular velocities
    of the robot and move it accordingly.

    Example usage:
    ```
    pid_controller = TwoWheelPID()
    pid_controller.move(0.5, 0.2)
    ```
    """

    def __init__(self):
        super().__init__('twoWheelPID', 'publishing')

    def set_message(self, x_linear, z_angular):
        """
        Set the desired linear and angular velocities of the robot.

        Args:
            x_linear (float): The desired linear velocity in the x-axis.
            z_angular (float): The desired angular velocity around the z-axis.

        Returns:
            str: The formatted message containing the desired velocities.
        """
        return super().set_message(f'{x_linear},{z_angular}')
    
    def move(self, x_linear, z_angular):
        """
        Move the robot with the specified linear and angular velocities.

        Args:
            x_linear (float): The desired linear velocity in the x-axis.
            z_angular (float): The desired angular velocity around the z-axis.

        Returns:
            str: The formatted message containing the desired velocities.
        """
        return self.set_message(x_linear, z_angular)