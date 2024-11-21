from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    
    pipe_write = Node(
            package='mini_control',
            executable='pipe_write',
            name='pipe_write_node',
            output='screen'
        )
    pipe_read = Node(
            package='mini_control',
            executable='pipe_read',
            name='pipe_read_node',
            output='screen'
        )
    emergency_button = Node(
            package='mini_control',
            executable='emergency_button',
            name='emergency_button_node',
            output='screen'
        )
    
    neopixel =Node(
            package='mini_control',
            executable='neopixel',
            name='neopixel_node',
            output='screen'
        )
    
    
    return LaunchDescription([
        pipe_write,
        pipe_read,
        emergency_button,
        neopixel
    ])
