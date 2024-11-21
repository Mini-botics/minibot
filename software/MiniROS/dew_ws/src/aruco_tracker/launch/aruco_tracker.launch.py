from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # get_image = Node(
    #     package='aruco_tracker',
    #     executable='get_image',
    #     name='get_image'
    # )

    detect_aruco = Node(
        package='aruco_tracker',
        executable='detect_aruco',
        name='detect_aruco'
    )
    follow_aruco = Node(
        package='aruco_tracker',
        executable='follow_aruco',
        name='follow_aruco'
    )

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
    
    twist_mux_params = os.path.join(get_package_share_directory('mini_robot'), 'config', 'twist_mux.yaml')

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )
    


    return LaunchDescription([
    #   get_image,
      detect_aruco,
      follow_aruco,
      pipe_write,
      pipe_read,
      emergency_button,
      neopixel, 
      twist_mux
    ])