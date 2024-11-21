from launch import LaunchDescription
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Hello from the mini robot. If you read this message, it means that you have started to learn the ROS part of the Mini Robot project. ROS can be seen hard at first seen, but be pationate. You can get it!")
    ])
